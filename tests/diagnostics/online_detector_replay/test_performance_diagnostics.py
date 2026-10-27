"""真实 process 的计时快照；不把稀疏实拍回放当作现场连续曝光。"""
import json
import math
import pathlib
import subprocess
import tempfile
import unittest

from native_runtime import configured_native


ROOT = pathlib.Path(__file__).resolve().parents[3]
FIXTURE = ROOT / 'tests/fixtures/paper_tracking_20261027'


class PerformanceDiagnosticsTests(unittest.TestCase):
    def test_real_process_reports_exclusive_stages_without_changing_result(self):
        binary = configured_native('DetectorReplay')[0]
        frames = [json.loads(line) for line in
                  (FIXTURE / 'online.jsonl').read_text(encoding='utf-8-sig').splitlines()][:3]
        folder = ROOT / 'output/analysis/online_correction_optimization'
        folder.mkdir(parents=True, exist_ok=True)
        run = pathlib.Path(tempfile.mkdtemp(prefix='performance-contract-', dir=folder))
        results = []
        for enabled in [False, True]:
            manifest = run / f'input-{int(enabled)}.jsonl'
            inputs = [dict(frame, image=str((FIXTURE / frame['image']).resolve()),
                           paper_tracking=True, performance_diagnostics=enabled)
                      for frame in frames]
            manifest.write_text(''.join(json.dumps(frame) + '\n' for frame in inputs),
                                encoding='utf-8')
            process = subprocess.run([str(binary), str(manifest)], capture_output=True, timeout=30)
            self.assertEqual(process.returncode, 0, process.stderr.decode('utf-8', errors='replace'))
            rows = [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
            for row in rows:
                row.pop('image_png_base64', None)
            results.append(rows)
        self.assertEqual(len(results[1]), 3)
        for ordinary, measured in zip(*results):
            self.assertIn('performance', measured,
                          'Public process replay must export measured stage durations')
            self.assertIsNone(ordinary['performance'])
            for field in ['detector_valid', 'detector_pixel', 'source', 'control_eligible',
                          'training_committed', 'response_template_count']:
                if field in ordinary:
                    self.assertEqual(ordinary[field], measured[field], field)
            timing = measured['performance']
            stages = [timing[name] for name in ['input_bookkeeping_ms', 'simulation_ms',
                      'initialization_ms', 'tracking_ms', 'validation_projection_ms', 'training_ms']]
            self.assertTrue(all(math.isfinite(value) and value >= 0 for value in stages))
            self.assertGreater(timing['total_ms'], 0)
            self.assertLessEqual(sum(stages), timing['total_ms'] + 0.01)
            self.assertLess(timing['total_ms'] - sum(stages), 5,
                            'Stage accounting must cover the detector call, without nested double counting')
        self.assertGreater(results[1][-1]['performance']['training_ms'], 0)


if __name__ == '__main__':
    unittest.main()
