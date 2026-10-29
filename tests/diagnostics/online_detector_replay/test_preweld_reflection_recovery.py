"""真实可见接缝恢复不能被旧局部相关响应挡住；不声明完整现场或空间精度。"""
import hashlib
import json
import math
import pathlib
import re
import subprocess
import tempfile
import unittest

from native_runtime import configured_native

ROOT = pathlib.Path(__file__).resolve().parents[3]
FIXTURE = ROOT / 'tests/fixtures/preweld_reflection_recovery_20261028'


class PreweldReflectionRecoveryTests(unittest.TestCase):
    def test_stale_reference_does_not_starve_current_image_search(self):
        binary, _ = configured_native('DetectorReplay')
        original = [json.loads(line) for line in (FIXTURE / 'case.jsonl').read_text().splitlines()]
        frames = original[:2]
        for index in range(12):
            frame = dict(original[-1])
            frame.update(frame_id=100 + index, timestamp_ms=10000 + index * 100,
                         sim_point=[1000, 300], reference_point_at_exposure=[1000, 300],
                         reference_exposure_frame_id=100 + index)
            frames.append(frame)
        for frame in frames:
            frame['image'] = str(FIXTURE / frame['image'])
        folder = pathlib.Path(tempfile.mkdtemp(prefix='i2r-stale-reference-'))
        manifest = folder / 'input.jsonl'
        manifest.write_text(''.join(json.dumps(row) + '\n' for row in frames), encoding='utf-8')
        process = subprocess.run([str(binary), str(manifest)], capture_output=True, timeout=30)
        self.assertEqual(process.returncode, 0, process.stderr.decode('utf-8', errors='replace'))
        rows = [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
        self.assertEqual(len(rows), len(frames))
        centers = set()
        for row in rows[2:]:
            self.assertLessEqual(row['refinement_calls'], 2, 'Recovery must retain the per-frame work budget')
            centers.update(re.findall(r'center=([\d.-]+,[\d.-]+);', row['image_search_diagnostic']))
        self.assertGreaterEqual(len(centers), 3,
                                'An unusable fixed hint must leave opportunities to search other image regions')

    def test_visible_seam_recovers_with_three_templates_after_history_displacement(self):
        binary, _ = configured_native('DetectorReplay')
        provenance = json.loads((FIXTURE / 'provenance.json').read_text(encoding='utf-8'))
        for name, evidence in provenance['files'].items():
            self.assertEqual(hashlib.sha256((FIXTURE / name).read_bytes()).hexdigest(),
                             evidence['sha256'])
        frames = [json.loads(line) for line in (FIXTURE / 'case.jsonl').read_text().splitlines()]
        for frame in frames:
            frame['image'] = str(FIXTURE / frame['image'])
        # 测试产物保留，不清理用户证据。
        folder = pathlib.Path(tempfile.mkdtemp(prefix='i2r-reflection-recovery-'))
        manifest = folder / 'input.jsonl'
        manifest.write_text(''.join(json.dumps(row) + '\n' for row in frames), encoding='utf-8')
        process = subprocess.run([str(binary), str(manifest)], capture_output=True, timeout=30)
        self.assertEqual(process.returncode, 0, process.stderr.decode('utf-8', errors='replace'))
        rows = [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
        self.assertEqual(len(rows), len(frames))
        for row in rows:
            row.pop('image_png_base64', None)
        (folder / 'results.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
        self.assertEqual(max(row['response_template_count'] for row in rows), 3)
        proposal = rows[2]
        if not proposal['anchor_refreshed']:
            self.assertFalse(proposal['image_observation_accepted'],
                             'An unconfirmed recovery proposal cannot replace the trusted identity')
            self.assertFalse(proposal['control_eligible'],
                             'A recovery proposal needs current identity evidence before control')
        recovered = rows[3]
        self.assertTrue(recovered['detector_valid'] and recovered['final_pixel'],
                        'Visible seam must recover rather than merely lose output')
        self.assertLessEqual(math.dist(recovered['final_pixel'], provenance['expected_visible_corner_pixel']),
                             provenance['tolerance_pixel'],
                             'A high local shoulder score cannot retain the displaced old position')
        self.assertTrue(recovered['control_eligible'] and recovered['image_observation_accepted'])
        self.assertEqual(rows[-1]['response_template_count'], 3,
                         'All three response templates must repopulate after confirmed recovery')


if __name__ == '__main__':
    unittest.main()
