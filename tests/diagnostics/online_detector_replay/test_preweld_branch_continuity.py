"""生产process回归：现场像素与明确构造的初始化，约束预焊估计及虚实位置偏差。"""
import json
import math
import pathlib
import subprocess
import tempfile
import unittest

from PIL import Image

from native_runtime import configured_native


ROOT = pathlib.Path(__file__).resolve().parents[3]
FIXTURE = ROOT / 'tests/fixtures/preweld_branch_jump_20261028'


class PreweldBranchContinuity(unittest.TestCase):
    def replay(self, shift=(0, 0), frames=None, folder=FIXTURE):
        if frames is None:
            frames = [json.loads(line) for line in (FIXTURE / 'input.jsonl').read_text().splitlines()]
        for frame in frames:
            frame.setdefault('paper_options', {})
            frame['paper_options']['branch_constraints'] = True
            frame['image'] = str(folder / frame['image'])
            for key in ('sim_point', 'reference_point_at_exposure'):
                if frame.get(key):
                    frame[key] = [p + delta for p, delta in zip(frame[key], shift)]
            frame['sim_line'] = [[p + delta for p, delta in zip(point, shift)] for point in frame['sim_line']]
        output = pathlib.Path(tempfile.mkdtemp(prefix='preweld-branch-continuity-'))
        manifest = output / 'input.jsonl'
        manifest.write_text(''.join(json.dumps(row) + '\n' for row in frames), encoding='utf-8')
        binary, _ = configured_native('DetectorReplay')
        process = subprocess.run([str(binary), str(manifest)], capture_output=True, timeout=30)
        self.assertEqual(process.returncode, 0, process.stderr.decode('utf-8', errors='replace'))
        rows = [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
        self.assertEqual(len(rows), len(frames))
        for row in rows:
            row.pop('image_png_base64', None)
        (output / 'results.json').write_text(json.dumps(rows), encoding='utf-8')
        return rows

    def test_degraded_target_stays_in_the_moving_mother_branch_region(self):
        rows = self.replay()[32:]
        self.assertTrue(all(row['final_pixel'] for row in rows), 'Both visible mother branches must retain an estimate')
        step = max(math.dist(a['final_pixel'], b['final_pixel']) for a, b in zip(rows, rows[1:]))
        self.assertLess(step, 8.0, 'This captured local preweld pattern must not make a tens-of-pixels step')
        for row in rows[3:]:
            x, y = row['final_pixel']
            self.assertTrue(450 <= x <= 495 and 580 <= y <= 600,
                            'Scene-specific extrapolation region excludes the visible tack surface and reflection')
        used = [row for row in rows if row['paper_diagnostics']['mother_used']]
        self.assertGreaterEqual(len(used), 10)
        for row in used:
            self.assertFalse(row['anchor_refreshed'] or row['can_independent_validate'])
            self.assertFalse(row['image_observation_accepted'])
            self.assertFalse(row['template_reinitialized'], 'A degraded estimate cannot initialize a new real identity')
            self.assertTrue(row['paper_diagnostics']['training_from_simulation'])

    def test_constant_simulation_offset_does_not_drag_the_real_structure(self):
        base, shifted = self.replay((0, 0)), self.replay((160, -70))
        self.assertTrue(all(a['final_pixel'] and b['final_pixel'] for a, b in zip(base, shifted)))
        self.assertLess(max(math.dist(a['final_pixel'], b['final_pixel']) for a, b in zip(base, shifted)), 0.25)

    def test_clear_seam_can_replace_the_estimate_immediately(self):
        folder = ROOT / 'tests/fixtures/preweld_persistent_drift_20261027'
        frames = [json.loads(line) for line in (folder / 'controlled-sequence.jsonl').read_text().splitlines()]
        provenance = json.loads((folder / 'provenance.json').read_text())
        rows = self.replay(frames=frames, folder=folder)
        for row in rows[-provenance['clear_inputs']:]:
            self.assertTrue(row['detector_valid'] and row['image_observation_accepted'])
            self.assertLess(math.dist(row['final_pixel'], provenance['clear_corner_manual_px']),
                            provenance['tolerance_px'])

    def test_no_current_branches_means_no_structural_estimate(self):
        frames = [json.loads(line) for line in (FIXTURE / 'input.jsonl').read_text().splitlines()]
        folder = pathlib.Path(tempfile.mkdtemp(prefix='preweld-no-support-'))
        blank = folder / 'blank.png'
        Image.new('L', (1292, 964), 14).save(blank)
        missing = dict(frames[-1], frame_id=3240, timestamp_ms=frames[-1]['timestamp_ms'] + 100,
                       image=str(blank), detector_actions=[])
        rows = self.replay(frames=frames + [missing])
        self.assertTrue(rows[-2]['paper_diagnostics']['mother_used'])
        self.assertFalse(rows[-1]['paper_diagnostics']['mother_supported'])
        self.assertFalse(rows[-1]['detector_valid'] or rows[-1]['control_eligible'])

    def test_unknown_or_stale_clock_cannot_reuse_the_structural_model(self):
        original = [json.loads(line) for line in (FIXTURE / 'input.jsonl').read_text().splitlines()]
        for timestamp in (-1, original[-1]['timestamp_ms'] + 13000, original[0]['timestamp_ms'] - 1):
            with self.subTest(timestamp=timestamp):
                frames = json.loads(json.dumps(original))
                frames[-1]['timestamp_ms'] = timestamp
                row = self.replay(frames=frames)[-1]
                self.assertFalse(row['paper_diagnostics']['mother_supported'])


if __name__ == '__main__':
    unittest.main()
