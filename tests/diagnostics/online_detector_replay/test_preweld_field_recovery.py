"""Real field history must not hide a currently visible mother-material corner."""
import hashlib
import json
import math
import pathlib
import subprocess
import tempfile
import unittest

from native_runtime import configured_native


ROOT = pathlib.Path(__file__).resolve().parents[3]
FIXTURE = ROOT / 'tests/fixtures/preweld_tracking_20261027'


class PreweldFieldRecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = configured_native('DetectorReplay')[0]
        cls.provenance = json.loads((FIXTURE / 'provenance.json').read_text(encoding='utf-8'))
        for filename, record in cls.provenance['images'].items():
            if hashlib.sha256((FIXTURE / filename).read_bytes()).hexdigest() != record['sha256']:
                raise AssertionError('Captured image hash mismatch: ' + filename)

    def test_visible_corner_is_not_replaced_by_a_high_scoring_old_branch(self):
        frames = [json.loads(line) for line in
                  (FIXTURE / 'history.jsonl').read_text(encoding='utf-8').splitlines()]
        for frame in frames:
            frame['image'] = str(FIXTURE / frame['image'])
        folder = pathlib.Path(tempfile.mkdtemp(prefix='preweld-history-'))
        manifest = folder / 'manifest.jsonl'
        manifest.write_text(''.join(json.dumps(frame) + chr(10) for frame in frames), encoding='utf-8')
        process = subprocess.run([str(self.binary), str(manifest)], capture_output=True, timeout=20)
        self.assertEqual(process.returncode, 0, process.stderr.decode('utf-8', errors='replace'))
        rows = [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
        for row in rows:
            row.pop('image_png_base64', None)
        (folder / 'results.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
        self.assertEqual(len(rows), len(frames))
        final = rows[-1]
        self.assertTrue(final['detector_valid'], 'Visible captured seam must recover, not just be suppressed')
        self.assertLess(math.dist(final['detector_pixel'], self.provenance['manual_corner_px']['4727']),
                        self.provenance['symptom_tolerance_px'],
                        'Old local appearance must not replace the visible left seam corner')

    def test_unsupported_local_correction_cannot_drive_without_reference_recovery(self):
        frames = [json.loads(line) for line in
                  (FIXTURE / 'history.jsonl').read_text(encoding='utf-8').splitlines()]
        for frame in frames:
            frame['image'] = str(FIXTURE / frame['image'])
        for field in ['sim_point', 'reference_point_at_exposure', 'reference_exposure_frame_id']:
            frames[-1].pop(field, None)
        folder = pathlib.Path(tempfile.mkdtemp(prefix='preweld-reference-gap-'))
        manifest = folder / 'manifest.jsonl'
        manifest.write_text(''.join(json.dumps(frame) + chr(10) for frame in frames), encoding='utf-8')
        process = subprocess.run([str(self.binary), str(manifest)], capture_output=True, timeout=20)
        self.assertEqual(process.returncode, 0, process.stderr.decode('utf-8', errors='replace'))
        rows = [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
        final = rows[-1]
        point = final['detector_pixel']
        if point is None or math.dist(point, self.provenance['manual_corner_px']['4727']) > 20:
            self.assertFalse(final['control_eligible'],
                             'An unsupported old-branch correction is not a usable control point')
            self.assertFalse(final['image_observation_accepted'])


if __name__ == '__main__':
    unittest.main()
