"""Captured-exposure regressions: no duplicated frames, devices or fabricated poses."""
import json
import pathlib
import subprocess
import tempfile
import unittest

from native_runtime import configured_native

ROOT = pathlib.Path(__file__).resolve().parents[3]
FIXTURE = ROOT / 'tests/fixtures/online_evidence_motion_20261026'


class FieldMotionRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = configured_native('DetectorReplay')[0]

    def replay(self, name, fixture=FIXTURE, current_thresholds=False):
        rows = [json.loads(line) for line in (fixture / (name + '.jsonl')).read_text(encoding='utf-8').splitlines()]
        for row in rows:
            row['image'] = str(fixture / row['image'])
            if current_thresholds:
                # The new program uses relaxed defaults; retain the archived thresholds in the fixture.
                for key in ['identity_min_patch_score', 'identity_min_branch_score',
                            'identity_min_uniqueness', 'max_recovery_step_pixel']:
                    row['evidence_options'].pop(key, None)
        # Keep the recorded 12 px setting: the state-machine fix must work without loosening it.
        folder = pathlib.Path(tempfile.mkdtemp(prefix='field-motion-'))
        manifest = folder / 'manifest.jsonl'
        manifest.write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')
        process = subprocess.run([str(self.binary), str(manifest)], capture_output=True, timeout=30)
        self.assertEqual(process.returncode, 0, process.stderr.decode('utf-8', errors='replace'))
        return [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]

    def test_confirmed_identity_survives_scan_motion(self):
        rows = self.replay('initial')
        self.assertFalse(rows[0]['control_eligible'])
        for row in rows[1:]:
            self.assertTrue(row['control_eligible'], 'visible captured seam rejected at frame ' + str(row['frame_id']))
            self.assertIsNotNone(row['raw_measurement_pixel'])
            self.assertEqual(row['trust'], 'independent_geometry')

    def test_saved_occlusion_window_has_continuous_following_output(self):
        rows = self.replay('online', ROOT / 'tests/fixtures/kcf_continuity_20261026')
        self.assertTrue(rows[1]['control_eligible'])
        previous = rows[1]['final_pixel']
        for row in rows[2:]:
            self.assertTrue(row['available'] and row['control_eligible'], str(row['frame_id']))
            self.assertIsNotNone(row['positioner_tcp_mm'])
            point = row['final_pixel']
            self.assertLessEqual(sum((a-b)**2 for a,b in zip(point, previous)), 8**2 + 1e-6)
            if row['source'] != 'geometry':
                self.assertFalse(row['can_independent_validate'])
                self.assertFalse(row['training_committed'])
            previous = point

    def test_latest_reverse_tail_regains_visible_image_geometry(self):
        rows = self.replay('initial', ROOT / 'tests/fixtures/kcf_reacquisition_20261026')
        self.assertGreaterEqual(sum(r['control_eligible'] for r in rows[-8:]), 6)
        self.assertTrue(rows[-1]['anchor_refreshed'])

    def test_reflective_online_samples_can_refresh_image_identity_again(self):
        # Sparse saved images test pixel identity only; their projection is a labelled fixture.
        rows = self.replay('online', ROOT / 'tests/fixtures/kcf_reacquisition_20261026', current_thresholds=True)
        self.assertTrue(all(r['refinement_calls'] <= 2 for r in rows[3:]))
        self.assertTrue(any(r['anchor_refreshed'] for r in rows[-2:]),
                        'visible raw seam remains locked behind the old appearance template')

    def test_bounded_search_recovers_before_window_expires(self):
        rows = self.replay('online')
        self.assertTrue(all(row['refinement_calls'] <= 2 for row in rows))
        self.assertTrue(any(row['control_eligible'] for row in rows[:5]),
                        'bounded search starved a visible seam during the first five real exposures')
        self.assertTrue(rows[-1]['control_eligible'], 'last real exposure must recover')
        for row in rows:
            if row['control_eligible']:
                self.assertLess(abs(row['raw_measurement_pixel'][0] - 390), 6)
                self.assertLess(abs(row['raw_measurement_pixel'][1] - 618), 6)


if __name__ == '__main__':
    unittest.main()
