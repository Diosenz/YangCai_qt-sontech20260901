"""Captured online exposures exercise the public detector, without synthetic images."""
import hashlib
import json
import math
import pathlib
import statistics
import subprocess
import tempfile
import unittest
from PIL import Image, ImageOps

from native_runtime import configured_native


ROOT = pathlib.Path(__file__).resolve().parents[3]
FIXTURE = ROOT / 'tests/fixtures/online_tracking_20261027'


class OnlineFieldBootstrapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = configured_native('DetectorReplay')[0]
        provenance = json.loads((FIXTURE / 'provenance.json').read_text(encoding='utf-8'))
        for name, record in provenance['images'].items():
            actual = hashlib.sha256((FIXTURE / name).read_bytes()).hexdigest()
            if actual != record['sha256']:
                raise AssertionError('Captured fixture hash mismatch: ' + name)

    def exposure(self, session, frame_id):
        context = next(json.loads(line) for line in
            (FIXTURE / 'exposures.jsonl').read_text(encoding='utf-8').splitlines()
            if json.loads(line)['frame_id'] == frame_id
            and json.loads(line)['capture_session'] == session)
        context['image'] = str(FIXTURE / context['image'])
        return context

    def replay(self, frames):
        folder = pathlib.Path(tempfile.mkdtemp(prefix='online-field-bootstrap-'))
        manifest = folder / 'manifest.jsonl'
        manifest.write_text(''.join(json.dumps(frame) + '\n' for frame in frames), encoding='utf-8')
        process = subprocess.run([str(self.binary), str(manifest)], capture_output=True, timeout=60)
        self.assertEqual(process.returncode, 0, process.stderr.decode('utf-8', errors='replace'))
        rows = [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
        for row in rows:
            row.pop('image_png_base64', None)
        (folder / 'results.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
        return rows

    def test_unconfirmed_online_proposal_cannot_become_control_target(self):
        for session, frame_id in [('20261027_123015_913', 7264),
                                  ('20261027_122726_191', 2070)]:
            with self.subTest(frame_id=frame_id):
                row = self.replay([self.exposure(session, frame_id)])[0]
                if row['detector_valid']:
                    self.assertLess(math.dist(row['detector_pixel'], [496, 585]), 15,
                        'Actual left seam corner must not be replaced by a self-matching proposal')
                else:
                    self.assertFalse(row['control_eligible'])
                    self.assertEqual(row['trust'], 'unavailable')
                    self.assertIsNone(row['positioner_tcp_mm'])
                    self.assertFalse(row['training_committed'])

    def test_real_online_exposures_acquire_visible_left_corner(self):
        frame_ids = [3801, 3830, 3856, 3884, 3910, 3937, 3969, 3999,
                     4031, 4061, 4087, 4115, 4143, 4171]
        rows = self.replay([self.exposure('20261027_122726_191', frame_id)
                            for frame_id in frame_ids])
        acquired = [row for row in rows if row['detector_valid']]
        self.assertTrue(acquired, 'Visible seam must be acquired, not indefinitely suppressed')
        frame_3937 = next(row for row in rows if row['frame_id'] == 3937)
        self.assertTrue(frame_3937['detector_valid'])
        self.assertLess(math.dist(frame_3937['detector_pixel'], [550, 585]), 15)
        landmarks = {3856: [540, 585], 4031: [535, 587], 4171: [525, 588]}
        for frame_id, point in landmarks.items():
            row = next(row for row in rows if row['frame_id'] == frame_id)
            self.assertTrue(row['detector_valid'], 'Visible captured seam lost at ' + str(frame_id))
            self.assertLess(math.dist(row['detector_pixel'], point), 15)

    def test_opposite_corner_orientation_does_not_require_convex_geometry(self):
        fixture = ROOT / 'tests/fixtures/paper_tracking_20261027'
        frame = json.loads((fixture / 'online.jsonl').read_text(encoding='utf-8-sig').splitlines()[0])
        folder = pathlib.Path(tempfile.mkdtemp(prefix='online-field-orientation-'))
        with Image.open(fixture / frame['image']) as source:
            height = source.height
            ImageOps.flip(source).save(folder / 'reoriented_1712.png')
        frame.update(image=str(folder / 'reoriented_1712.png'), paper_tracking=True,
                     bounded_search=True)
        frame['sim_point'][1] = height - 1 - frame['sim_point'][1]
        frame['reference_point_at_exposure'][1] = height - 1 - frame['reference_point_at_exposure'][1]
        if 'laser_line' in frame:
            frame['laser_line'] = [[x, height - 1 - y] for x, y in frame['laser_line']]
        row = self.replay([frame])[0]
        self.assertTrue(row['detector_valid'],
                        'Reorienting a captured corner must not permanently prevent image tracking')
        self.assertLess(math.dist(row['detector_pixel'], [504, height - 1 - 585]), 15)
        self.assertFalse(row['can_independent_validate'],
                         'Bootstrap identity confirmation is not independent convex seam geometry')

    def test_real_image_three_template_training_budget(self):
        frame = self.exposure('20261027_122726_191', 3937)
        process = subprocess.run([str(self.binary), '--tracker-benchmark', frame['image']],
                                 capture_output=True, timeout=30)
        self.assertEqual(process.returncode, 0, process.stderr.decode('utf-8', errors='replace'))
        rows = [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
        folder = pathlib.Path(tempfile.mkdtemp(prefix='online-field-timing-'))
        (folder / 'timing.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
        self.assertEqual(len(rows), 4)
        self.assertTrue(all(row['templates'] == 3 and row['training_committed'] for row in rows[1:]))
        baseline = json.loads((FIXTURE / 'training_reference.json').read_text(encoding='utf-8'))
        for actual, expected in zip(rows, baseline):
            for field in ['response_peak', 'response_psr', 'hog_weight', 'gray_weight']:
                self.assertAlmostEqual(actual[field], expected[field], places=6,
                                       msg='Optimization changes ' + field)
            self.assertLess(math.dist(actual['point'], expected['point']), 0.001)
        median_ms = statistics.median(row['training_ms'] for row in rows[1:])
        print(f'Real image training median: {median_ms:.1f} ms; evidence: {folder}', flush=True)
        self.assertLess(median_ms, 250, 'Local Debug regression budget, not a production safety threshold')


if __name__ == '__main__':
    unittest.main()
