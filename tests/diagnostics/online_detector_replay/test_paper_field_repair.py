"""Production detector regressions for captured wrong-target and recovery patterns."""
import json
import math
import pathlib
import struct
import subprocess
import tempfile
import unittest

from native_runtime import configured_native


ROOT = pathlib.Path(__file__).resolve().parents[3]
FIXTURE = ROOT / 'tests/fixtures/paper_tracking_20261027'


class PaperFieldRepairTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = configured_native('DetectorReplay')[0]

    def replay(self, frames):
        folder = pathlib.Path(tempfile.mkdtemp(prefix='paper-field-repair-'))
        normalized = [dict(frame, image=str(FIXTURE / frame['image']), paper_tracking=True)
                      for frame in frames]
        manifest = folder / 'manifest.jsonl'
        manifest.write_text(''.join(json.dumps(frame) + '\n' for frame in normalized), encoding='utf-8')
        process = subprocess.run([str(self.binary), str(manifest)], capture_output=True, timeout=35)
        self.assertEqual(process.returncode, 0, process.stderr.decode('utf-8', errors='replace'))
        rows = [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
        for row in rows:
            row.pop('image_png_base64', None)
        (folder / 'results.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')
        return rows

    def load(self, name):
        return [json.loads(line) for line in (FIXTURE / name).read_text(encoding='utf-8').splitlines()]

    def test_first_reverse_exposure_acquires_left_corner(self):
        row = self.replay(self.load('initial.jsonl'))[0]
        self.assertTrue(row['detector_valid'])
        self.assertLess(math.dist(row['detector_pixel'], [470, 680]), 15)

    def test_blank_simulation_proposal_cannot_poison_first_real_model(self):
        first = self.load('initial.jsonl')[0]
        width, height = struct.unpack('>II', (FIXTURE / first['image']).read_bytes()[16:24])
        folder = pathlib.Path(tempfile.mkdtemp(prefix='paper-blank-bootstrap-'))
        blank = folder / 'blank.pgm'
        blank.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + bytes(width * height))
        frames = [dict(first, image=str(blank), frame_id=first['frame_id'] - 1,
                       sim_line=[[308, 677], [468, 677]]), first]
        rows = self.replay(frames)
        self.assertFalse(rows[0]['detector_valid'])
        self.assertTrue(rows[1]['detector_valid'])
        self.assertLess(math.dist(rows[1]['detector_pixel'], [470, 680]), 15)

    def test_geometry_confirmation_does_not_become_persistent_motion(self):
        rows = self.replay(self.load('online.jsonl'))
        scored = {2539: (547, 591), 2548: (544, 590), 2566: (539, 591), 2665: (516, 589)}
        for row in rows:
            if row['frame_id'] in scored:
                self.assertTrue(row['detector_valid'], str(row['frame_id']))
                self.assertLess(math.dist(row['detector_pixel'], scored[row['frame_id']]), 15,
                                'captured left corner lost at ' + str(row['frame_id']))
        self.assertGreaterEqual(sum(row['detector_valid'] for row in rows[2:]), 13)

    def test_valid_simulation_is_not_vetoed_by_raw_gray_correlation(self):
        frame = self.load('initial.jsonl')[0]
        frame['sim_line'] = [[308, 677], [468, 677]]
        rows = self.replay([frame, dict(frame, frame_id=frame['frame_id'] + 1,
                                       timestamp_ms=frame['timestamp_ms'] + 100)])
        self.assertTrue(all(row['simulation_shape_available'] for row in rows))
        self.assertTrue(rows[0]['training_committed'])
        self.assertEqual(rows[1]['response_template_count'], 3)

    def test_single_template_can_collect_normal_preweld_statistics(self):
        first = self.load('initial.jsonl')[0]
        frames = [dict(first, frame_id=first['frame_id'] + index,
                       timestamp_ms=first['timestamp_ms'] + index * 100)
                  for index in range(34)]
        rows = self.replay(frames)
        self.assertTrue(all(row['detector_valid'] for row in rows))
        self.assertGreaterEqual(rows[-1]['paper_diagnostics']['normal_response_samples'], 30)
        self.assertTrue(rows[-1]['paper_diagnostics']['response_statistics_ready'])


if __name__ == '__main__':
    unittest.main()
