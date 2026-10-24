"""Behaviour checks at the public image-session replay boundary."""
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent

class ReplayTests(unittest.TestCase):
    def test_visible_then_missing_image_preserves_exposure_and_rejects_output(self):
        root = pathlib.Path(tempfile.mkdtemp(prefix="kcf-replay-test-"))
        # Independent fixture: exact triangular stripe vertex at (360, 260).
        image = bytes(225 if abs(y - (260 - .45 * abs(x - 360))) < 1.5 else 14
                      for y in range(480) for x in range(640))
        (root / 'visible.pgm').write_bytes(b'P5\n640 480\n255\n' + image)
        records = [dict(image='visible.pgm', frame_id=101, timestamp_ms=5000,
                        reference_revision=7, kind='synthetic',
                        projection={'fixture': 'synthetic_identity_1mm_per_pixel'}),
                   dict(image=None, frame_id=102, timestamp_ms=5100,
                        reference_revision=8, kind='synthetic')]
        manifest = root / 'session.jsonl'
        manifest.write_text(''.join(json.dumps(row) + '\n' for row in records))
        run = subprocess.run([sys.executable, str(HERE / 'replay.py'),
                              str(manifest), '--output', str(root / 'result')],
                             text=True, capture_output=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        rows = [json.loads(row) for row in (root/'result/results.jsonl').read_text().splitlines()]
        self.assertTrue(rows[0]['available'])
        self.assertLess(abs(rows[0]['final_pixel'][0] - 360), 3)
        self.assertLess(abs(rows[0]['final_pixel'][1] - 260), 3)
        self.assertEqual(rows[0]['source'], 'geometry')
        self.assertEqual(rows[0]['frame_id'], 101)
        self.assertEqual(rows[0]['reference_revision'], 7)
        self.assertFalse(rows[1]['available'])
        self.assertIsNone(rows[1]['final_pixel'])
        self.assertEqual(rows[1]['frame_id'], 102)
        self.assertIn('data:image/png;base64,', (root/'result/viewer.html').read_text())
        repeat = subprocess.run([sys.executable, str(HERE/'replay.py'),
                                 str(root/'result/manifest.jsonl'),
                                 '--output', str(root/'repeat')], text=True, capture_output=True)
        self.assertEqual(repeat.returncode, 0, repeat.stderr)
        repeated = [json.loads(line) for line in (root/'repeat/results.jsonl').read_text().splitlines()]
        for before, after in zip(rows, repeated):
            for key in ('available', 'final_pixel', 'raw_kcf_pixel', 'source', 'refinement_status'):
                self.assertEqual(before[key], after[key], key)


    def test_fractional_exposure_identity_is_not_silently_rounded(self):
        root = pathlib.Path(tempfile.mkdtemp(prefix="kcf-replay-invalid-"))
        manifest = root/'invalid.jsonl'
        manifest.write_text(json.dumps({'frame_id': 1.5, 'image': None})+'\n')
        run = subprocess.run([sys.executable, str(HERE/'replay.py'), str(manifest),
                              '--output', str(root/'result')], capture_output=True, text=True)
        self.assertNotEqual(run.returncode, 0)
        self.assertIn('frame_id', run.stderr)
        self.assertFalse((root/'result/results.jsonl').exists())

    def test_real_kcf_updates_then_blank_is_unavailable(self):
        root = pathlib.Path(tempfile.mkdtemp(prefix="kcf-replay-baseline-"))
        pixels = bytearray(225 if abs(y - (260 - .45 * abs(x - 360))) < 1.5 else 14
                           for y in range(480) for x in range(640))
        (root/'visible.pgm').write_bytes(b'P5\n640 480\n255\n' + pixels)
        (root/'blank.pgm').write_bytes(b'P5\n640 480\n255\n' + bytes([14])*640*480)
        rows = [dict(image=name, frame_id=index + 10, kind='synthetic',
                     projection={'fixture': 'synthetic_identity_1mm_per_pixel'})
                for index, name in enumerate(['visible.pgm', 'visible.pgm', 'blank.pgm'])]
        # Missing source archive time/revision must remain unknown, not fabricated zero.
        manifest = root/'input.jsonl'
        manifest.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        run = subprocess.run([sys.executable, str(HERE/'replay.py'), str(manifest),
                              '--output', str(root/'result')], capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        results = [json.loads(row) for row in (root/'result/results.jsonl').read_text().splitlines()]
        self.assertTrue(results[1]['available'])
        self.assertIsNotNone(results[1]['raw_kcf_pixel'])
        self.assertIsNotNone(results[1]['kcf_response_peak'])
        self.assertIsNone(results[1]['timestamp_ms'])
        self.assertIsNone(results[1]['reference_revision'])
        self.assertFalse(results[2]['available'])
        self.assertFalse(results[2]['training_committed'])
        self.assertTrue(results[2]['message'])

    def test_gap_request_fails_explicitly(self):
        root = pathlib.Path(tempfile.mkdtemp(prefix="kcf-replay-gap-"))
        manifest = root/'input.jsonl'
        manifest.write_text(json.dumps({'image': None, 'enable_gap': True})+'\n')
        run = subprocess.run([sys.executable, str(HERE/'replay.py'), str(manifest),
                              '--output', str(root/'result')], capture_output=True, text=True)
        self.assertNotEqual(run.returncode, 0)
        self.assertIn('gap excluded', run.stderr)

if __name__ == '__main__':
    unittest.main()
