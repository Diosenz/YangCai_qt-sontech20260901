"""Public detector recovery with captured pixels and explicitly constructed history."""
import copy
import json
import math
import pathlib
import subprocess
import tempfile
import unittest

import numpy as np
from PIL import Image

from native_runtime import configured_native


ROOT = pathlib.Path(__file__).resolve().parents[3]
FIXTURE = ROOT / 'tests/fixtures/preweld_branch_jump_20261028'


class PreweldReferenceContinuity(unittest.TestCase):
    def replay(self, frames):
        binary, _ = configured_native('DetectorReplay')
        folder = pathlib.Path(tempfile.mkdtemp(prefix='preweld-reference-continuity-'))
        manifest = folder / 'input.jsonl'
        manifest.write_text(''.join(json.dumps(frame) + '\n' for frame in frames),
                            encoding='utf-8')
        result = subprocess.run([str(binary), str(manifest)], capture_output=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr.decode('utf-8', errors='replace'))
        rows = [json.loads(line) for line in result.stdout.splitlines()]
        for row in rows:
            row.pop('image_png_base64', None)
        (folder / 'results.json').write_text(json.dumps(rows), encoding='utf-8')
        self.assertEqual(len(rows), len(frames))
        return rows

    def warmup(self):
        frames = [json.loads(line) for line in
                  (FIXTURE / 'input.jsonl').read_text(encoding='utf-8').splitlines()][:32]
        for frame in frames:
            frame['image'] = str(FIXTURE / frame['image'])
            frame.setdefault('paper_options', {})['branch_constraints'] = True
        return frames

    def test_clear_seam_recovers_current_observation_after_search_started(self):
        frames = self.warmup()
        folder = pathlib.Path(tempfile.mkdtemp(prefix='preweld-short-loss-'))
        blank = folder / 'blank.png'
        Image.new('L', (1292, 964), 14).save(blank)
        for index in range(4):
            frame = copy.deepcopy(frames[31])
            frame.update(frame_id=4000 + index,
                         timestamp_ms=frames[31]['timestamp_ms'] + 100 * (index + 1),
                         bounded_search=True, detector_actions=[])
            if index == 0:
                frame['image'] = str(blank)
            frames.append(frame)
        rows = self.replay(frames)
        self.assertFalse(rows[32]['control_eligible'], 'Blank pixels cannot supply a control point')
        final = rows[-1]
        self.assertTrue(final['detector_valid'] and final['image_observation_accepted'],
                        'A clear original seam must recover current observation after brief loss')
        self.assertFalse(final['paper_diagnostics']['mother_used'],
                         'Recovered clear geometry must replace a degraded structural estimate')
        x, y = final['final_pixel']
        self.assertTrue(450 <= x <= 475 and 580 <= y <= 600,
                        'The visible captured corner must remain in its image landmark region')

    def test_virtual_workpiece_revision_does_not_drop_current_real_branches(self):
        frames = [json.loads(line) for line in
                  (FIXTURE / 'input.jsonl').read_text(encoding='utf-8').splitlines()]
        for frame in frames:
            frame['image'] = str(FIXTURE / frame['image'])
            frame.setdefault('paper_options', {})['branch_constraints'] = True
        base = self.replay(frames)
        changed = copy.deepcopy(frames)
        for frame in changed:
            if frame['frame_id'] >= 3204:
                frame['reference_revision'] = 2
        refreshed = self.replay(changed)
        for first, second in zip(base[32:], refreshed[32:]):
            self.assertIsNotNone(first['final_pixel'])
            self.assertIsNotNone(second['final_pixel'])
            self.assertLess(math.dist(first['final_pixel'], second['final_pixel']), 0.25,
                            'Virtual pose metadata cannot move a target constrained by the same real pixels')

    def test_growing_tack_cannot_retrain_the_mother_seam_identity(self):
        folder = pathlib.Path(tempfile.mkdtemp(prefix='preweld-growing-tack-'))
        anchor = self.warmup()[0]
        frames = []
        xs = np.arange(1292, dtype=float)
        ys = np.arange(964, dtype=float)[:, None]
        for index in range(43):
            # 真实母材接缝固定在(490,600)，预焊仅改变中心160列的可见表面。
            drift = max(0, index - 11) * 0.5 if index < 40 else 0.0
            center = 490 + drift
            stripe = 600 - np.abs(xs - 490) * 0.32
            left = (xs >= 410) & (xs <= center)
            right = (xs > center) & (xs <= 570)
            stripe[left] = 574.4 + (600 - drift * 0.4 - 574.4) * (xs[left] - 410) / (center - 410)
            stripe[right] = 600 - drift * 0.4 + (574.4 - 600 + drift * 0.4) * (xs[right] - center) / (570 - center)
            pixels = (14 + 225 * np.exp(-((ys - stripe) ** 2) / 3.38)).astype(np.uint8)
            image = folder / ('frame-%d.png' % index)
            Image.fromarray(pixels).save(image, compress_level=1)
            frame = copy.deepcopy(anchor)
            frame.update(kind='synthetic', frame_id=index + 1, timestamp_ms=1000 + index * 100,
                         image=str(image), bounded_search=index >= 10,
                         detector_actions=['reset'] if index == 0 else [],
                         sim_point=[490, 600], reference_point_at_exposure=[490, 600],
                         reference_exposure_frame_id=index + 1,
                         sim_line=[[0, 443.2], [490, 600], [1291, 343.68]])
            frames.append(frame)
        rows = self.replay(frames)
        self.assertTrue(all(r['final_pixel'] for r in rows),
                        'Visible current mother branches must retain a target estimate')
        self.assertLess(max(math.dist(r['final_pixel'], [490, 600]) for r in rows), 5.0,
                        'A growing tack surface cannot move the stationary mother seam or calibrate itself as that seam')
        self.assertLess(max(math.dist(a['final_pixel'], b['final_pixel'])
                            for a, b in zip(rows, rows[1:])), 5.0,
                        'Entering and leaving the local tack must not alternate between different target identities')
        self.assertTrue(rows[-1]['image_observation_accepted'],
                        'A genuinely clear original seam must immediately recover current observation')


if __name__ == '__main__':
    unittest.main()
