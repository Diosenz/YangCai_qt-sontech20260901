"""近静态真实预焊入口的连续性回归；4px仅为该短片段的诊断要求。"""
import json
import math
import pathlib
import subprocess
import tempfile
import unittest

from native_runtime import configured_native

ROOT = pathlib.Path(__file__).resolve().parents[3]


class PreweldParameterStability(unittest.TestCase):
    def test_captured_near_stationary_tack_onset_does_not_make_a_six_pixel_step(self):
        folder = ROOT/'tests/fixtures/online_tack_instability_20261027'
        frames = [json.loads(line) for line in (folder/'input.jsonl').read_text().splitlines()]
        for frame in frames:
            frame['image'] = str(folder/frame['image'])
        out = pathlib.Path(tempfile.mkdtemp(prefix='preweld-parameters-'))
        manifest = out/'input.jsonl'
        manifest.write_text(''.join(json.dumps(f)+'\n' for f in frames), encoding='utf-8')
        binary, _ = configured_native('DetectorReplay')
        process = subprocess.run([str(binary),str(manifest)],capture_output=True,timeout=20)
        self.assertEqual(process.returncode,0)
        rows=[json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
        self.assertEqual(len(rows),3)
        self.assertTrue(all(r['detector_valid'] and r['control_eligible'] for r in rows))
        self.assertLess(max(math.dist(a['final_pixel'],b['final_pixel'])
            for a,b in zip(rows,rows[1:])),4.0,
            'Near-stationary tack onset must remain locally stable while retaining usable output')

    def test_reference_translation_without_icp_does_not_shift_the_detected_seam(self):
        folder = ROOT/'tests/fixtures/online_tack_instability_20261027'
        original = [json.loads(line) for line in (folder/'input.jsonl').read_text().splitlines()]
        binary, _ = configured_native('DetectorReplay')
        points = []
        for dx,dy in [(0,0),(160,-70),(-160,70)]:
            frames = json.loads(json.dumps(original))
            for frame in frames:
                frame['image'] = str(folder/frame['image'])
                frame['sim_point'] = [frame['sim_point'][0]+dx,frame['sim_point'][1]+dy]
                if frame.get('reference_point_at_exposure'):
                    p=frame['reference_point_at_exposure']
                    frame['reference_point_at_exposure']=[p[0]+dx,p[1]+dy]
            out=pathlib.Path(tempfile.mkdtemp(prefix='preweld-offset-'))
            manifest=out/'input.jsonl'
            manifest.write_text(''.join(json.dumps(f)+'\n' for f in frames),encoding='utf-8')
            process=subprocess.run([str(binary),str(manifest)],capture_output=True,timeout=20)
            self.assertEqual(process.returncode,0)
            rows=[json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
            self.assertTrue(all(r['detector_valid'] and r['control_eligible'] for r in rows))
            points.append([r['final_pixel'] for r in rows])
        for shifted in points[1:]:
            self.assertTrue(all(math.dist(a,b)<0.1 for a,b in zip(points[0],shifted)),
                'Static virtual workpiece displacement must not drag real-image localization')


if __name__ == '__main__':
    unittest.main()
