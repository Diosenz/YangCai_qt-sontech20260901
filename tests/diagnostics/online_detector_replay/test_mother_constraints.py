"""Pixel-only mother support via public replay CLI; truth stays in assertions."""
import json
import math
import pathlib
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent


def raster(left=True, right=True, cap=False, shift_y=0, slope=.45, cap_radius=32, cap_height=13):
    values = bytearray()
    for y in range(480):
        for x in range(640):
            dx = x-360
            mother = 260+shift_y-slope*abs(dx)
            stripe = ((left if dx<0 else right) and abs(y-mother)<1.5)
            if cap and abs(dx)<cap_radius:
                stripe = abs(y-(260+shift_y+cap_height*(1-(dx/cap_radius)**2)))<2
            values.append(225 if stripe else 14)
    return b'P5\n640 480\n255\n'+values


class MotherConstraintTests(unittest.TestCase):
    def run_case(self, scenes, extra=None, anchor_image=None, config_override=None):
        directory = pathlib.Path(tempfile.mkdtemp(prefix='kcf-mother-'))
        images = [anchor_image or raster(),anchor_image or raster()]+scenes
        frames = []
        for i,data in enumerate(images):
            name = f'frame{i}.pgm'
            (directory/name).write_bytes(data)
            frame = dict(image=name,kind='synthetic',frame_id=i+1,timestamp_ms=1000+i*100,
                         projection={'fixture':'synthetic_identity_1mm_per_pixel'},
                         weld_progress_mm=i,progress_reference='workpiece/path/v1',
                         progress_provenance='defined caller exposure-associated arc',progress_exposure_frame_id=i+1,
                         reference_revision=1,reference_point_at_exposure=[100,100],reference_exposure_frame_id=i+1)
            if extra:
                frame.update(extra)
            frames.append(frame)
        manifest = directory/'input.jsonl'
        manifest.write_text(''.join(json.dumps(row)+'\n' for row in frames),encoding='utf-8')
        config = directory/'config.json'
        candidates = dict(max_prediction_ms=1000,max_prediction_travel_mm=10,enable_mother_constraints=True)
        candidates.update(config_override or {})
        config.write_text(json.dumps(candidates),encoding='utf-8')
        result = subprocess.run([sys.executable,str(HERE/'replay.py'),str(manifest),'--evidence',
                                 '--policy-config',str(config),'--output',str(directory/'output')],
                                text=True,capture_output=True)
        self.assertEqual(result.returncode,0,result.stderr)
        return [json.loads(line) for line in (directory/'output/results.jsonl').read_text().splitlines()]

    def test_cap_with_outer_mother_support_is_auxiliary_and_does_not_renew_anchor(self):
        rows = self.run_case([raster(cap=True,shift_y=3)])
        self.assertTrue(rows[1]['anchor_refreshed'])
        self.assertTrue(rows[2]['available'])
        self.assertEqual(rows[2]['source'],'mother_material_estimate')
        self.assertEqual(rows[2]['mother_observation']['support_rank'],2)
        self.assertFalse(rows[2]['mother_observation']['center_supported'])
        self.assertEqual(rows[2]['anchor'],rows[1]['anchor'])
        self.assertFalse(rows[2]['can_independent_validate'])
        self.assertIsNone(rows[2]['positioner_tcp_mm'])
        self.assertLess(math.dist(rows[2]['final_pixel'],[360,263]),2)
        self.assertGreater(abs(rows[2]['final_pixel'][1]-276),10)

    def test_single_side_changes_normal_only_and_cannot_refresh_anchor(self):
        rows = self.run_case([raster(right=False,shift_y=3)])
        row = rows[-1]
        self.assertEqual(row['source'],'mother_material_estimate')
        observed = row['mother_observation']
        self.assertEqual(observed['support_rank'],1)
        self.assertTrue(observed['tangential_component_predicted'])
        self.assertTrue(observed['left']['valid'])
        self.assertFalse(observed['right']['valid'])
        n = observed['left']['normal']
        correction = [a-b for a,b in zip(row['final_pixel'],observed['proposal'])]
        self.assertAlmostEqual(correction[0]*n[1]-correction[1]*n[0],0,places=6)
        self.assertGreater(math.dist(row['final_pixel'],observed['proposal']),1)
        self.assertEqual(row['anchor'],rows[1]['anchor'])
        self.assertFalse(row['can_independent_validate'])

    def test_lost_support_remains_prediction_and_truth_metadata_cannot_supply_pixels(self):
        blank = b'P5\n640 480\n255\n'+bytes([14])*640*480
        first = self.run_case([blank],extra={'scene':'bilateral', 'truth_pixel':[360,260], 'mother_lines':[[1,2],[3,4]]})
        second = self.run_case([blank],extra={'scene':'tack', 'truth_pixel':[-999,-999]})
        self.assertEqual(first[-1]['source'],'bounded_prediction')
        self.assertEqual(first[-1]['mother_observation']['support_rank'],0)
        self.assertEqual(first[-1]['final_pixel'],second[-1]['final_pixel'])
        self.assertEqual(first[-1]['mother_observation']['left']['reason'],'insufficient_narrow_pixel_support')

    def test_nearly_parallel_side_normals_do_not_claim_two_dimensions(self):
        rows = self.run_case([raster(cap=True,slope=.18,shift_y=3)],anchor_image=raster(slope=.18),
                             config_override={'mother_options':{'min_normal_determinant':.4}})
        observation = rows[-1]['mother_observation']
        self.assertTrue(rows[1]['anchor_refreshed'])
        self.assertTrue(observation['left']['valid'])
        self.assertTrue(observation['right']['valid'])
        self.assertEqual(observation['support_rank'],1)
        self.assertIn('near_parallel_rank_one_only',observation['reasons'])

    def test_detector_accepted_small_extra_corner_cannot_refresh_mother_anchor(self):
        cap = raster(cap=True,cap_radius=12,cap_height=1)
        guarded = self.run_case([cap,cap],config_override={'max_prediction_ms':200})
        baseline = self.run_case([cap],config_override={'enable_mother_constraints':False})
        self.assertEqual(baseline[-1]['source'],'geometry')
        self.assertEqual(guarded[2]['raw_detector']['source'],'geometry')
        self.assertTrue(guarded[2]['mother_geometry_rejected'])
        self.assertEqual(guarded[2]['source'],'mother_material_estimate')
        self.assertEqual(guarded[2]['anchor']['frame_id'],2)
        self.assertFalse(guarded[3]['available'])
        self.assertIn('time_budget_exhausted',guarded[3]['prediction_failures'])
        self.assertEqual(guarded[3]['anchor']['frame_id'],2)

if __name__=='__main__':
    unittest.main()
