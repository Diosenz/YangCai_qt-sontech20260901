"""已确认完整回放入口的证据行为验收；真值只用于断言。"""
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent


class EvidenceReplayTests(unittest.TestCase):
    def run_sequence(self, frames, config=None):
        folder = pathlib.Path(tempfile.mkdtemp(prefix='kcf-evidence-'))
        image = bytearray(225 if abs(y - (260 - .45 * abs(x - 360))) < 1.5 else 14
                          for y in range(480) for x in range(640))
        (folder/'visible.pgm').write_bytes(b'P5\n640 480\n255\n'+image)
        (folder/'blank.pgm').write_bytes(b'P5\n640 480\n255\n'+bytes([14])*640*480)
        manifest = folder/'input.jsonl'
        values = []
        for frame in frames:
            value = dict(image='visible.pgm', kind='synthetic',
                         projection={'fixture':'synthetic_identity_1mm_per_pixel'})
            value.update(frame)
            values.append(value)
        manifest.write_text(''.join(json.dumps(f)+'\n' for f in values), encoding='utf-8')
        command = [sys.executable,str(HERE/'replay.py'),str(manifest),'--output',str(folder/'out'),'--evidence']
        if config:
            path = folder/'config.json'
            path.write_text(json.dumps(config),encoding='utf-8')
            command += ['--policy-config',str(path)]
        result = subprocess.run(command,capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        return [json.loads(line) for line in (folder/'out/results.jsonl').read_text().splitlines()]

    def test_geometry_without_cad_establishes_anchor_only_on_new_exposures(self):
        rows = self.run_sequence([
            {'frame_id':1,'sim_point':[80,80]},
            {'frame_id':2},
            {'frame_id':2},
            {'frame_id':3,'projection':{}}])
        self.assertFalse(rows[0]['available'])
        self.assertTrue(rows[1]['available'])
        self.assertEqual(rows[1]['anchor']['frame_id'],2)
        self.assertTrue(rows[1]['can_independent_validate'])
        self.assertLess(abs(rows[1]['final_pixel'][0]-360),2)
        self.assertFalse(rows[2]['can_independent_validate'])
        self.assertEqual(rows[2]['anchor']['frame_id'],2)
        self.assertFalse(rows[3]['available'])
        self.assertTrue(rows[1]['raw_detector']['available'])

    def continuation_frame(self, identity, time, progress, reference, **updates):
        value = dict(frame_id=identity, timestamp_ms=time, weld_progress_mm=progress,
                     progress_reference='workpiece-A/frozen-seam-path-v1',
                     progress_provenance='caller-supplied exposure-associated workpiece arc; includes external axes',
                     progress_exposure_frame_id=identity,
                     reference_revision=1, reference_point_at_exposure=reference,
                     reference_exposure_frame_id=identity)
        value.update(updates)
        return value

    def test_anchored_prediction_expires_at_time_boundary_without_self_renewal(self):
        rows = self.run_sequence([
            self.continuation_frame(1, 1000, 0, [100,100]),
            self.continuation_frame(2, 1100, 1, [101,100]),
            self.continuation_frame(3, 1200, 2, [103,101], image='blank.pgm'),
            self.continuation_frame(4, 1300, 3, [104,102], image='blank.pgm')],
            {'max_prediction_ms':200, 'max_prediction_travel_mm':10})
        self.assertTrue(rows[2]['available'])
        self.assertEqual(rows[2]['source'],'bounded_prediction')
        self.assertEqual(rows[2]['anchor']['frame_id'],2)
        self.assertAlmostEqual(rows[2]['final_pixel'][0]-rows[1]['final_pixel'][0],2)
        self.assertAlmostEqual(rows[2]['final_pixel'][1]-rows[1]['final_pixel'][1],1)
        self.assertFalse(rows[2]['can_independent_validate'])
        self.assertIsNone(rows[2]['positioner_tcp_mm'])
        self.assertFalse(rows[2]['control_eligible'])
        self.assertEqual(rows[2]['prediction_elapsed_ms'],100)
        self.assertTrue(rows[2]['prediction_travel_known'])
        self.assertFalse(rows[3]['available'])
        self.assertIn('time_budget_exhausted',rows[3]['prediction_failures'])
        self.assertEqual(rows[3]['anchor']['frame_id'],2)

    def test_reference_jump_requires_same_exposure_bridge_without_budget_reset(self):
        rows = self.run_sequence([
            self.continuation_frame(1, 1000, 0, [100,100]),
            self.continuation_frame(2, 1100, 1, [101,100]),
            self.continuation_frame(3, 1200, 2, [501,500], image='blank.pgm',
                reference_revision=2, previous_reference_revision=1,
                previous_reference_point_at_exposure=[102,100],
                previous_reference_exposure_frame_id=3),
            self.continuation_frame(4, 1300, 3, [502,501], image='blank.pgm', reference_revision=2),
            self.continuation_frame(5, 1400, 4, [900,900], image='blank.pgm', reference_revision=3)],
            {'max_prediction_ms':1000, 'max_prediction_travel_mm':10})
        self.assertTrue(rows[2]['available'])
        self.assertEqual(rows[2]['reference_alignment'],'same_exposure_bridge')
        self.assertAlmostEqual(rows[2]['final_pixel'][0]-rows[1]['final_pixel'][0],1)
        self.assertAlmostEqual(rows[2]['final_pixel'][1]-rows[1]['final_pixel'][1],0)
        self.assertEqual(rows[2]['prediction_elapsed_ms'],100)
        self.assertEqual(rows[3]['prediction_travel_mm'],2)
        self.assertEqual(rows[3]['anchor'],rows[1]['anchor'])
        self.assertAlmostEqual(rows[3]['final_pixel'][0]-rows[1]['final_pixel'][0],2)
        self.assertFalse(rows[4]['available'])
        self.assertIn('reference_revision_unaligned',rows[4]['prediction_failures'])

    def test_boolean_progress_exposure_cannot_establish_budget_provenance(self):
        rows = self.run_sequence([
            self.continuation_frame(1,1000,0,[100,100], progress_exposure_frame_id=True),
            self.continuation_frame(2,1100,1,[101,100], image='blank.pgm')],
            {'anchor_confirmation_frames':1, 'max_prediction_ms':1000,
             'max_prediction_travel_mm':10})
        self.assertTrue(rows[0]['can_independent_validate'])
        self.assertFalse(rows[1]['available'])
        self.assertIn('progress_provenance_unknown',rows[1]['prediction_failures'])

    def test_reversal_and_external_axis_workpiece_progress_consume_distance_budget(self):
        rows = self.run_sequence([
            self.continuation_frame(1,1000,0,[100,100], robot_base_tcp_mm=[0,0,0], external_s2_deg=0),
            self.continuation_frame(2,1100,1,[101,100], robot_base_tcp_mm=[0,0,0], external_s2_deg=1),
            self.continuation_frame(3,1110,3,[102,100], image='blank.pgm', robot_base_tcp_mm=[0,0,0], external_s2_deg=2),
            self.continuation_frame(4,1120,1,[103,100], image='blank.pgm', robot_base_tcp_mm=[0,0,0], external_s2_deg=0)],
            {'max_prediction_ms':1000, 'max_prediction_travel_mm':4})
        self.assertTrue(rows[2]['available'])
        self.assertEqual(rows[2]['prediction_travel_mm'],2)
        self.assertFalse(rows[3]['available'])
        self.assertEqual(rows[3]['prediction_travel_mm'],4)
        self.assertIn('travel_budget_exhausted',rows[3]['prediction_failures'])
        self.assertEqual(rows[3]['prediction_elapsed_ms'],20)

    def test_missing_progress_latches_unavailable_and_duplicates_do_not_refresh(self):
        rows = self.run_sequence([
            self.continuation_frame(1,1000,0,[100,100]),
            self.continuation_frame(2,1100,1,[101,100]),
            self.continuation_frame(2,1100,1,[101,100]),
            self.continuation_frame(3,1200,None,[102,100],image='blank.pgm'),
            self.continuation_frame(4,1300,3,[103,100],image='blank.pgm')],
            {'max_prediction_ms':1000, 'max_prediction_travel_mm':10})
        self.assertFalse(rows[2]['fresh_exposure'])
        self.assertFalse(rows[2]['available'])
        self.assertEqual(rows[2]['anchor'],rows[1]['anchor'])
        for row in rows[3:]:
            self.assertFalse(row['available'])
            self.assertIn('progress_provenance_unknown',row['prediction_failures'])
            self.assertFalse(row['prediction_travel_known'])
            self.assertEqual(row['anchor'],rows[1]['anchor'])

    def test_no_anchor_and_unknown_time_or_reference_never_emit_prediction(self):
        config = {'max_prediction_ms':1000,'max_prediction_travel_mm':10}
        absent = self.run_sequence([
            self.continuation_frame(1,1000,0,[100,100],image='blank.pgm'),
            self.continuation_frame(2,1100,1,[101,100],image='blank.pgm')],config)
        self.assertFalse(any(row['available'] for row in absent))
        unknown = self.run_sequence([
            self.continuation_frame(1,1000,0,[100,100]),
            self.continuation_frame(2,1100,1,[101,100]),
            self.continuation_frame(3,None,2,None,image='blank.pgm'),
            self.continuation_frame(4,1300,3,[103,100],image='blank.pgm')],config)
        self.assertFalse(unknown[2]['available'])
        self.assertIn('reference_geometry_unavailable',unknown[2]['prediction_failures'])
        self.assertFalse(unknown[3]['available'])
        self.assertEqual(unknown[3]['anchor']['frame_id'],2)

    def test_within_budget_is_not_accuracy_or_truth_qualification(self):
        frames = [self.continuation_frame(1,1000,0,[100,100]),
                  self.continuation_frame(2,1100,1,[101,100]),
                  self.continuation_frame(3,1200,2,[103,101],image='blank.pgm',
                                          truth_pixel=[380,280], scene='occluded')]
        config = {'max_prediction_ms':1000,'max_prediction_travel_mm':10}
        first = self.run_sequence(frames,config)
        frames[-1].update(truth_pixel=[-999,-999],scene='normal')
        second = self.run_sequence(frames,config)
        self.assertTrue(first[-1]['available'])
        self.assertGreater(abs(first[-1]['final_pixel'][0]-380),10)
        self.assertFalse(first[-1]['control_eligible'])
        for key in ('final_pixel','source','anchor','prediction_travel_mm'):
            self.assertEqual(first[-1][key],second[-1][key])

    def test_out_of_image_relative_prediction_is_unavailable_even_with_budget(self):
        rows = self.run_sequence([
            self.continuation_frame(1,1000,0,[100,100]),
            self.continuation_frame(2,1100,1,[101,100]),
            self.continuation_frame(3,1200,2,[900,900],image='blank.pgm')],
            {'max_prediction_ms':1000,'max_prediction_travel_mm':10})
        self.assertFalse(rows[-1]['available'])
        self.assertIn('prediction_outside_image',rows[-1]['prediction_failures'])

if __name__=='__main__':
    unittest.main()
