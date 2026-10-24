"""Public replay acceptance for new-exposure recovery and bounded transitions."""
import math
import unittest
import test_evidence_session as replay_tools


class RecoveryTests(unittest.TestCase):
    run_sequence = replay_tools.EvidenceReplayTests.run_sequence
    continuation_frame = replay_tools.EvidenceReplayTests.continuation_frame
    def test_recovered_measurement_is_preserved_while_transition_is_not_an_anchor(self):
        frames=[self.continuation_frame(i,1000+i*100,i*.1,[100,100],
                    image='blank.pgm' if i==3 else ('shifted.pgm' if i>=4 else 'visible.pgm'))
                for i in range(1,10)]
        rows=self.run_sequence(frames,dict(max_prediction_ms=1000,max_prediction_travel_mm=10,
                  recovery_confirmation_frames=2,max_recovery_step_pixel=1,
                  max_reacquisition_distance_pixel=12))
        self.assertEqual(rows[3]['anchor']['frame_id'],2)
        self.assertEqual(rows[3]['source'],'bounded_prediction')
        self.assertEqual(rows[4]['source'],'recovery_transition')
        self.assertFalse(rows[4]['can_independent_validate'])
        self.assertEqual(rows[4]['anchor']['frame_id'],2)
        self.assertLessEqual(math.dist(rows[4]['final_pixel'],rows[3]['final_pixel']),1.000001)
        self.assertLess(abs(rows[4]['recovery_measurement_pixel'][0]-365),2)
        self.assertGreater(math.dist(rows[4]['final_pixel'],rows[4]['recovery_measurement_pixel']),2)
        self.assertTrue(any(r['source']=='geometry' and r['anchor']['frame_id']>2 for r in rows[5:]))

    def test_duplicate_exposure_breaks_recovery_confirmation(self):
        ids=[1,2,3,4,4,5,6]
        frames=[self.continuation_frame(i,1000+j*100,j*.1,[100,100],
                    image='blank.pgm' if i==3 else ('shifted.pgm' if i>=4 else 'visible.pgm'))
                for j,i in enumerate(ids)]
        rows=self.run_sequence(frames,dict(max_prediction_ms=1000,max_prediction_travel_mm=10,
                  recovery_confirmation_frames=2,max_recovery_step_pixel=1,
                  max_reacquisition_distance_pixel=12))
        self.assertEqual(rows[4]['recovery_confirmation_count'],0)
        self.assertFalse(rows[4]['available'])
        self.assertEqual(rows[5]['recovery_confirmation_count'],1)
        self.assertEqual(rows[5]['anchor']['frame_id'],2)
        self.assertEqual(rows[6]['source'],'recovery_transition')

    def test_transition_cannot_renew_expired_prediction_budget(self):
        frames=[self.continuation_frame(i,1000+i*100,i*.1,[100,100],
                    image='blank.pgm' if i==3 else ('shifted.pgm' if i>=4 else 'visible.pgm'))
                for i in range(1,8)]
        rows=self.run_sequence(frames,dict(max_prediction_ms=400,max_prediction_travel_mm=10,
                  recovery_confirmation_frames=2,max_recovery_step_pixel=1,
                  max_reacquisition_distance_pixel=12))
        self.assertEqual(rows[4]['source'],'recovery_transition')
        self.assertFalse(rows[5]['available'])
        self.assertEqual(rows[5]['anchor']['frame_id'],2)
        self.assertIn('time_budget_exhausted',rows[5]['prediction_failures'])
        self.assertEqual(rows[6]['source'],'geometry')
        self.assertEqual(rows[6]['anchor']['frame_id'],7)

    def test_distant_geometry_is_not_silently_adopted(self):
        frames=[self.continuation_frame(i,1000+i*100,i*.1,[100,100],
                    image='blank.pgm' if i==3 else ('far.pgm' if i>=4 else 'visible.pgm'))
                for i in range(1,8)]
        rows=self.run_sequence(frames,dict(max_prediction_ms=1000,max_prediction_travel_mm=10,
                  recovery_confirmation_frames=2,max_recovery_step_pixel=1,
                  max_reacquisition_distance_pixel=12))
        self.assertTrue(any(r['raw_detector']['available'] for r in rows[3:]))
        self.assertTrue(all(r['anchor']['frame_id']==2 for r in rows[2:]))
        self.assertTrue(all(not r['can_independent_validate'] for r in rows[2:]))
        self.assertTrue(any('distance' in r['message'] for r in rows[3:]))

if __name__=='__main__':
    unittest.main()
