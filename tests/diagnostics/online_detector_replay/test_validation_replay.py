"""定位校验公开入口：证据来源实际决定焊点是否被纳入。"""
import copy
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest

from validation_replay import ValidationReplaySession


def measured(frame=1):
    raw=dict(frame_id=frame,available=True, image_observation_accepted=True,
             image_source='geometry', source='geometry',trust='independent_geometry',
             accepted_image_pixel=[361,260],positioner_tcp_mm=[1,0,0])
    return dict(raw,raw_detector=raw,mode='measured',fresh_exposure=True,
                anchor_refreshed=True,anchor={'frame_id':frame},can_independent_validate=True)


def context():
    return {'coordinate_frame':'PositionerTcp','spatial_provenance':{
        'kind':'synthetic_fixture','frame_ids':[1]},'candidate_point_by_frame':{'1':[0,0,0]}}


class ValidationReplayTests(unittest.TestCase):
    def test_equal_geometry_different_source_changes_weld_verdict_but_not_surface_path(self):
        session=ValidationReplaySession({'max_weld_residual_mm':.2})
        accepted=measured()
        estimated=copy.deepcopy(accepted)
        estimated.update(source='prediction',mode='estimated',available=True,can_independent_validate=True)
        a=session.replay([accepted],context())
        b=session.replay([estimated],context())
        self.assertEqual(a['weld_validation']['decision'],'rejected')
        self.assertEqual(a['weld_validation']['included_frame_ids'],[1])
        self.assertEqual(b['weld_validation']['decision'],'unavailable')
        self.assertEqual(b['weld_validation']['included_frame_ids'],[])
        self.assertTrue(a['surface_validation']['executed'])
        self.assertTrue(b['surface_validation']['executed'])
        self.assertEqual(a['surface_validation']['decision'],b['surface_validation']['decision'])
        self.assertFalse(a['surface_validation']['verified'])

    def test_same_image_geometry_with_unknown_space_has_no_independent_spatial_qualification(self):
        ctx=context()
        ctx['spatial_provenance']={'kind':'nominal','frame_ids':[1]}
        result=ValidationReplaySession({'max_weld_residual_mm':.2}).replay([measured()],ctx)
        self.assertTrue(result['qualification'][0]['image_geometry_eligible'])
        self.assertFalse(result['qualification'][0]['independent_spatial_eligible'])
        self.assertEqual(result['weld_validation']['decision'],'unavailable')

    def test_valid_measured_geometry_can_accept_while_all_estimate_families_are_excluded(self):
        ctx=context();ctx['candidate_point_by_frame']['1']=[.9,0,0]
        passed=ValidationReplaySession({'max_weld_residual_mm':.2}).replay([measured()],ctx)
        self.assertEqual(passed['weld_validation']['decision'],'accepted')
        rows=[]
        for frame,source in enumerate(['kcf_identity','mother_assisted','prediction','transition'],1):
            row=measured(frame);row.update(source=source,mode='estimated',available=True,can_independent_validate=True)
            rows.append(row)
        excluded=ValidationReplaySession({'max_weld_residual_mm':.2}).replay(rows,ctx)
        self.assertEqual(excluded['weld_validation']['included_frame_ids'],[])
        self.assertTrue(excluded['surface_validation']['executed'])

    def test_surface_residual_improves_but_coverage_prevents_posterior_promotion(self):
        result=ValidationReplaySession().replay([],context())
        metrics=result['surface_validation']['frame_metrics']
        self.assertEqual(len(metrics),4)
        self.assertTrue(all(m['paired_after']<m['paired_before'] for m in metrics))
        self.assertLess(metrics[-1]['real_coverage'],.7)
        self.assertEqual(metrics[-1]['decision'],2)
        self.assertEqual(result['surface_validation']['decision'],'not_verified')
        self.assertEqual(result['weld_validation']['decision'],'unavailable')

    def test_duplicate_exposure_and_unaccepted_raw_geometry_do_not_prove_candidate(self):
        first=measured();second=copy.deepcopy(first)
        second['raw_detector']['image_observation_accepted']=False
        result=ValidationReplaySession({'max_weld_residual_mm':2}).replay([first,second],context())
        self.assertEqual(result['weld_validation']['included_frame_ids'],[1])
        self.assertFalse(result['qualification'][1]['included'])

    def test_real_detector_evidence_is_consumed_by_validation_entry(self):
        here=pathlib.Path(__file__).resolve().parent
        folder=pathlib.Path(tempfile.mkdtemp(prefix='kcf-validation-image-'))
        pixels=bytes(225 if abs(y-(260-.45*abs(x-360)))<1.5 else 14
                     for y in range(480) for x in range(640))
        (folder/'visible.pgm').write_bytes(b'P5\n640 480\n255\n'+pixels)
        frames=[dict(image='visible.pgm',frame_id=i,kind='synthetic',
                     projection={'fixture':'synthetic_identity_1mm_per_pixel'}) for i in [1,2]]
        manifest=folder/'manifest.jsonl'
        manifest.write_text(''.join(json.dumps(f)+'\n' for f in frames))
        command=[sys.executable,str(here/'replay.py'),str(manifest),'--evidence','--output',str(folder/'out')]
        if os.environ.get('SONTECH_DETECTOR_REPLAY_CACHE'):
            command+=['--cache',os.environ['SONTECH_DETECTOR_REPLAY_CACHE']]
        subprocess.run(command,check=True,capture_output=True,text=True)
        rows=[json.loads(line) for line in (folder/'out/results.jsonl').read_text().splitlines()]
        ctx=context();ctx['spatial_provenance']['frame_ids']=[1,2]
        ctx['candidate_point_by_frame']['2']=[40,20,1000]
        result=ValidationReplaySession().replay(rows,ctx)
        self.assertEqual(result['weld_validation']['included_frame_ids'],[2])
        self.assertEqual(result['weld_validation']['decision'],'threshold_pending')
        self.assertTrue(result['surface_validation']['executed'])
        self.assertTrue(result['qualification'][1]['image_geometry_eligible'])

if __name__=='__main__':
    unittest.main()
