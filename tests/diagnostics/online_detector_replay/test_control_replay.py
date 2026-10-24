"""控制回放公开边界：真实跟随器的参考和本地P，不连接运动设备。"""
import json
import os
import subprocess
import sys
import pathlib
import tempfile
import unittest
import copy

from control_replay import ControlReplaySession


def measured(frame, point):
    return dict(frame_id=frame, kind='synthetic', available=True, mode='measured', source='geometry',
                trust='independent_geometry', can_independent_validate=True,
                fresh_exposure=True, anchor_refreshed=True, anchor={'frame_id':frame}, positioner_tcp_mm=point,
                raw_detector={'positioner_tcp_mm':point,'frame_id':frame,'available':True,
                    'kind':'synthetic','projection_kind':'synthetic_identity_1mm_per_pixel',
                    'image_observation_accepted':True,'image_source':'geometry','source':'geometry',
                    'trust':'independent_geometry','accepted_image_pixel':[360,260]})


def context():
    return dict(coordinate_frame='PositionerTcp', calibration_valid=True,
                tool_tcp_in_last_link_mm=[0,0,231], station_base_translation_mm=[0,0,0],
                robot_pose_mm_deg=[0,0,231,0,0,0], axes_deg=[0,0],
                teach_last_link_mm=[[0,0,231],[20,0,231]],
                model_positioner_tcp_mm=[[2,-3,0],[12,-3,0]],
                spatial_provenance={'kind':'synthetic_fixture','frame_ids':[1,2,3,4]})


class ControlReplayTests(unittest.TestCase):
    def test_measured_line_controls_actual_follower_instead_of_model(self):
        session = ControlReplaySession()
        result = session.replay([measured(1,[2,2,0]),measured(2,[12,2,0])],context())
        self.assertTrue(result['follow']['valid'])
        self.assertEqual(result['released_positioner_tcp_mm'],[[2,2,0],[12,2,0]])
        self.assertEqual(result['follow']['foot_last_link_mm'],[0,2,231])
        self.assertEqual(result['follow']['delta_um'],[0,1,0])
        self.assertEqual(result['model_counterfactual']['foot_last_link_mm'],[0,-3,231])
        self.assertEqual(result['model_counterfactual']['delta_um'],[0,-1,0])
        self.assertFalse(result['motion_sent'])

    def test_unknown_projection_cannot_release_despite_measured_flags(self):
        ctx = context()
        ctx.pop('spatial_provenance')
        with self.assertRaises(ValueError):
            ControlReplaySession().replay([measured(1,[2,2,0]),measured(2,[12,2,0])],ctx)

    def test_estimates_and_duplicate_exposure_cannot_gain_measured_control_eligibility(self):
        estimate = measured(3,[20,8,0])
        estimate.update(source='prediction',mode='estimated')
        result = ControlReplaySession().replay([measured(1,[2,2,0]),
            measured(1,[2,50,0]),estimate,measured(4,[12,2,0])],context())
        self.assertEqual(result['released_positioner_tcp_mm'],[[2,2,0],[12,2,0]])
        self.assertEqual([r['accepted'] for r in result['release_diagnostics']],[True,False,False,True])

    def test_axis_rotation_and_tcp_offset_are_applied_once(self):
        ctx = context()
        ctx.update(axes_deg=[0,90],robot_pose_mm_deg=[-10,20,201,0,0,0],
                   station_base_translation_mm=[10,-20,30])
        result = ControlReplaySession().replay([measured(1,[2,2,0]),measured(2,[12,2,0])],ctx)
        # Rz(90) * [x,y,z+231] - [10,-20,30] maps current [0,0,231]
        # to robot [-10,20,201]. +workpiece Y therefore requires -robot X.
        self.assertEqual(result['follow']['first_last_link_mm'],[2,2,231])
        for got,want in zip(result['follow']['current_last_link_mm'],[0,0,231]):
            self.assertAlmostEqual(got,want,places=8)
        self.assertEqual(result['follow']['delta_um'],[-1,0,0])

    def test_model_proposal_over_budget_preserves_raw_measurement_and_evidence(self):
        result = ControlReplaySession().replay([measured(1,[2,2,0]),measured(2,[12,2,0])],context(),
                                               proposals=[[2,-3,0],[12,-3,0]])
        self.assertEqual(result['released_positioner_tcp_mm'],[[2,2,0],[12,2,0]])
        self.assertEqual(result['release_diagnostics'][0]['rejected_processing_displacement_mm'],5)
        self.assertEqual(result['model_distance_diagnostic']['max_nearest_polyline_distance_mm'],5)
        self.assertFalse(result['model_distance_diagnostic']['is_true_seam_error'])

    def test_curved_measured_path_is_not_silently_flattened_to_endpoints(self):
        rows=[measured(1,[2,2,0]),measured(2,[7,6,0]),measured(3,[12,2,0])]
        result=ControlReplaySession().replay(rows,context())
        self.assertFalse(result['follow']['valid'])
        self.assertEqual(result['released_positioner_tcp_mm'],[[2,2,0],[7,6,0],[12,2,0]])
        self.assertEqual(result['reference_chord_deviation_mm'],4)

    def test_real_image_replay_evidence_flows_to_control_reference(self):
        here=pathlib.Path(__file__).resolve().parent
        folder=pathlib.Path(tempfile.mkdtemp(prefix='kcf-control-image-'))
        frames=[]
        for identity,x in enumerate([360,362,364],1):
            image=bytes(225 if abs(y-(260-.45*abs(i-x)))<1.5 else 14
                        for y in range(480) for i in range(640))
            name=f'{identity}.pgm'
            (folder/name).write_bytes(b'P5\n640 480\n255\n'+image)
            frames.append(dict(image=name,frame_id=identity,kind='synthetic',
                projection={'fixture':'synthetic_identity_1mm_per_pixel'}))
        manifest=folder/'manifest.jsonl'
        manifest.write_text(''.join(json.dumps(f)+'\n' for f in frames))
        command=[sys.executable,str(here/'replay.py'),str(manifest),
                 '--evidence','--output',str(folder/'evidence')]
        if os.environ.get('SONTECH_DETECTOR_REPLAY_CACHE'):
            command+=['--cache',os.environ['SONTECH_DETECTOR_REPLAY_CACHE']]
        subprocess.run(command,check=True,capture_output=True,text=True)
        rows=[json.loads(line) for line in (folder/'evidence/results.jsonl').read_text().splitlines()]
        result=ControlReplaySession().replay(rows,context())
        self.assertTrue(result['follow']['valid'])
        self.assertEqual(len(result['released_positioner_tcp_mm']),2)
        self.assertEqual(result['released_positioner_tcp_mm'],[r['positioner_tcp_mm'] for r in rows[1:]])
        self.assertLess(abs(result['released_positioner_tcp_mm'][0][0]-42),2)
        self.assertEqual(result['raw_evidence'][1]['raw_detector']['final_pixel'],rows[1]['raw_detector']['final_pixel'])

    def test_old_anchor_identity_does_not_certify_new_frame(self):
        stale=measured(2,[12,2,0])
        stale['anchor']['frame_id']=1
        result=ControlReplaySession().replay([measured(1,[2,2,0]),stale],context())
        self.assertFalse(result['follow']['valid'])
        self.assertEqual(len(result['released_positioner_tcp_mm']),1)

    def test_contradictory_raw_evidence_cannot_be_relabelled_for_control(self):
        def complete(frame,point):
            row=measured(frame,point)
            row['raw_detector'].update(frame_id=frame,available=True,image_source='geometry',
                source='geometry',trust='independent_geometry',accepted_image_pixel=[360,260])
            return row
        changes=[{'frame_id':99},{'available':False},{'image_observation_accepted':False},
                 {'image_source':'kcf_identity'},{'trust':'tracked_not_independent'},
                 {'accepted_image_pixel':None},{'positioner_tcp_mm':[12,-3,0]}]
        for patch in changes:
            with self.subTest(patch=patch):
                rows=[complete(1,[2,2,0]),complete(2,[12,2,0])]
                rows[1]['raw_detector'].update(copy.deepcopy(patch))
                result=ControlReplaySession().replay(rows,context())
                self.assertEqual(len(result['released_positioner_tcp_mm']),1)
                self.assertFalse(result['follow']['valid'])
                self.assertFalse(result['release_diagnostics'][1]['accepted'])

    def test_actual_icp_model_replacement_is_compared_to_measured_release(self):
        rows=[measured(i+1,[float(i),2+.03*i*i+(.35 if i%2 else -.25),.02*i*i])
              for i in range(16)]
        ctx=context()
        ctx['spatial_provenance']['frame_ids']=list(range(1,17))
        ctx['model_positioner_tcp_mm']=[[float(i),2+.03*i*i,.02*i*i] for i in range(16)]
        result=ControlReplaySession({'max_reference_chord_deviation_mm':10}).replay(rows,ctx)
        baseline=result['actual_icp_baseline']
        self.assertTrue(baseline['icp_attempted'])
        self.assertTrue(baseline['icp_applied'])
        self.assertEqual(result['released_positioner_tcp_mm'],[row['positioner_tcp_mm'] for row in rows])
        self.assertNotEqual(baseline['released_positioner_tcp_mm'],result['released_positioner_tcp_mm'])
        self.assertTrue(baseline['follow']['valid'])
        self.assertTrue(result['follow']['valid'])
        self.assertGreater(baseline['max_raw_to_released_polyline_distance_mm'],.01)
        self.assertFalse(baseline['distance_is_true_seam_error'])

if __name__=='__main__':
    unittest.main()
