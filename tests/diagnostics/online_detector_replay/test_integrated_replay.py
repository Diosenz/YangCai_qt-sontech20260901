"""用户确认的公开 CLI：图像 JSONL 到分层报告；不测试内部函数。"""
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent


class IntegratedReplayTests(unittest.TestCase):
    def run_cli(self, change=None):
        folder = pathlib.Path(tempfile.mkdtemp(prefix='kcf-integrated-'))
        frames = []
        for index in range(3):
            x = 360 + index * 2
            pixels = bytes(225 if abs(y-(260-.45*abs(i-x))) < 1.5 else 14
                           for y in range(480) for i in range(640))
            name = str(index) + '.pgm'
            (folder/name).write_bytes(b'P5\n640 480\n255\n'+pixels)
            frames.append(dict(image=name, kind='synthetic', observation_id='o'+str(index),
                frame_id=index+1, timestamp_ms=1000+index*100, reference_revision=1,
                projection={'fixture':'synthetic_identity_1mm_per_pixel'},
                material_position=dict(coordinate_frame='PositionerTcp', workpiece_frame='fixture-A',
                                       path_id='seam-A', path_revision=1, s_mm=index*2)))
        context = dict(coordinate_frame='PositionerTcp', calibration_valid=True,
            tool_tcp_in_last_link_mm=[0,0,231], station_base_translation_mm=[0,0,0],
            robot_pose_mm_deg=[0,0,1231,0,0,0], axes_deg=[0,0],
            teach_last_link_mm=[[40,20,1231],[60,20,1231]],
            model_positioner_tcp_mm=[[40,19,1000],[60,19,1000]],
            spatial_provenance={'kind':'synthetic_fixture','frame_ids':[1,2,3]})
        tcp = [dict(record_id='later-same-material', kind='synthetic', timestamp_ms=2000,
            material_position=frames[1]['material_position'], positioner_tcp_mm=[42,20.5,1000]),
            dict(record_id='simultaneous-wrong-material', kind='synthetic', timestamp_ms=1100,
                 material_position=frames[2]['material_position'], positioner_tcp_mm=[42,20,1000])]
        evaluation = {'kind':'synthetic_defined_truth', 'observations':{
            'o1':{'scenario':'normal','pixel':[362,260], 'positioner_tcp_mm':[42,20,1000]}}}
        if change:
            change(frames,tcp,evaluation)
        for name,value in [('context.json',context),('evaluation.json',evaluation),('policy.json',{})]:
            (folder/name).write_text(json.dumps(value),encoding='utf-8')
        for name,values in [('manifest.jsonl',frames),('tcp.jsonl',tcp)]:
            (folder/name).write_text(''.join(json.dumps(v)+'\n' for v in values),encoding='utf-8')
        command=[sys.executable,str(HERE/'integrated_replay.py'),str(folder/'manifest.jsonl'),
                 '--context',str(folder/'context.json'),'--tcp',str(folder/'tcp.jsonl'),
                 '--evaluation',str(folder/'evaluation.json'),'--policy-config',str(folder/'policy.json'),
                 '--output',str(folder/'out')]
        cache=os.environ.get('SONTECH_DETECTOR_REPLAY_CACHE')
        if cache:
            command+=['--cache',cache]
        run=subprocess.run(command,text=True,capture_output=True)
        self.assertEqual(run.returncode,0,run.stderr)
        return json.loads((folder/'out/report.json').read_text()),folder

    def test_same_material_later_tcp_is_compared_without_nearest_or_same_time_matching(self):
        result,folder=self.run_cli()
        rows=result['trace']
        pair=rows[1]['tcp_comparison']
        self.assertEqual(pair['tcp_record_id'],'later-same-material')
        self.assertEqual(pair['association'],'exact_explicit_material_position_and_later_time')
        self.assertAlmostEqual(pair['reference_to_tcp_distance_mm'],.5,delta=.2)
        self.assertFalse(pair['is_seam_accuracy'])
        self.assertIsNone(rows[2]['tcp_comparison']['tcp_record_id'])
        self.assertTrue(result['control']['follow']['valid'])
        self.assertTrue(result['validation']['surface_validation']['executed'])
        self.assertEqual(rows[1]['identity']['observation_id'],'o1')
        self.assertEqual(rows[1]['identity']['reference_revision'],1)
        self.assertEqual(rows[1]['released_positioner_tcp_mm'],rows[1]['final_positioner_tcp_mm'])
        self.assertEqual(result['summary']['total_records'],3)
        self.assertEqual(result['summary']['unavailable_records'],1)
        self.assertTrue((folder/'out/index.html').is_file())
        self.assertFalse(result['field_accuracy_verified'])
        archive=result['real_archived_execution']
        self.assertTrue(archive['actual_tcp_archive_available'])
        self.assertGreater(archive['files'][0]['records'],6000)
        self.assertFalse(archive['same_material_association_available'])

    def test_missing_material_position_stays_unknown_and_truth_cannot_change_decisions(self):
        first,_=self.run_cli()
        def change(frames,tcp,evaluation):
            frames[1]['material_position']=None
            evaluation['observations']['o1'].update(pixel=[-900,900],scenario='arbitrary_label')
        second,_=self.run_cli(change)
        self.assertIsNone(second['trace'][1]['material_position'])
        self.assertIsNone(second['trace'][1]['tcp_comparison']['tcp_record_id'])
        for a,b in zip(first['trace'],second['trace']):
            for key in ('final_pixel','source','final_positioner_tcp_mm','released_positioner_tcp_mm'):
                self.assertEqual(a[key],b[key])
        self.assertGreater(second['trace'][1]['evaluation']['final_pixel_error'],100)

    def test_unknown_real_space_is_reported_without_issuing_control_reference(self):
        def change(frames,tcp,evaluation):
            # Real images may be replayed without pretending that fixture poses are actual exposure poses.
            for frame in frames:
                frame.update(kind='real',projection=None,observation_id=None,material_position=None,
                             timestamp_ms=None,reference_revision=None)
        result,_=self.run_cli(change)
        self.assertFalse(result['control']['follow']['valid'])
        self.assertTrue(all(r['identity']['timestamp_ms'] is None for r in result['trace']))
        self.assertTrue(all(r['tcp_comparison']['tcp_record_id'] is None for r in result['trace']))
        self.assertTrue(result['validation']['surface_validation']['executed'])
        self.assertEqual(result['summary']['measured_space_error_mm']['count'],0)

    def test_different_path_revision_or_tcp_kind_cannot_be_paired(self):
        def change(frames,tcp,evaluation):
            tcp[0]['material_position']=dict(tcp[0]['material_position'],path_revision=2)
        result,_=self.run_cli(change)
        self.assertIsNone(result['trace'][1]['tcp_comparison']['tcp_record_id'])
        def mixed(frames,tcp,evaluation):
            tcp[0]['kind']='real'
        result,_=self.run_cli(mixed)
        self.assertIsNone(result['trace'][1]['tcp_comparison']['tcp_record_id'])

    def test_unidentified_truth_does_not_produce_accuracy_score(self):
        def change(frames,tcp,evaluation):
            evaluation.pop('kind')
        result,_=self.run_cli(change)
        self.assertEqual(result['summary']['final_pixel_error']['count'],0)

    def test_real_nominal_projection_cannot_gain_synthetic_control_or_validation(self):
        def change(frames,tcp,evaluation):
            identity=[1,0,0,0,1,0,0,0,1,0,0,0]
            projection=dict(calibration={'focal_length_m':.0122258,'sx_m':3.75202e-6,'sy_m':3.75e-6,
                'cx_pixel':672.386,'cy_pixel':513.657,'kappa':-2503.95,'laser_plane_a':.334870,
                'laser_plane_b':10.819546,'laser_plane_c':-4.521574},camera_in_tool=identity,
                tool_in_robot_base=identity,robot_base_to_positioner_tcp=identity,
                provenance='nominal diagnostic transforms; NOT archived exposure poses')
            for frame in frames:
                frame.update(kind='real',projection=projection)
        result,_=self.run_cli(change)
        self.assertTrue(any(r['raw_projected_positioner_tcp_mm'] for r in result['trace']))
        self.assertFalse(result['control']['follow']['valid'])
        self.assertEqual(result['control']['released_positioner_tcp_mm'],[])
        self.assertEqual(result['validation']['weld_validation']['included_frame_ids'],[])

    def test_single_combined_sequence_preserves_hard_frames_and_provenance(self):
        folder=pathlib.Path(tempfile.mkdtemp(prefix='kcf-combined-demo-'))/'demo'
        command=[sys.executable,str(HERE/'make_integrated_demo.py'),'--output',str(folder)]
        cache=os.environ.get('SONTECH_DETECTOR_REPLAY_CACHE')
        if cache:
            command+=['--cache',cache]
        run=subprocess.run(command,text=True,capture_output=True)
        self.assertEqual(run.returncode,0,run.stderr)
        result=json.loads((folder/'combined-input/result/report.json').read_text())
        scenes={r['evaluation']['scenario'] for r in result['trace']}
        self.assertTrue({'normal','tack_occlusion','single_side','reflection_competition',
                         'reference_update','reference_missing','recovery'}<=scenes)
        sources={r['source'] for r in result['trace']}
        self.assertIn('mother_material_estimate',sources)
        self.assertIn('bounded_prediction',sources)
        self.assertIn('recovery_transition',sources)
        self.assertGreater(result['summary']['unavailable_records'],0)
        self.assertGreater(result['summary']['estimated_records'],0)
        self.assertGreater(result['summary']['measured_records'],10)
        self.assertTrue(result['control']['actual_icp_baseline']['icp_attempted'])
        self.assertTrue(result['control']['follow']['valid'])
        self.assertEqual(result['validation']['weld_validation']['decision'],'threshold_pending')
        self.assertGreater(len(result['validation']['weld_validation']['included_frame_ids']),10)
        self.assertTrue(all(not r['validation_qualification']['included'] for r in result['trace']
                            if r['source']!='geometry'))
        self.assertTrue(all(r['processing_displacement_mm']==0 for r in result['trace']
                            if r['released_positioner_tcp_mm'] is not None))
        self.assertGreater(result['summary']['recovery_transition_pixel_error']['count'],0)
        self.assertGreater(result['summary']['tcp_execution_discrepancy_mm']['count'],0)
        self.assertFalse(result['field_accuracy_verified'])
        self.assertTrue((folder/'real-14591/result/report.json').is_file())


if __name__=='__main__':
    unittest.main()
