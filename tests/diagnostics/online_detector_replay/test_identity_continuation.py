"""公开 replay --evidence 的目标身份与真实模板事务验收；不替换图像算法。"""
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
EVIDENCE = pathlib.Path(os.environ.get('KCF_IDENTITY_EVIDENCE',tempfile.gettempdir()))


class IdentityContinuationTests(unittest.TestCase):
    def run_sequence(self, images, config=None, extra=None):
        folder = pathlib.Path(tempfile.mkdtemp(prefix='kcf-identity-', dir=EVIDENCE))
        for name, corners in images.items():
            pixels = bytes(max([14]+[brightness for x0,y0,brightness in corners
                if abs(y-(y0-.45*abs(x-x0))) < 1.5]) for y in range(480) for x in range(640))
            (folder/(name+'.pgm')).write_bytes(b'P5\n640 480\n255\n'+pixels)
        names = ['anchor','anchor'] + [name for name in images if name != 'anchor']
        frames=[]
        for index,name in enumerate(names):
            frame=dict(image=name+'.pgm', kind='synthetic',frame_id=index+1,timestamp_ms=1000+100*index,
                projection={'fixture':'synthetic_identity_1mm_per_pixel'},
                weld_progress_mm=index*.1,progress_reference='workpiece/frozen-path-v1',
                progress_provenance='explicit synthetic exposure-associated arc input',
                progress_exposure_frame_id=index+1,reference_revision=1,
                reference_point_at_exposure=[360,260],reference_exposure_frame_id=index+1)
            if extra: frame.update(extra(index,name))
            frames.append(frame)
        manifest=folder/'input.jsonl';manifest.write_text(''.join(json.dumps(f)+'\n' for f in frames))
        policy={'max_prediction_ms':500,'max_prediction_travel_mm':10}
        policy.update(config or {})
        (folder/'config.json').write_text(json.dumps(policy))
        result=subprocess.run([sys.executable,str(HERE/'replay.py'),str(manifest),'--evidence',
            '--policy-config',str(folder/'config.json'),'--output',str(folder/'out')],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        return [json.loads(line) for line in (folder/'out/results.jsonl').read_text().splitlines()]

    def test_brighter_competitor_cannot_take_identity_or_retained_template(self):
        images={'anchor':[(360,260,150)]}
        images.update({f'competition-{i}':[(360,260,150),(210,330,255)] for i in range(5)})
        images['returned-1']=[(360,260,150)]
        images['returned-2']=[(360,260,150)]
        rows=self.run_sequence(images)
        self.assertFalse(any(row['can_independent_validate'] for row in rows[2:-2]),
                         'a brighter far corner replaced the historical target')
        for row in rows[2:-2]:
            self.assertFalse(row['can_independent_validate'])
            self.assertEqual(row['anchor']['frame_id'],2)
            self.assertFalse(row['template_retained_for_next_exposure'])
            self.assertEqual(row['checkpoint_source_frame'],2)
            self.assertIn('outside_historical_target_corridor',row['identity_evidence']['reasons'])
            if row['available']: self.assertLess(abs(row['final_pixel'][0]-360),3)
        self.assertIn('time_budget_exhausted',rows[-3]['prediction_failures'])
        self.assertTrue(rows[-2]['raw_detector']['training_committed'])
        self.assertTrue(rows[-2]['temporary_training_discarded'])
        self.assertFalse(rows[-2]['template_retained_for_next_exposure'])
        self.assertTrue(rows[-1]['can_independent_validate'])
        self.assertLess(abs(rows[-1]['final_pixel'][0]-360),2)


    def test_similar_multiple_competitors_allow_only_budgeted_historical_appearance(self):
        images={'anchor':[(360,260,150)]}
        images.update({f'similar-{i}':[(360,260,150),(210,330,255),(500,450,220)] for i in range(5)})
        rows=self.run_sequence(images, {'identity_min_patch_score':.60})
        supported=[row for row in rows[2:-1] if row['source']=='appearance_tracking']
        self.assertTrue(supported, 'real KCF plus visible historical appearance must be demonstrated')
        for row in supported:
            self.assertGreater(row['kcf_response_peak'],.20)
            self.assertGreater(row['identity_evidence']['historical_corridor_score'],.60)
            self.assertFalse(row['can_independent_validate'])
            self.assertFalse(row['anchor_refreshed'])
            self.assertEqual(row['anchor']['frame_id'],2)
            self.assertLess(abs(row['final_pixel'][0]-360),4)
            self.assertFalse(row['space_available'])
            self.assertIsNone(row['positioner_tcp_mm'])
        self.assertFalse(rows[-1]['available'])
        self.assertEqual(rows[-1]['anchor']['frame_id'],2)
        self.assertIn('time_budget_exhausted', rows[-1]['prediction_failures'])


    def test_relative_reference_cannot_authorize_far_similar_appearance_as_same_target(self):
        rows=self.run_sequence({'anchor':[(360,260,150)],
            'lure':[(360,260,150),(210,330,255)], 'blank':[],
            'returned-1':[(360,260,150)], 'returned-2':[(360,260,150)]},
            extra=lambda index,name: {'reference_point_at_exposure':[210,330]} if name=='lure' else {})
        row=rows[2]
        self.assertFalse(row['available'],
                         'a moved reference cannot drag the final KCF point to a far lookalike')
        self.assertEqual(row['anchor']['frame_id'],2)
        self.assertIn('reference_identity_conflict',row['prediction_failures'])
        self.assertFalse(rows[3]['available'])
        self.assertIn('reference_identity_conflict',rows[3]['prediction_failures'])
        self.assertEqual(rows[3]['prediction_elapsed_ms'],200)
        self.assertAlmostEqual(rows[3]['prediction_travel_mm'],.2)
        self.assertTrue(rows[-1]['can_independent_validate'])
        self.assertEqual(rows[-1]['anchor']['frame_id'],6)


    def test_initial_confirmation_requires_consistent_target_before_first_anchor(self):
        rows=self.run_sequence({'anchor':[(360,260,150)],'other':[(210,330,255)]},
            extra=lambda index,name: {'image':'other.pgm'} if index==1 else {})
        self.assertFalse(rows[1]['can_independent_validate'],
                         'two different corners are not confirmation of one target')
        self.assertIsNone(rows[1]['anchor'])


    def test_normal_target_motion_keeps_geometry_and_truth_labels_cannot_change_decisions(self):
        images={'anchor':[(360,260,150)],'motion-1':[(362,261,150)],'motion-2':[(364,262,150)]}
        def context(index,name):
            x=360+2*max(0,index-1)
            y=260+max(0,index-1)
            return {'reference_point_at_exposure':[x,y], 'truth_pixel':[x,y], 'scene':'moving'}
        rows=self.run_sequence(images,extra=context)
        for row,truth in zip(rows[1:],[(360,260),(362,261),(364,262)]):
            self.assertTrue(row['can_independent_validate'])
            self.assertLess(sum((x-y)**2 for x,y in zip(row['final_pixel'],truth))**.5,1)
        changed=self.run_sequence(images,extra=lambda index,name:dict(context(index,name),
            truth_pixel=[-999,999],scene='unrelated'))
        for first,second in zip(rows,changed):
            for key in ('final_pixel','source','anchor_refreshed','identity_evidence','checkpoint_source_frame'):
                self.assertEqual(first[key],second[key])

if __name__=='__main__':
    unittest.main()
