"""通过同一个生产 process 入口验收 C++ 策略，不运行 Python EvidenceSession。"""
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

from replay import build


class ProductionPolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = build(pathlib.Path(__file__).resolve().parents[3] /
                           'output/analysis/production_policy_build')[0]

    def run_frames(self, frames):
        folder = pathlib.Path(tempfile.mkdtemp(prefix='kcf-production-'))
        for name, shift in [('visible', 0), ('shifted', 5), ('far', 30)]:
            pixels = bytes(225 if abs(y - (260 - .45 * abs(x - 360 - shift))) < 1.5
                           else 14 for y in range(480) for x in range(640))
            (folder / (name + '.pgm')).write_bytes(b'P5\n640 480\n255\n' + pixels)
        (folder/'blank.pgm').write_bytes(b'P5\n640 480\n255\n'+bytes([14])*640*480)
        for name in ['hidden_center', 'single_side']:
            pixels = bytes(225 if abs(y - (260 - .45 * abs(x - 360))) < 1.5
                           and (abs(x-360)>35 if name=='hidden_center' else x<325)
                           else 14 for y in range(480) for x in range(640))
            (folder/(name+'.pgm')).write_bytes(b'P5\n640 480\n255\n'+pixels)
        values = []
        for index, updates in enumerate(frames):
            value = dict(image='visible.pgm', kind='synthetic', frame_id=index+1,
                         timestamp_ms=1000+index*100, evidence_policy=True,
                         projection={'fixture':'synthetic_identity_1mm_per_pixel'})
            value.update(updates)
            values.append(value)
        manifest = folder/'input.jsonl'
        self.last_manifest = manifest
        manifest.write_text(''.join(json.dumps(row)+'\n' for row in values), encoding='utf-8')
        run = subprocess.run([str(self.binary), str(manifest)], capture_output=True,
                             text=True, encoding='utf-8')
        self.assertEqual(run.returncode, 0, run.stderr)
        return [json.loads(line) for line in run.stdout.splitlines()]

    def test_production_confirms_new_geometry_without_simulation(self):
        rows = self.run_frames([{}, {}, {'frame_id':2}, {'frame_id':3}, {'frame_id':4}])
        self.assertFalse(rows[0]['available'], 'unconfirmed geometry must not train or drive control')
        self.assertTrue(rows[1]['available'])
        self.assertFalse(rows[2]['available'], 'duplicate exposure is not new geometry')
        self.assertFalse(rows[2]['training_committed'])
        self.assertFalse(rows[2]['template_reinitialized'])
        self.assertFalse(rows[3]['available'])
        self.assertTrue(rows[4]['available'])

    def test_missing_then_recovered_geometry_requires_new_confirmation(self):
        rows = self.run_frames([{}, {}, {'image':'blank.pgm'}, {}, {}])
        self.assertFalse(rows[2]['available'])
        self.assertFalse(rows[3]['available'])
        self.assertTrue(rows[4]['available'])

    def test_distant_candidate_does_not_replace_reliable_template(self):
        rows = self.run_frames([{}, {}, {'image':'far.pgm'}, {'image':'far.pgm'}, {}])
        self.assertFalse(rows[2]['available'])
        self.assertFalse(rows[3]['available'])
        self.assertFalse(rows[2]['template_reinitialized'])
        self.assertFalse(rows[3]['training_committed'])

    def continuation(self, index, **updates):
        value = dict(frame_id=index, timestamp_ms=1000+index*100,
                     weld_progress_mm=index*.1, progress_reference='fixed-workpiece-path',
                     progress_provenance='synthetic exposure-associated arc including external axes',
                     progress_exposure_frame_id=index, reference_revision=1,
                     reference_point_at_exposure=[100+index,100], reference_exposure_frame_id=index,
                     max_prediction_ms=1000, max_prediction_travel_mm=10)
        value.update(updates)
        return value

    def test_time_budget_and_prediction_never_refresh_anchor_or_project(self):
        rows = self.run_frames([self.continuation(i, max_prediction_ms=200,
                                image='blank.pgm' if i>2 else 'visible.pgm') for i in range(1,5)])
        self.assertTrue(rows[2]['available'])
        self.assertEqual(rows[2]['source'], 'bounded_prediction')
        self.assertEqual(rows[2]['anchor_frame_id'], 2)
        self.assertFalse(rows[2]['anchor_refreshed'])
        self.assertFalse(rows[2]['control_eligible'])
        self.assertFalse(rows[2]['can_independent_validate'])
        self.assertIsNone(rows[2]['positioner_tcp_mm'])
        self.assertFalse(rows[3]['available'])
        self.assertEqual(rows[3]['anchor_frame_id'],2)

    def test_reverse_travel_consumes_budget(self):
        values = [0,1,3,1]
        rows = self.run_frames([self.continuation(i+1, weld_progress_mm=p,
                    max_prediction_travel_mm=4, image='blank.pgm' if i>=2 else 'visible.pgm')
                    for i,p in enumerate(values)])
        self.assertTrue(rows[2]['available'])
        self.assertEqual(rows[2]['prediction_travel_mm'], 2)
        self.assertFalse(rows[3]['available'])
        self.assertEqual(rows[3]['prediction_travel_mm'], 4)

    def test_reference_revision_bridge_uses_old_version_current_exposure_only(self):
        frames = [self.continuation(1), self.continuation(2),
                  self.continuation(3, image='blank.pgm', reference_revision=2,
                    reference_point_at_exposure=[500,500], previous_reference_revision=1,
                    previous_reference_point_at_exposure=[103,100], previous_reference_exposure_frame_id=3),
                  self.continuation(4, image='blank.pgm', reference_revision=3,
                    reference_point_at_exposure=[900,900])]
        rows=self.run_frames(frames)
        self.assertTrue(rows[2]['available'])
        self.assertAlmostEqual(rows[2]['final_pixel'][0]-rows[1]['final_pixel'][0],1)
        self.assertEqual(rows[2]['anchor_frame_id'],2)
        self.assertFalse(rows[3]['available'])

    def test_missing_progress_latches_until_new_geometry(self):
        rows=self.run_frames([self.continuation(1),self.continuation(2),
            self.continuation(3,image='blank.pgm',weld_progress_mm=None),
            self.continuation(4,image='blank.pgm')])
        self.assertFalse(rows[2]['available'])
        self.assertFalse(rows[3]['available'])

    def test_invalid_projection_cannot_create_an_anchor(self):
        rows=self.run_frames([{'projection':{}}, {}, {}])
        self.assertFalse(rows[0]['available'])
        self.assertFalse(rows[1]['available'])
        self.assertTrue(rows[2]['control_eligible'])

    def test_hidden_center_and_single_side_remain_partial_estimates(self):
        rows = self.run_frames([self.continuation(1), self.continuation(2),
            self.continuation(3, image='hidden_center.pgm'),
            self.continuation(4, image='single_side.pgm')])
        for row, rank in zip(rows[2:], [2, 1]):
            self.assertTrue(row['available'])
            self.assertEqual(row['source'], 'mother_material_estimate')
            self.assertEqual(row['mother_support_rank'], rank)
            self.assertEqual(row['anchor_frame_id'], 2)
            self.assertFalse(row['control_eligible'])
            self.assertFalse(row['can_independent_validate'])
            self.assertFalse(row['training_committed'])

    def test_bounded_online_search_can_establish_confirmed_geometry(self):
        rows=self.run_frames([{'bounded_search':True} for _ in range(160)])
        self.assertTrue(any(row['control_eligible'] for row in rows))
        self.assertTrue(all(row['refinement_calls'] <= 2 for row in rows))

    def test_missing_reference_never_outputs_unanchored_prediction(self):
        rows=self.run_frames([self.continuation(1),self.continuation(2),
            self.continuation(3, image='blank.pgm',reference_point_at_exposure=None)])
        self.assertFalse(rows[2]['available'])
        self.assertEqual(rows[2]['anchor_frame_id'],2)

    def test_production_cli_runs_native_policy_without_python_evidence_layer(self):
        self.run_frames([{},{}])
        records=[json.loads(line) for line in self.last_manifest.read_text().splitlines()]
        for record in records:
            record.pop('evidence_policy')
        manifest=self.last_manifest.with_name('cli-input.jsonl')
        manifest.write_text(''.join(json.dumps(record)+'\n' for record in records),encoding='utf-8')
        output=manifest.parent/'production-cli'
        script=pathlib.Path(__file__).with_name('replay.py')
        run=subprocess.run([sys.executable,str(script),str(manifest),'--production','--output',str(output)],
                           capture_output=True,text=True,encoding='utf-8')
        self.assertEqual(run.returncode,0,run.stderr)
        rows=[json.loads(line) for line in (output/'results.jsonl').read_text().splitlines()]
        self.assertFalse(rows[0]['available'])
        self.assertTrue(rows[1]['production_evidence_policy'])
        self.assertTrue(rows[1]['control_eligible'])
        self.assertNotIn('raw_detector',rows[1])
        self.assertTrue(json.loads((output/'metadata.json').read_text())['production_cpp_evidence_enabled'])


if __name__ == '__main__':
    unittest.main()
