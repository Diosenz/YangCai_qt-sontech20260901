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

if __name__=='__main__':
    unittest.main()
