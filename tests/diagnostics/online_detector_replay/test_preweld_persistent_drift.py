"""实拍受控序列通过生产process验证清晰接缝恢复，不替代完整现场重演。"""
import hashlib
import json
import math
import pathlib
import subprocess
import time
import unittest

from native_runtime import configured_native

ROOT = pathlib.Path(__file__).resolve().parents[3]
FIXTURE = ROOT/'tests/fixtures/preweld_persistent_drift_20261027'


class PreweldPersistentDriftTests(unittest.TestCase):
    def replay(self, frames):
        binary, build = configured_native('DetectorReplay')
        folder = ROOT/'output/analysis/preweld_drift_tdd_20261027'/str(time.time_ns())
        folder.mkdir(parents=True)
        manifest = folder/'input.jsonl'
        manifest.write_text(''.join(json.dumps(row)+'\n' for row in frames), encoding='utf-8')
        process = subprocess.run([str(binary),str(manifest)],capture_output=True,timeout=30)
        self.assertEqual(process.returncode,0,process.stderr.decode('utf-8',errors='replace'))
        rows = [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
        self.assertEqual([row['frame_id'] for row in rows],
                         [frame['frame_id'] for frame in frames],
                         'Replay must return every input in order before checking recovery')
        for row in rows:
            row.pop('image_png_base64',None)
        (folder/'results.json').write_text(json.dumps(dict(build=build,results=rows),indent=2),encoding='utf-8')
        return rows

    def load(self):
        provenance = json.loads((FIXTURE/'provenance.json').read_text(encoding='utf-8'))
        for name, expected in provenance['files'].items():
            data = (FIXTURE/name).read_bytes()
            if name.endswith('.jsonl'):
                data = data.replace(b'\r\n',b'\n')
            self.assertEqual(hashlib.sha256(data).hexdigest(),expected,name)
        frames = [json.loads(line) for line in (FIXTURE/'controlled-sequence.jsonl')
                  .read_text(encoding='utf-8').splitlines()]
        for frame in frames:
            frame['image'] = str(FIXTURE/frame['image'])
        return frames, provenance

    def test_clear_seam_recovers_after_tack_instead_of_staying_outside(self):
        frames,provenance = self.load()
        rows = self.replay(frames)
        self.assertTrue(rows[31]['paper_diagnostics']['response_statistics_ready'])
        for recovered in rows[-provenance['clear_inputs']:]:
            self.assertTrue(recovered['detector_valid'] and recovered['final_pixel'],
                'A clear seam must recover instead of being hidden')
            self.assertLess(math.dist(recovered['final_pixel'],provenance['clear_corner_manual_px']),
                provenance['tolerance_px'],'Persistent tack history must not retain the wrong old branch')
            self.assertTrue(recovered['control_eligible'] and recovered['image_observation_accepted'],
                'First and subsequent clear inputs must regain usable current-image support')

    def test_exposure_gaps_and_unknown_clock_do_not_prevent_clear_recovery(self):
        original,provenance = self.load()
        for clock in ['uneven','unknown','backward']:
            with self.subTest(clock=clock):
                frames = [dict(frame) for frame in original]
                offset = 0
                for index,frame in enumerate(frames):
                    if clock == 'unknown':
                        frame.pop('timestamp_ms',None)
                    elif clock == 'uneven':
                        offset += {36:1947,40:264,44:3102}.get(index,0)
                        frame['timestamp_ms'] += offset
                    elif index == 39:
                        frame['timestamp_ms'] -= 250
                rows = self.replay(frames)
                for recovered in rows[-provenance['clear_inputs']:]:
                    self.assertTrue(recovered['detector_valid'] and recovered['final_pixel'])
                    self.assertLess(math.dist(recovered['final_pixel'],provenance['clear_corner_manual_px']),
                        provenance['tolerance_px'],'Clock provenance must not amplify stale motion')
                    self.assertTrue(recovered['control_eligible'] and recovered['image_observation_accepted'])


if __name__ == '__main__':
    unittest.main()
