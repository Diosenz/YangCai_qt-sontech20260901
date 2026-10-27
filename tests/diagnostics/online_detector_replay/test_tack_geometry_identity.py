"""捕获的预焊入口经生产process验证，防局部几何替换到相邻支系。"""
import hashlib
import json
import math
import pathlib
import subprocess
import time
import unittest

from native_runtime import configured_native

ROOT = pathlib.Path(__file__).resolve().parents[3]
FIXTURE = ROOT/'tests/fixtures/online_tack_instability_20261027'


class TackGeometryIdentityTests(unittest.TestCase):
    def test_local_geometry_cannot_replace_supported_target_with_neighboring_edge(self):
        provenance = json.loads((FIXTURE/'provenance.json').read_text(encoding='utf-8'))
        for name, expected in provenance['files'].items():
            data = (FIXTURE/name).read_bytes()
            if name.endswith('.jsonl'):
                data = data.replace(b'\r\n', b'\n')
            self.assertEqual(hashlib.sha256(data).hexdigest(), expected)
        rows = [json.loads(line) for line in (FIXTURE/'input.jsonl').read_text(encoding='utf-8').splitlines()]
        for row in rows:
            row['image'] = str(FIXTURE/row['image'])
        folder = ROOT/'output/analysis/detection_instability_20261027'/str(time.time_ns())
        folder.mkdir(parents=True)
        manifest = folder/'input.jsonl'
        manifest.write_text(''.join(json.dumps(row)+'\n' for row in rows), encoding='utf-8')
        binary, build = configured_native('DetectorReplay')
        process = subprocess.run([str(binary), str(manifest)], capture_output=True, timeout=20)
        self.assertEqual(process.returncode, 0, process.stderr.decode('utf-8', errors='replace'))
        actual = [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
        for row in actual:
            row.pop('image_png_base64', None)
        (folder/'result.json').write_text(json.dumps(dict(results=actual, build=build),indent=2),encoding='utf-8')
        self.assertEqual([row['frame_id'] for row in actual], [8154,8157,8160])
        self.assertTrue(all(row['detector_valid'] and row['final_pixel'] for row in actual),
            'Rejecting a conflicting local fit must not hide the target in this captured clip')
        self.assertLess(math.dist(actual[0]['final_pixel'], [458.5,595.8]), 2,
            'The visible pre-tack target establishes the local anchor, not a nearby stable wrong point')
        self.assertLess(max(math.dist(a['final_pixel'], b['final_pixel'])
                            for a,b in zip(actual,actual[1:])), 15,
            'Captured near-stationary tack onset must not jump 32px to the neighboring edge')


if __name__ == '__main__':
    unittest.main()
