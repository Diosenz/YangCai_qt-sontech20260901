"""通过公开 JSONL CLI 检查显式原生程序选择；不运行编译器或替代检测算法。"""
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent


class NativeRuntimeCliTests(unittest.TestCase):
    @unittest.skipUnless(os.environ.get('I2R_KCF_REPLAY_BIN_DIR'), 'requires actual CMake replay programs')
    def test_prebuilt_evidence_cli_accepts_a_new_transaction_cache(self):
        folder = pathlib.Path(tempfile.mkdtemp(prefix='kcf-native-fresh-cache-'))
        manifest = folder / 'input.jsonl'
        manifest.write_text(json.dumps({'image': None, 'frame_id': 1}) + chr(10), encoding='utf-8')
        run = subprocess.run([sys.executable, str(HERE / 'replay.py'), str(manifest),
                              '--evidence', '--cache', str(folder / 'new-cache'),
                              '--output', str(folder / 'result')],
                             capture_output=True, text=True, encoding='utf-8')
        self.assertEqual(run.returncode, 0, run.stderr)
        rows = [json.loads(line) for line in (folder / 'result/results.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertEqual(len(rows), 1)
        self.assertFalse(rows[0]['available'])

    def test_explicit_native_directory_without_manifest_fails_without_fallback(self):
        folder = pathlib.Path(tempfile.mkdtemp(prefix='kcf-native-runtime-'))
        manifest = folder / 'input.jsonl'
        manifest.write_text(json.dumps({'image': None, 'frame_id': 1}) + chr(10), encoding='utf-8')
        binaries = folder / 'native'
        binaries.mkdir()
        environment = dict(os.environ, I2R_KCF_REPLAY_BIN_DIR=str(binaries))
        run = subprocess.run([sys.executable, str(HERE / 'replay.py'), str(manifest),
                              '--output', str(folder / 'result')],
                             env=environment, capture_output=True, text=True, encoding='utf-8')
        self.assertNotEqual(run.returncode, 0)
        self.assertIn('replay-native.json', run.stderr)
        self.assertFalse((folder / 'result/results.jsonl').exists())


if __name__ == '__main__':
    unittest.main()
