"""固定三模板实拍负载的等价性与本机Release预算；不冒充现场检测率。"""
import json
import math
import pathlib
import statistics
import subprocess
import unittest

from native_runtime import configured_native

ROOT = pathlib.Path(__file__).resolve().parents[3]


class EquivalentTrainingPerformanceTests(unittest.TestCase):
    def test_three_template_training_keeps_outputs_with_reduced_repeated_fft_work(self):
        binary, provenance = configured_native('DetectorReplay')
        process = subprocess.run([str(binary), '--tracker-benchmark', str(ROOT /
            'tests/fixtures/online_tracking_20261027/20261027_122726_191_3937.png')],
            capture_output=True, timeout=30)
        self.assertEqual(process.returncode, 0, process.stderr.decode('utf-8', errors='replace'))
        rows = [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
        expected = json.loads((ROOT/'tests/fixtures/online_tracking_20261027/training_reference.json')
                              .read_text(encoding='utf-8'))
        self.assertEqual(len(rows), len(expected))
        for index, (actual, reference) in enumerate(zip(rows, expected)):
            if index > 0:
                self.assertEqual(actual['templates'], 3)
            self.assertTrue(actual['training_committed'])
            for field in ['response_peak', 'response_psr', 'hog_weight', 'gray_weight']:
                self.assertTrue(math.isclose(actual[field], reference[field], abs_tol=1e-6, rel_tol=1e-7), field)
            self.assertLess(math.dist(actual['point'], reference['point']), 0.001)
        median = statistics.median(row['training_ms'] for row in rows[1:])
        print(f'Three-template equivalent training median={median:.2f}ms', flush=True)
        if provenance['dependency_versions'].get('configuration') == 'Release':
            self.assertLess(median, 50,
                'Local fixed-workload regression budget (baseline 54-60ms); not field acceptance')


if __name__ == '__main__':
    unittest.main()
