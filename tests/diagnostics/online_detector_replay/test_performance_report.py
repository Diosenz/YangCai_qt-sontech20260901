"""报告的公开 JSON 行为：曝光去重、缺失证据和固定窗口。"""
import unittest

from performance_report import summarize


class PerformanceReportTests(unittest.TestCase):
    def test_fixed_windows_keep_rejections_predictions_and_unique_exposures(self):
        rows = []
        for i in range(102):
            rows.append(dict(event='observation', session_id='7', frame_id=str(i),
                             camera_timestamp_ms=str(1000 + i * 100), valid=True,
                             estimated=False, localization_evidence={'current_image_supported': True},
                             space_parent_tcp_mm=[i * 0.1, 0, 0],
                             performance=dict(processing_started_ms=str(1020 + i * 100),
                                              processing_finished_ms=str(1050 + i * 100),
                                              runtime_received_ms=str(1060 + i * 100),
                                              image_preparation_ms=2.5)))
        rows[1]['valid'] = False
        rows[2]['estimated'] = True
        rows[3]['localization_evidence'] = {}
        rows.append(rows[4].copy())
        report = summarize(rows, [{'label': 'normal', 'start_ms': 1000, 'end_ms': 11200}])
        window = report['intervals'][0]['windows'][0]
        self.assertEqual(window['duration_ms'], 10000)
        self.assertEqual(window['effective_new_exposures'], 97)
        self.assertEqual(window['effective_hz'], 9.7)
        self.assertEqual(window['duplicate_exposures'], 1)
        self.assertEqual(window['invalid_results'], 1)
        self.assertEqual(window['estimated_results'], 1)
        self.assertEqual(window['unknown_image_support'], 1)
        self.assertEqual(window['exposure_to_runtime_ms']['p95'], 60)
        self.assertEqual(window['image_preparation_ms']['p95'], 2.5)
        self.assertEqual(window['longest_effective_gap_ms'], 400)
        self.assertAlmostEqual(window['spatial_step_mm']['max'], 0.4)
        self.assertFalse(report['intervals'][0]['windows'][1]['full_window'])

    def test_missing_or_reversed_timestamps_are_not_reported_as_zero_latency(self):
        rows = [dict(event='observation', session_id='1', frame_id='1',
                     camera_timestamp_ms='1000', valid=True,
                     performance={'processing_started_ms': '1020',
                                  'processing_finished_ms': '1050', 'runtime_received_ms': '999'}),
                dict(event='observation', frame_id='2', camera_timestamp_ms='-1', valid=True)]
        report = summarize(rows, [{'label': 'unlabelled', 'start_ms': 1000, 'end_ms': 11000}])
        window = report['intervals'][0]['windows'][0]
        self.assertIsNone(window['exposure_to_runtime_ms']['p95'])
        self.assertEqual(window['exposure_to_runtime_ms']['count'], 0)
        self.assertEqual(window['invalid_latency_sequences'], 1)
        self.assertEqual(window['effective_new_exposures'], 0)
        self.assertEqual(report['unknown_exposure_time'], 1)
        self.assertFalse(report['intervals'][0]['prelabelled_for_acceptance'])
        self.assertIsNone(window['image_preparation_ms']['mean'])

    def test_long_gap_and_spatial_step_survive_window_boundaries(self):
        rows = [dict(event='observation', session_id='1', frame_id=str(i),
                     camera_timestamp_ms=str(t), valid=True, estimated=False,
                     localization_evidence={'current_image_supported': True},
                     space_parent_tcp_mm=[i * 3, 0, 0]) for i, t in enumerate([1000, 31000])]
        section = summarize(rows, [{'label': 'normal', 'start_ms': 1000,
                                    'end_ms': 41000}])['intervals'][0]
        self.assertEqual(section['longest_effective_gap_ms'], 30000)
        self.assertEqual(section['spatial_step_mm']['max'], 3)

    def test_interval_boundaries_do_not_count_one_exposure_twice(self):
        row = dict(event='observation', session_id='1', frame_id='1',
                   camera_timestamp_ms='11000', valid=True, estimated=False,
                   localization_evidence={'current_image_supported': True})
        result = summarize([row], [{'label': 'normal', 'start_ms': 1000, 'end_ms': 11000},
                                   {'label': 'reflection', 'start_ms': 11000, 'end_ms': 21000}])
        self.assertEqual(result['intervals'][0]['windows'][0]['effective_new_exposures'], 0)
        self.assertEqual(result['intervals'][1]['windows'][0]['effective_new_exposures'], 1)


if __name__ == '__main__':
    unittest.main()
