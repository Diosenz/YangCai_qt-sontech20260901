"""通过离线CLI核对命令、回执与曝光关联的证据边界。"""
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest


class TcpTraceAuditTests(unittest.TestCase):
    def run_audit(self, rows):
        evidence = pathlib.Path(__file__).resolve().parents[3]/'output/analysis/online_correction_optimization'
        evidence.mkdir(parents=True, exist_ok=True)
        directory = tempfile.mkdtemp(prefix='tcp-audit-test-', dir=evidence)
        source = pathlib.Path(directory)/'session.jsonl'
        source.write_text(''.join(json.dumps(row)+'\n' for row in rows), encoding='utf-8')
        process = subprocess.run([sys.executable, str(pathlib.Path(__file__).with_name(
            'tcp_trace_audit.py')), str(source)], capture_output=True, text=True, timeout=10)
        self.assertEqual(process.returncode, 0, process.stderr)
        return json.loads(process.stdout)

    def test_generated_and_completed_do_not_prove_execution_or_merge(self):
        trace = dict(session_id='1', command_id='1', target_generation='1')
        report = self.run_audit([
            dict(event='command', type='realtime', local_trace=trace, recorded_at_ms='100'),
            dict(event='command', type='realtime', local_trace=dict(trace, command_id='2')),
            dict(event='receipt', local_trace=trace, sent=True, response_received=True,
                 completed=True, execution_semantics='unknown_execution', recorded_at_ms='200'),
            dict(event='robot_state', phase='online_running', recorded_at_ms='100', state={'tcp_mm':[0,0,0]}),
            dict(event='robot_state', phase='online_running', recorded_at_ms='130', state={'tcp_mm':[1,0,0]})])
        self.assertEqual(report['realtime_commands']['unique_with_receipt'], 1)
        self.assertEqual(report['realtime_commands']['unique_without_receipt'], 1)
        self.assertIsNone(report['command_merge_count'])
        self.assertIsNone(report['send_to_response_ms'])
        self.assertIsNone(report['tcp_fixed_reference_error_mm'])
        self.assertEqual(report['receipts']['unknown_execution'], 1)
        self.assertEqual(report['feedback_recording_intervals_ms']['p95'], 30)
        self.assertFalse(report['field_acceptance'])

    def test_sessions_and_reference_revisions_do_not_cross_associate(self):
        exposure = dict(session_id='1', frame_id='10', reference_revision='1')
        report = self.run_audit([
            dict(event='command', type='realtime', local_trace=dict(session_id='1', command_id='1')),
            dict(event='receipt', local_trace=dict(session_id='2', command_id='1')),
            dict(event='observation', **exposure), dict(event='observation', **exposure),
            dict(event='control_target', source_exposure=exposure,
                 direction_source_exposure=dict(exposure, session_id='2')),
            dict(event='control_target', source_exposure=dict(exposure, reference_revision='2'),
                 direction_source_exposure=exposure),
            dict(event='control_target', source_exposure=None)])
        self.assertEqual(report['realtime_commands']['unique_with_receipt'], 0)
        self.assertEqual(report['receipts']['unique_without_generated_command'], 1)
        self.assertEqual(report['exposure_attribution']['duplicate_observation_keys'], 1)
        for kind in ['target_first_point', 'target_direction_point']:
            self.assertEqual(report['exposure_attribution'][kind]['with_observed_key'], 1)
            self.assertEqual(report['exposure_attribution'][kind]['without_observed_key'], 1)
            self.assertEqual(report['exposure_attribution'][kind]['missing_key'], 1)

    def test_output_cannot_overwrite_source_evidence(self):
        evidence = pathlib.Path(__file__).resolve().parents[3]/'output/analysis/online_correction_optimization'
        evidence.mkdir(parents=True, exist_ok=True)
        directory = pathlib.Path(tempfile.mkdtemp(prefix='tcp-audit-protection-', dir=evidence))
        source = directory/'session.jsonl'
        original = b'{"event":"session_start"}\n'
        source.write_bytes(original)
        process = subprocess.run([sys.executable, str(pathlib.Path(__file__).with_name(
            'tcp_trace_audit.py')), str(source), '--output', str(source)],
            capture_output=True, text=True, timeout=10)
        self.assertNotEqual(process.returncode, 0)
        self.assertEqual(source.read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
