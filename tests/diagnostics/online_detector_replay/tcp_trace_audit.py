"""只读审计正式会话的关联完整性；未知发送、执行和固定参考误差不补推断。"""
import argparse
import collections
import hashlib
import json
import pathlib

from performance_report import number, statistics_ms


def trace_key(row):
    trace = row.get('local_trace') or {}
    session, command = trace.get('session_id'), trace.get('command_id')
    if session is None or command is None:
        return None
    return str(session), str(command)


def exposure_key(exposure):
    """曝光身份必须同时保留会话、帧与参考版本；缺项不进行弱关联。"""
    if not exposure:
        return None
    values = [exposure.get(field) for field in ('session_id', 'frame_id', 'reference_revision')]
    return tuple(str(value) for value in values) if all(value is not None for value in values) else None


def source_counts(keys, observed_keys):
    return dict(with_observed_key=sum(key in observed_keys for key in keys if key is not None),
        without_observed_key=sum(key not in observed_keys for key in keys if key is not None),
        missing_key=sum(key is None for key in keys))


def audit(rows):
    """仅关联同会话命令ID；recorded_at是主机记录时刻，completed不证明设备执行。"""
    events = collections.Counter(row.get('event') for row in rows)
    commands = [row for row in rows if row.get('event') == 'command' and row.get('type') == 'realtime']
    receipts = [row for row in rows if row.get('event') == 'receipt']
    command_keys = collections.Counter(key for row in commands if (key := trace_key(row)) is not None)
    receipt_keys = collections.Counter(key for row in receipts if (key := trace_key(row)) is not None)
    observations = [row for row in rows if row.get('event') == 'observation']
    observed_keys = collections.Counter(key for row in observations if (key := exposure_key(row)) is not None)
    targets = [row for row in rows if row.get('event') == 'control_target']
    feedback = [row for row in rows if row.get('event') == 'robot_state'
                and row.get('phase') == 'online_running']
    times = [number(row.get('recorded_at_ms')) for row in feedback]
    intervals = [b-a for a, b in zip(times, times[1:]) if a is not None and b is not None and b >= a]
    return dict(schema_version=1, event_counts=dict(events),
        exposure_attribution=dict(observations=len(observations),
            missing_observation_key=sum(exposure_key(row) is None for row in observations),
            duplicate_observation_keys=sum(count-1 for count in observed_keys.values()),
            target_first_point=source_counts([exposure_key(row.get('source_exposure')) for row in targets], observed_keys),
            target_direction_point=source_counts([exposure_key(row.get('direction_source_exposure')) for row in targets], observed_keys)),
        realtime_commands=dict(generated=len(commands), unique_keys=len(command_keys),
            missing_trace=sum(trace_key(row) is None for row in commands),
            duplicate_keys=sum(count-1 for count in command_keys.values()),
            unique_with_receipt=len(command_keys.keys() & receipt_keys.keys()),
            unique_without_receipt=len(command_keys.keys() - receipt_keys.keys())),
        receipts=dict(total=len(receipts), missing_trace=sum(trace_key(row) is None for row in receipts),
            unique_without_generated_command=len(receipt_keys.keys() - command_keys.keys()),
            duplicate_keys=sum(count-1 for count in receipt_keys.values()),
            reported_sent=sum(row.get('sent') is True for row in receipts),
            reported_response_received=sum(row.get('response_received') is True for row in receipts),
            unknown_execution=sum(row.get('execution_semantics') in (None, 'unknown_execution') for row in receipts)),
        feedback_online_running=dict(records=len(feedback), unknown_recording_time=sum(t is None for t in times),
            nonmonotonic_intervals=sum(a is not None and b is not None and b < a for a,b in zip(times,times[1:]))),
        feedback_recording_intervals_ms=statistics_ms(intervals),
        command_merge_count=None, send_to_response_ms=None, tcp_fixed_reference_error_mm=None,
        field_acceptance=False,
        limitations=[
            'Generated commands without receipts are unassociated; not proven merged, dropped or executed.',
            'Receipt completed and response_received do not establish robot execution.',
            'Recording intervals measure host logging cadence, not device sampling or command sending cadence.',
            'No independent send/receive timestamps or merge events are supplied by this audit schema.',
            'No frozen independent reference, coordinate transform and prelabelled business windows supplied; TCP error unevaluated.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=pathlib.Path)
    parser.add_argument('--output', type=pathlib.Path, help='New report file; existing files are never overwritten')
    args = parser.parse_args()
    raw = args.session.read_bytes()
    rows = [json.loads(line) for line in raw.decode('utf-8-sig').splitlines() if line.strip()]
    report = audit(rows)
    report['source'] = dict(path=str(args.session.resolve()), sha256=hashlib.sha256(raw).hexdigest())
    encoded = json.dumps(report, ensure_ascii=False, indent=2)+'\n'
    if args.output:
        with args.output.open('x', encoding='utf-8') as output:
            output.write(encoded)
    else:
        print(encoded, end='')


if __name__ == '__main__':
    main()
