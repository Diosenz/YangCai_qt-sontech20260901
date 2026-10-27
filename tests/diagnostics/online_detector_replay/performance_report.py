"""离线读取正式会话；不启动设备，不用 recorder 写入时刻替代结果接收。"""
import argparse
import hashlib
import json
import math
import pathlib
import statistics


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) and result >= 0 else None
    except (TypeError, ValueError):
        return None


def statistics_ms(values):
    """P95 使用 nearest-rank，缺测保留 null。空间距离也复用此分布格式。"""
    ordered = sorted(values)
    return dict(count=len(ordered), mean=statistics.fmean(ordered) if ordered else None,
                p95=ordered[math.ceil(len(ordered) * 0.95) - 1] if ordered else None,
                max=ordered[-1] if ordered else None)


def summarize(rows, intervals):
    """预标注区间为 [start,end)，窗口固定 10 s；尾窗单列，缺少支持不补推断。"""
    observations = [row for row in rows if row.get('event') == 'observation']
    known = [(number(row.get('camera_timestamp_ms')), row) for row in observations]
    report = dict(schema_version=1, clock='host wall-clock milliseconds; detector stages use steady_clock',
                  percentile='nearest-rank', window_ms=10000,
                  unknown_exposure_time=sum(t is None for t, _ in known), intervals=[])
    seen_intervals = []
    for interval in intervals:
        start, end = number(interval['start_ms']), number(interval['end_ms'])
        if start is None or end is None or end <= start:
            raise ValueError('Interval must have finite nonnegative start_ms < end_ms')
        if any(start < b and end > a for a, b in seen_intervals):
            raise ValueError('Prelabelled intervals must not overlap')
        seen_intervals.append((start, end))
        section = dict(label=interval['label'], start_ms=start, end_ms=end,
                       prelabelled_for_acceptance=interval['label'] in
                       ('normal', 'reflection', 'preweld', 'loss'), windows=[])
        selected = sorted([(t, row) for t, row in known if t is not None and start <= t < end],
                          key=lambda item: item[0])
        seen = set()
        cursor = start
        interval_times, interval_points = [], []
        while cursor < end:
            stop = min(cursor + 10000, end)
            window = dict(start_ms=cursor, end_ms=stop, duration_ms=stop-cursor,
                          full_window=stop-cursor == 10000, results=0, effective_new_exposures=0,
                          duplicate_exposures=0, unknown_exposure_identity=0, invalid_results=0,
                          estimated_results=0, unknown_image_support=0, invalid_latency_sequences=0)
            durations = {name: [] for name in ('exposure_to_runtime_ms', 'waiting_ms',
                         'worker_ms', 'delivery_ms', 'image_preparation_ms')}
            stages = {}
            effective_times, points = [], []
            for timestamp, row in selected:
                if not cursor <= timestamp < stop:
                    continue
                window['results'] += 1
                identity = (row.get('session_id'), row.get('frame_id'))
                identity_known = (identity[0] is not None and number(identity[1]) is not None)
                duplicate = identity_known and identity in seen
                if identity_known:
                    seen.add(identity)
                window['unknown_exposure_identity'] += not identity_known
                window['duplicate_exposures'] += duplicate
                window['invalid_results'] += not row.get('valid', False)
                window['estimated_results'] += bool(row.get('estimated', False))
                support = row.get('localization_evidence', {}).get('current_image_supported')
                window['unknown_image_support'] += not isinstance(support, bool)
                if (identity_known and not duplicate and row.get('valid') is True
                        and row.get('estimated') is False and support is True):
                    window['effective_new_exposures'] += 1
                    effective_times.append(timestamp)
                    interval_times.append(timestamp)
                    point = row.get('space_parent_tcp_mm')
                    if (isinstance(point, list) and len(point) == 3
                            and all(isinstance(p, (int, float)) and math.isfinite(p) for p in point)):
                        points.append(point)
                    else:
                        points.append(None)
                    interval_points.append(points[-1])
                timing = row.get('performance') or {}
                preparation = number(timing.get('image_preparation_ms'))
                if preparation is not None:
                    durations['image_preparation_ms'].append(preparation)
                started = number(timing.get('processing_started_ms'))
                finished = number(timing.get('processing_finished_ms'))
                received = number(timing.get('runtime_received_ms'))
                sequence = [v for v in (timestamp, started, finished, received) if v is not None]
                if sequence != sorted(sequence):
                    window['invalid_latency_sequences'] += 1
                else:
                    for name, a, b in [('exposure_to_runtime_ms', timestamp, received),
                                       ('waiting_ms', timestamp, started),
                                       ('worker_ms', started, finished),
                                       ('delivery_ms', finished, received)]:
                        if a is not None and b is not None:
                            durations[name].append(b-a)
                for name, value in (timing.get('detector') or {}).items():
                    value = number(value)
                    if value is not None:
                        stages.setdefault(name, []).append(value)
            window['effective_hz'] = window['effective_new_exposures'] * 1000 / (stop-cursor)
            window['longest_effective_gap_ms'] = max(b-a for a, b in zip(
                [cursor] + effective_times, effective_times + [stop]))
            window.update({name: statistics_ms(values) for name, values in durations.items()})
            window['detector'] = {name: statistics_ms(values) for name, values in stages.items()}
            window['spatial_step_mm'] = statistics_ms([
                math.dist(a, b) for a, b in zip(points, points[1:]) if a is not None and b is not None])
            section['windows'].append(window)
            cursor = stop
        section['longest_effective_gap_ms'] = max(b-a for a, b in zip(
            [start] + interval_times, interval_times + [end]))
        section['spatial_step_mm'] = statistics_ms([
            math.dist(a, b) for a, b in zip(interval_points, interval_points[1:])
            if a is not None and b is not None])
        report['intervals'].append(section)
    report['limitations'] = [
        'Unlabelled data and partial windows cannot establish normal-segment acceptance.',
        'Missing runtime timestamps cannot establish exposure-to-business-result latency.',
        'Spatial steps are adjacent valid 3D chords, not weld arc length or new observations.',
        'Wall-clock jumps are flagged when ordering reverses; clock synchronization is not inferred.',
        'Sparse replay and repeated images cannot establish continuous camera throughput.']
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=pathlib.Path)
    parser.add_argument('--intervals', type=pathlib.Path,
                        help='Prelabelled JSON list: label/start_ms/end_ms in exposure host epoch milliseconds')
    parser.add_argument('--output', type=pathlib.Path, required=True)
    args = parser.parse_args()
    source = args.session / 'unified_session.jsonl' if args.session.is_dir() else args.session
    data = source.read_bytes()
    rows = [json.loads(line) for line in data.decode('utf-8-sig').splitlines() if line.strip()]
    if args.intervals:
        intervals = json.loads(args.intervals.read_text(encoding='utf-8-sig'))
    else:
        times = [number(row.get('camera_timestamp_ms')) for row in rows if row.get('event') == 'observation']
        times = [value for value in times if value is not None]
        intervals = [dict(label='unlabelled', start_ms=min(times), end_ms=max(times)+1)] if times else []
    report = summarize(rows, intervals)
    report['input'] = dict(path=str(source.resolve()), sha256=hashlib.sha256(data).hexdigest(),
                           intervals_sha256=hashlib.sha256(args.intervals.read_bytes()).hexdigest()
                           if args.intervals else None)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(args.output)


if __name__ == '__main__':
    main()
