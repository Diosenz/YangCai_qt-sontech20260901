"""只读检查连续采集是否可完整回放；不把相机未处理的曝光当作采集丢帧。"""
import argparse
import csv
import hashlib
import json
import math
import pathlib
import struct
import zlib


def rows(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream, delimiter='\t'))


def png_size(data):
    if data[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('Invalid PNG signature')
    offset, size, ended = 8, None, False
    while offset + 12 <= len(data):
        length = struct.unpack('>I', data[offset:offset + 4])[0]
        end = offset + 12 + length
        if end > len(data):
            raise ValueError('Truncated PNG')
        kind = data[offset + 4:offset + 8]
        chunk = data[offset + 8:end - 4]
        expected = struct.unpack('>I', data[end - 4:end])[0]
        if zlib.crc32(kind + chunk) & 0xffffffff != expected:
            raise ValueError('PNG checksum mismatch')
        if kind == b'IHDR':
            size = list(struct.unpack('>II', chunk[:8]))
            if chunk[8:10] != b'\x08\x00':
                raise ValueError('Replay image is not the actual 8-bit grayscale detector input')
        offset = end
        if kind == b'IEND':
            ended = True
            break
    if not size or not ended or offset != len(data):
        raise ValueError('Incomplete PNG structure')
    return size


def input_problems(context):
    """严格验证本轮生产paper入口契约；缺失数据不能由native默认值补齐。"""
    errors = []
    required = ('session_id', 'frame_id', 'timestamp_ms', 'reference_revision', 'paper_tracking',
                'bounded_search', 'detector_actions', 'simulation_valid', 'has_weld_point',
                'simulation_frame_id', 'sim_line', 'projection', 'seam_index', 'evidence_policy',
                'evidence_options', 'enable_gap', 'reference_point_at_exposure',
                'reference_exposure_frame_id', 'input_image_kind')
    for key in required:
        if key not in context:
            errors.append('missing ' + key)
    if errors:
        return errors
    finite = lambda value: type(value) in (float, int) and math.isfinite(value)
    point = lambda value: isinstance(value, list) and len(value) == 2 and all(map(finite, value))
    for key in ('session_id', 'frame_id', 'timestamp_ms', 'reference_revision', 'seam_index',
                'simulation_frame_id', 'reference_exposure_frame_id'):
        if type(context[key]) is not int or abs(context[key]) >= 2**53:
            errors.append('invalid integer ' + key)
    for key in ('paper_tracking', 'bounded_search', 'simulation_valid', 'has_weld_point',
                'evidence_policy', 'enable_gap'):
        if type(context[key]) is not bool:
            errors.append('invalid boolean ' + key)
    if not context['paper_tracking'] or context['enable_gap']:
        errors.append('unsupported detector profile')
    if context['input_image_kind'] != 'detector_gray8':
        errors.append('detector grayscale pixels not captured')
    if not isinstance(context['detector_actions'], list) or any(
            action not in ('reset', 'restart', 'gap') for action in context['detector_actions']):
        errors.append('unsupported lifecycle action')
    if not isinstance(context['sim_line'], list) or not all(map(point, context['sim_line'])):
        errors.append('invalid simulation polyline')
    if context['simulation_valid']:
        if context['simulation_frame_id'] != context['frame_id']:
            errors.append('simulation belongs to a different exposure')
        if context['has_weld_point'] and not point(context.get('sim_point')):
            errors.append('valid simulation weld point missing')
    reference = context['reference_point_at_exposure']
    if reference is not None and (not point(reference)
            or context['reference_exposure_frame_id'] != context['frame_id']):
        errors.append('invalid evidence reference')
    options = context['evidence_options']
    keys = ('enable_mother_constraints', 'enable_continuous_output', 'max_continuous_step_pixel',
            'anchor_confirmation_frames', 'recovery_confirmation_frames', 'identity_corridor_pixel',
            'identity_min_patch_score', 'identity_min_branch_score', 'identity_min_uniqueness',
            'max_recovery_step_pixel', 'max_prediction_ms', 'max_prediction_travel_mm')
    if not isinstance(options, dict) or any(key not in options for key in keys):
        errors.append('incomplete evidence options')
    else:
        for key in keys:
            value = options[key]
            valid = type(value) is bool if key.startswith('enable_') else finite(value) and value >= 0
            if not valid:
                errors.append('invalid evidence option ' + key)
        for key in ('anchor_confirmation_frames', 'recovery_confirmation_frames', 'identity_corridor_pixel'):
            if type(options[key]) is not int or not 1 <= options[key] <= 10000:
                errors.append('invalid evidence integer ' + key)
    projection = context['projection']
    calibration = projection.get('calibration', {}) if isinstance(projection, dict) else {}
    if not isinstance(calibration, dict):
        calibration = {}
    keys = ('focal_length_m', 'sx_m', 'sy_m', 'cx_pixel', 'cy_pixel', 'kappa',
            'laser_plane_a', 'laser_plane_b', 'laser_plane_c')
    if any(not finite(calibration.get(key)) for key in keys):
        errors.append('incomplete calibration')
    elif any(calibration[key] <= 0 for key in ('focal_length_m', 'sx_m', 'sy_m')):
        errors.append('invalid camera calibration')
    for key in ('camera_in_tool', 'tool_in_robot_base', 'robot_base_to_positioner_tcp'):
        value = projection.get(key) if isinstance(projection, dict) else None
        if not isinstance(value, list) or len(value) != 12 or not all(map(finite, value)):
            errors.append('incomplete transform ' + key)
        else:
            # 与生产回放保持同一刚体约束，避免审计通过后输入仍被拒绝。
            r = [value[row * 3:row * 3 + 3] for row in range(3)]
            orthogonal_error = sum((sum(r[k][i] * r[k][j] for k in range(3))
                                    - (1 if i == j else 0)) ** 2
                                   for i in range(3) for j in range(3)) ** 0.5
            determinant = (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
                           - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
                           + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))
            if orthogonal_error > 1e-6 or abs(determinant - 1) > 1e-6:
                errors.append('invalid rigid transform ' + key)
    if not isinstance(projection, dict) or not projection.get('provenance'):
        errors.append('projection provenance missing')
    return errors


def audit(directory):
    directory = pathlib.Path(directory).resolve()
    problems, hashes = [], []
    try:
        summary = dict(line.split('\t', 1) for line in
                       (directory / 'session_summary.txt').read_text(encoding='utf-8-sig')
                       .splitlines() if '\t' in line)
        if summary.get('session_state') in (None, 'running'):
            problems.append('Session has not been closed and drained')
        contexts = [json.loads(line) for line in
                    (directory / 'replay_context.jsonl').read_text(encoding='utf-8-sig').splitlines()]
        index = rows(directory / 'replay_index.tsv')
        manifest = rows(directory / 'replay_images/image_manifest.tsv')
        outcomes = {row['stem']: row for row in manifest}
        if len(outcomes) != len(manifest):
            problems.append('Duplicate image outcomes')
        if len(contexts) != len(index) or not contexts:
            problems.append('Missing contexts or capture index rows')
        if contexts and 'reset' not in contexts[0].get('detector_actions', []):
            problems.append('Capture does not establish a fresh detector history')
        events = [json.loads(line) for line in
                  (directory / 'unified_session.jsonl').read_text(encoding='utf-8-sig').splitlines()]
        observed, seen = [], set()
        for event in events:
            if (event.get('event') != 'observation' or not event.get('trigger_accepted')
                    or event.get('phase') not in ('initial_detection_running', 'online_running')):
                continue
            identity = (int(event['session_id']), int(event['frame_id']))
            if identity not in seen:
                observed.append(identity)
                seen.add(identity)
        captured = [(int(row['session_id']), int(row['frame_id'])) for row in contexts]
        if observed != captured:
            problems.append('Capture does not cover the recorded processed exposure sequence')
        for sequence, (context, entry) in enumerate(zip(contexts, index), 1):
            prefix = f'Exposure {context["frame_id"]}: '
            if (context.get('capture_sequence') != sequence
                    or int(entry['sequence']) != sequence
                    or int(entry['frame_id']) != context['frame_id']
                    or entry['source_image'] != context['image']):
                problems.append(prefix + 'sequence/index mismatch')
            if entry['request_status'] != 'queued':
                problems.append(prefix + entry['request_status'])
                continue
            if not context.get('detector_input_complete'):
                problems.append(prefix + 'detector input unavailable')
            problems.extend(prefix + error for error in input_problems(context))
            image = (directory / context['image']).resolve()
            if not image.is_relative_to(directory):
                problems.append(prefix + 'image outside session')
                continue
            stem = image.name.removesuffix('_source.png')
            outcome = outcomes.get(stem, {})
            if outcome.get('status') != 'saved':
                problems.append(prefix + 'image not confirmed saved')
                continue
            data = image.read_bytes()
            if (len(data) != int(outcome['encoded_bytes'])
                    or png_size(data) != context.get('image_size')):
                problems.append(prefix + 'image size/content mismatch')
            hashes.append(dict(frame_id=context['frame_id'], sha256=hashlib.sha256(data).hexdigest()))
    except (OSError, ValueError, KeyError, TypeError, IndexError, AttributeError, struct.error) as error:
        problems.append(str(error))
    return dict(complete=not problems, problems=problems, images=hashes,
                scope='Processed detector exposures only; not every camera exposure or physical accuracy')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=pathlib.Path)
    report = audit(parser.parse_args().session)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    raise SystemExit(0 if report['complete'] else 1)
