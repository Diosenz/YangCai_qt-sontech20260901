"""Freeze incident evidence and existing executable checks; never access devices.

Only derived files beside this script are written. Source code, recording inputs,
and earlier analyses are read-only. All times in results use Asia/Shanghai.
"""
from pathlib import Path
import collections
import csv
import datetime
import hashlib
import json
import subprocess
import sys

sys.dont_write_bytecode = True
OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
SESSION = ROOT / 'output/online_correction/20261021_235225_324'
ARCHIVE = ROOT / 'output/online_local_registration/20261021_155225_278_session_1'
TZ = datetime.timezone(datetime.timedelta(hours=8))


def stamp(value):
    return datetime.datetime.fromtimestamp(int(value) / 1000, TZ).isoformat(timespec='milliseconds')


def dump(name, value):
    (OUT / name).write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')


def main():
    diagnostic = SESSION / 'vision_diagnostics.tsv'
    with diagnostic.open(encoding='utf-8-sig') as stream:
        rows = list(csv.DictReader(stream, delimiter='\t'))
    by_frame = {row['frame_id']: row for row in rows}
    online = [row for row in rows if row['phase'] == 'online_running' and row['result_valid'] == '1']
    candidates = []
    inputs = [diagnostic, SESSION / 'session_summary.txt']
    for path in sorted(ARCHIVE.glob('*.json')):
        inputs.append(path)
        value = json.loads(path.read_text(encoding='utf-8-sig'))
        if 'accepted_before_archive' in value:
            candidates.append(value)
    images = [SESSION / row['source_image'] for row in rows if row['source_image']]
    inputs.extend(images)
    production_roots = ['Camera', 'Localization', 'Vision', 'Control', 'IBVS',
                        'DigitalTwin', 'RobotComm', 'UI', 'tests', 'config', 'docs']
    protected = [ROOT / 'AGENTS.md', ROOT / 'CMakeLists.txt']
    for directory in production_roots:
        protected.extend(path for path in (ROOT / directory).rglob('*') if path.is_file())
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in sorted(set(protected))}
    protected_path = OUT / 'protected_inputs_before.json'
    if not protected_path.exists():
        dump(protected_path.name, hashes)
    else:
        assert hashes == json.loads(protected_path.read_text(encoding='utf-8')), 'Protected files changed'
    dump('incident_input_hashes.json', {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(set(inputs))})
    screenshot = by_frame['17927']
    index = rows.index(screenshot)
    first = last = index
    while first > 0 and rows[first - 1]['geometry_valid'] == '0':
        first -= 1
    while last + 1 < len(rows) and rows[last + 1]['geometry_valid'] == '0':
        last += 1
    baseline = {
        'scope': 'offline feasibility only; no production edits, build, device access or motion',
        'target_at_tack': 'continue original seam beneath tack; do not follow tack surface',
        'session': SESSION.name,
        'diagnostic_count': len(rows),
        'valid_online_count': len(online),
        'online_source_counts': dict(collections.Counter(row['point_source'] for row in online)),
        'saved_image_count': len(images),
        'candidate_count': len(candidates),
        'candidate_accepted_count': sum(item['accepted_before_archive'] for item in candidates),
        'writeback_count': len(list(ARCHIVE.glob('*.writeback.json'))),
        'verified_count': len(list(ARCHIVE.glob('*.verified.json'))),
        'screenshot_frame': screenshot,
        'geometry_invalid_run': {
            'first_frame': rows[first]['frame_id'], 'last_frame': rows[last]['frame_id'],
            'count': last - first + 1,
            'start': stamp(rows[first]['camera_timestamp_ms']),
            'end': stamp(rows[last]['camera_timestamp_ms'])},
        'protected_file_count': len(hashes),
        'recording_limit': 'sparse images and geometry-gated cloud windows; no physical ground truth'}
    dump('baseline.json', baseline)
    checks = []
    for name in ['i2r_rlc_online_seam_continuity_tests', 'i2r_rlc_online_seam_refiner_tests']:
        path = ROOT / 'out/build/qt-debug/tests' / (name + '.exe')
        if not path.exists():
            checks.append({'name': name, 'status': 'unavailable'})
            continue
        completed = subprocess.run([str(path)], cwd=path.parent, capture_output=True, timeout=30)
        output = completed.stdout.decode('utf-8', errors='replace') + completed.stderr.decode('utf-8', errors='replace')
        (OUT / (name + '.log')).write_text(output, encoding='utf-8')
        checks.append({'name': name, 'exit_code': completed.returncode,
                       'binary_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                       'status': 'passed' if completed.returncode == 0 else 'failed',
                       'rebuilt': False})
    dump('existing_executable_checks.json', checks)
    print(json.dumps({'baseline': OUT.as_posix(), 'source_counts': baseline['online_source_counts'],
                      'protected_files': len(hashes), 'checks': checks}, ensure_ascii=False))


if __name__ == '__main__':
    main()
