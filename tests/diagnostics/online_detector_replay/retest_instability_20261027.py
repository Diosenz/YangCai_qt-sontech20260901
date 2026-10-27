"""回放预焊实拍的突跳反例；连续性阈值仅为该近静态短片段的诊断断言。"""
import argparse
import json
import math
import pathlib
import subprocess
import time

from native_runtime import configured_native

ROOT = pathlib.Path(__file__).resolve().parents[3]
SESSION = ROOT/'output/online_correction/20261027_200039_905'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', default='8154,8157,8160')
    args = parser.parse_args()
    frames = {int(frame) for frame in args.frames.split(',')}
    contexts = [json.loads(line) for line in (SESSION/'loss_window_context.jsonl')
                .read_text(encoding='utf-8-sig').splitlines()]
    inputs = [dict(row, image=str(SESSION/row['image']), paper_tracking=True)
              for row in contexts if row['frame_id'] in frames]
    if {row['frame_id'] for row in inputs} != frames:
        raise ValueError('Requested exposure lacks captured context')
    output = ROOT/'output/analysis/detection_instability_20261027'/str(time.time_ns())
    output.mkdir(parents=True)
    manifest = output/'input.jsonl'
    manifest.write_text(''.join(json.dumps(row)+'\n' for row in inputs), encoding='utf-8')
    binary, provenance = configured_native('DetectorReplay')
    process = subprocess.run([str(binary), str(manifest)], capture_output=True, timeout=20)
    (output/'results.jsonl').write_bytes(process.stdout)
    if process.returncode:
        raise RuntimeError(process.stderr.decode('utf-8', errors='replace'))
    rows = [json.loads(line) for line in process.stdout.splitlines() if line.startswith(b'{')]
    valid = [row for row in rows if row.get('detector_valid') and row.get('final_pixel')]
    jumps = [dict(first=a['frame_id'], second=b['frame_id'],
                  distance_px=math.dist(a['final_pixel'], b['final_pixel']))
             for a,b in zip(valid,valid[1:])]
    maximum = max((row['distance_px'] for row in jumps), default=None)
    passed = len(valid) == len(inputs) and len(valid) >= 2 and maximum is not None and maximum <= 15
    report = dict(passed=passed, scope='Captured near-stationary tack onset; 15px local continuity diagnostic, not general motion gate or independent weld accuracy.',
        frames=sorted(frames), valid_frames=len(valid), jumps=jumps,
        missing_simulation_polyline=True, historical_tracker_state_restored=False,
        field_acceptance=False, provenance=provenance)
    (output/'report.json').write_text(json.dumps(report,indent=2), encoding='utf-8')
    print(json.dumps(dict(passed=passed, jumps=jumps, evidence=str(output)),indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
