"""同一冻结实拍输入对比显式 Debug/Release 构建；不授予现场频率资格。"""
import argparse
import hashlib
import json
import math
import os
import pathlib
import platform
import subprocess
import time

from native_runtime import configured_native, ROOT
from performance_report import statistics_ms


def differences(a, b, path=''):
    """离散行为必须相同；数值 abs=1e-5、rel=1e-7，不比较计时和 PNG 编码。"""
    if isinstance(a, bool) or isinstance(b, bool):
        return [] if a == b else [path]
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return [] if math.isclose(a, b, abs_tol=1e-5, rel_tol=1e-7) else [path]
    if isinstance(a, dict) and isinstance(b, dict):
        keys = (set(a) | set(b)) - {'performance', 'image_png_base64'}
        return [issue for key in sorted(keys) for issue in differences(a.get(key), b.get(key), path+'/'+key)]
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        return [issue for i, (x, y) in enumerate(zip(a, b)) for issue in differences(x, y, path+'/'+str(i))]
    return [] if a == b else [path]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--debug-dir', type=pathlib.Path, required=True)
    parser.add_argument('--release-dir', type=pathlib.Path, required=True)
    parser.add_argument('--output', type=pathlib.Path, required=True)
    parser.add_argument('--repeats', type=int, default=2)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    fixture = ROOT / 'tests/fixtures/paper_tracking_20261027'
    frames = [json.loads(line) for line in (fixture / 'online.jsonl').read_text(encoding='utf-8-sig').splitlines()]
    frames = [dict(frame, paper_tracking=True, image=str((fixture / frame['image']).resolve())) for frame in frames]
    manifests = {}
    for enabled in (False, True):
        manifest = args.output / f'input-{int(enabled)}.jsonl'
        manifest.write_text(''.join(json.dumps(dict(frame, performance_diagnostics=enabled))+'\n'
                                    for frame in frames), encoding='utf-8')
        manifests[enabled] = manifest
    report = dict(environment=dict(platform=platform.platform(), processor=platform.processor(),
                                    logical_processors=os.cpu_count(), python=platform.python_version()),
                  input_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in [fixture/'online.jsonl'] + [pathlib.Path(f['image']) for f in frames]},
                  tolerance=dict(absolute=1e-5, relative=1e-7), runs=[], mismatches=[],
                  limitations=['Sparse captured pixels; recorded poses include labelled borrowed transforms.',
                               'Fixture lacks continuous same-exposure simulated laser polylines.',
                               'Process wall time includes image reading, PNG encoding, IPC and startup.',
                               'No continuous camera rate, exposure latency or TCP accuracy acceptance.'])
    reference = None
    original_path = os.environ.get('PATH', '')
    for label, directory in [('Debug', args.debug_dir), ('Release', args.release_dir)]:
        os.environ['PATH'] = original_path
        os.environ['I2R_KCF_REPLAY_BIN_DIR'] = str(directory.resolve())
        binary, provenance = configured_native('DetectorReplay')
        for repeat in range(args.repeats):
            for enabled in (False, True):
                started = time.perf_counter()
                result = subprocess.run([str(binary), str(manifests[enabled].resolve())],
                                        capture_output=True, timeout=90)
                elapsed = (time.perf_counter()-started)*1000
                if result.returncode:
                    raise RuntimeError(result.stderr.decode('utf-8', errors='replace'))
                rows = [json.loads(line) for line in result.stdout.splitlines() if line.startswith(b'{')]
                for row in rows:
                    row.pop('image_png_base64', None)
                if reference is None:
                    reference = rows
                mismatches = differences(reference, rows)
                report['mismatches'].extend(dict(configuration=label, repeat=repeat,
                                                 enabled=enabled, path=p) for p in mismatches)
                name = f'{label.lower()}-{repeat}-{int(enabled)}.jsonl'
                (args.output/name).write_text(''.join(json.dumps(row)+'\n' for row in rows), encoding='utf-8')
                timings = [row['performance'] for row in rows if row.get('performance')]
                stable = {name: statistics_ms([t[name] for t in timings[1:]])
                          for name in timings[0]} if timings else None
                report['runs'].append(dict(configuration=label, repeat=repeat, enabled=enabled,
                    provenance=provenance, input_manifest_sha256=hashlib.sha256(manifests[enabled].read_bytes()).hexdigest(),
                    process_wall_ms=elapsed, frames=len(rows), valid=sum(r['detector_valid'] for r in rows),
                    template_counts=sorted(set(r['response_template_count'] for r in rows)),
                    cold=timings[0] if timings else None, stable=stable, results=name))
                print(f'{label} repeat={repeat} measured={enabled} wall={elapsed:.1f}ms '
                      f'frames={len(rows)} differences={len(mismatches)}', flush=True)
    (args.output/'comparison.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    if report['mismatches']:
        raise SystemExit('Output differs beyond stated tolerance; inspect comparison.json')


if __name__ == '__main__':
    main()
