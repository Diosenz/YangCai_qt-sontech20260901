"""Auxiliary macOS replay of the real detector image branch; no live devices."""
import argparse
import base64
import hashlib
import json
import pathlib
import shlex
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]
HERE = pathlib.Path(__file__).resolve().parent
SOURCES = ['track/LegacyOnlineKcfDetector', 'track/KcfTracker', 'track/KcfFhog',
           'track/KcfFftTools', 'track/KcfRectTools', 'track/FilterKalman',
           'track/OnlineSeamContinuity', 'feature/OnlineSeamRefiner',
           'feature/SimulationGuidedSeamEstimator', 'geometry/WeldImageToSpaceProjector',
           'preprocess/ImageDenoise', 'stripe/DpStripeCenterlineDetector',
           'stripe/SubpixelCenterlineTools']


def build(cache):
    if sys.platform != 'darwin':
        raise RuntimeError('Auxiliary clang runner requires macOS; production validation uses VS2022')
    sources = [ROOT / 'Vision/src' / (name + '.cpp') for name in SOURCES]
    sources += [ROOT / 'Vision/third_party/libicp/matrix.cpp', HERE / 'DetectorReplay.cpp',
                HERE / 'GapDisabledAdapter.cpp']
    includes = [ROOT / 'Vision/include', ROOT / 'Vision/third_party', ROOT / 'DigitalTwin/include']
    flags = shlex.split(subprocess.check_output(
        ['pkg-config', '--cflags', '--libs-only-L', 'opencv4', 'Qt6Core', 'eigen3'], text=True))
    flags += shlex.split(subprocess.check_output(['pkg-config', '--libs', 'Qt6Core'], text=True))
    flags += ['-lopencv_imgcodecs', '-lopencv_imgproc', '-lopencv_core']
    files = sources + [path for directory in includes for path in directory.rglob('*.h')]
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in files}
    versions = subprocess.check_output(['pkg-config', '--modversion', 'opencv4', 'Qt6Core', 'eigen3'], text=True)
    key = hashlib.sha256(json.dumps([hashes, flags, versions], sort_keys=True).encode()).hexdigest()[:16]
    cache.mkdir(parents=True, exist_ok=True)
    binary = cache / ('DetectorReplay-' + key)
    if not binary.exists():
        command = ['clang++', '-std=c++17', '-O1', *[str(path) for path in sources]]
        for directory in includes:
            command += ['-I', str(directory)]
        subprocess.run(command + flags + ['-o', str(binary)], check=True)
    return binary, hashes, versions


def viewer(rows, manifest):
    rendered = []
    for row in rows:
        data = None
        if row.get('display_image'):
            data = 'data:image/png;base64,' + base64.b64encode(
                (manifest / row['display_image']).read_bytes()).decode()
        rendered.append({'row': row, 'image': data})
    payload = json.dumps(rendered, ensure_ascii=False).replace('<', '\\u003c')
    return '''<!doctype html><meta charset="utf-8"><title>真实检测入口离线回放</title>
<style>body{font:16px system-ui;margin:24px;background:#18212b;color:#eef}svg{width:70%;background:#050505;vertical-align:top}pre{display:inline-block;width:28%;white-space:pre-wrap;font-size:12px}input{width:70%}</style>
<h1>真实检测入口：图像分支离线回放</h1><p>真实 process + KCF + 几何 + 投影；排除 PCL 间隙检测及设备控制。绿色：可用 KCF；橙色：原始几何候选。候选列表仅为公开诊断，不是全部候选。</p>
<input id="slider" type="range" min="0" value="0"><span id="index"></span><br>
<svg id="canvas" viewBox="0 0 640 480"></svg><pre id="detail"></pre>
<script>const frames=''' + payload + ''';
const slider=document.getElementById('slider'),canvas=document.getElementById('canvas');slider.max=Math.max(0,frames.length-1);
function draw(){const f=frames[+slider.value];if(!f)return;document.getElementById('index').textContent=(+slider.value+1)+' / '+frames.length;const r=f.row;canvas.innerHTML='';
if(f.image){const image=document.createElementNS('http://www.w3.org/2000/svg','image');image.setAttribute('href',f.image);image.setAttribute('width',r.image_size[0]);image.setAttribute('height',r.image_size[1]);canvas.append(image);canvas.setAttribute('viewBox','0 0 '+r.image_size.join(' '));}
for(const [key,color] of [['raw_measurement_pixel','orange'],['final_pixel','lime']]){const p=r[key];if(!p)continue;const c=document.createElementNS('http://www.w3.org/2000/svg','circle');c.setAttribute('cx',p[0]);c.setAttribute('cy',p[1]);c.setAttribute('r',6);c.setAttribute('fill','none');c.setAttribute('stroke',color);c.setAttribute('stroke-width',2);canvas.append(c);}
document.getElementById('detail').textContent=JSON.stringify(r,null,2);}slider.oninput=draw;draw();</script>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=pathlib.Path)
    parser.add_argument('--output', type=pathlib.Path, required=True)
    parser.add_argument('--cache', type=pathlib.Path, default=ROOT/'output/analysis/detector_replay_build')
    args = parser.parse_args()
    if args.output.exists():
        raise RuntimeError('output exists; choose a new directory to preserve previous evidence')
    binary, hashes, versions = build(args.cache)
    result = subprocess.run([str(binary), str(args.manifest.resolve())], text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError(result.stderr)
    rows = [json.loads(line) for line in result.stdout.splitlines()]
    inputs = [json.loads(line) for line in args.manifest.read_text().splitlines() if line.strip()]
    args.output.mkdir(parents=True)
    portable_inputs = []
    for index, (row, original) in enumerate(zip(rows, inputs)):
        row['image_sha256'] = hashlib.sha256((args.manifest.parent/original['image']).read_bytes()).hexdigest() if original.get('image') else None
        encoded = row.pop('image_png_base64', None)
        row['display_image'] = None
        if encoded:
            name = f'frame-{index:05d}.png'
            (args.output/name).write_bytes(base64.b64decode(encoded))
            row['display_image'] = name
        portable_inputs.append(dict(original, image=row['display_image']))
    (args.output/'manifest.jsonl').write_text(''.join(json.dumps(row, ensure_ascii=False) + '\n' for row in portable_inputs), encoding='utf-8')
    (args.output/'results.jsonl').write_text(''.join(json.dumps(row, ensure_ascii=False) + '\n' for row in rows))
    (args.output/'viewer.html').write_text(viewer(rows, args.output), encoding='utf-8')
    metadata = {'scope': 'real CLegacyOnlineKcfDetector::process image branch; PCL gap excluded; no runtime adapter or control',
                'source_revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'source_hashes': hashes, 'dependency_versions': versions.splitlines(),
                'manifest_sha256': hashlib.sha256(args.manifest.read_bytes()).hexdigest(),
                'synthetic_or_real': [row.get('kind') for row in rows],
                'missing_metadata': 'null; detector receives frame=-1/reference=0 defaults without claiming archive identity',
                'baseline_revision': '430aaa204fd5319dd7f697d3de2b4e1faddbb06f',
                'baseline_production_diff': subprocess.check_output(
                    ['git', 'diff', '--name-only', '430aaa204fd5319dd7f697d3de2b4e1faddbb06f', '--',
                     'Vision', 'DigitalTwin/include'], cwd=ROOT, text=True).splitlines(),
                'msvc_validated': False}
    (args.output/'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'frames': len(rows), 'available': sum(row['available'] for row in rows), 'output': str(args.output)}))

if __name__ == '__main__':
    main()
