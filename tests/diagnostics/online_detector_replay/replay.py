"""Auxiliary macOS replay of the real detector image branch; no live devices."""
import argparse
import base64
import hashlib
import json
import math
import pathlib
import shlex
import subprocess
import sys
import tempfile

from native_runtime import configured_native, runtime_provenance

ROOT = pathlib.Path(__file__).resolve().parents[3]
HERE = pathlib.Path(__file__).resolve().parent
SOURCES = ['track/LegacyOnlineKcfDetector', 'track/KcfTracker', 'track/KcfFhog',
           'track/KcfFftTools', 'track/KcfRectTools', 'track/FilterKalman',
           'track/OnlineSeamContinuity', 'track/OnlineSeamEvidence', 'feature/OnlineSeamRefiner',
           'feature/SimulationGuidedSeamEstimator', 'geometry/WeldImageToSpaceProjector',
           'preprocess/ImageDenoise', 'stripe/DpStripeCenterlineDetector',
           'stripe/SubpixelCenterlineTools']


def build(cache):
    native = configured_native('DetectorReplay')
    if native:
        binary, provenance = native
        return binary, provenance['source_sha256'], json.dumps(provenance['dependency_versions'])
    if sys.platform != 'darwin':
        raise RuntimeError('Auxiliary clang runner requires macOS; production validation uses VS2022')
    sources = [ROOT / 'Vision/src' / (name + '.cpp') for name in SOURCES]
    sources += [ROOT / 'Vision/third_party/libicp/matrix.cpp', HERE / 'DetectorReplay.cpp',
                HERE / 'GapDisabledAdapter.cpp']
    includes = [ROOT / 'Vision/include', ROOT / 'Vision/third_party', ROOT / 'DigitalTwin/include']
    flags = shlex.split(subprocess.check_output(
        ['pkg-config', '--cflags', '--libs-only-L', 'opencv4', 'Qt6Core', 'eigen3'], text=True,encoding="utf-8"))
    flags += shlex.split(subprocess.check_output(['pkg-config', '--libs', 'Qt6Core'], text=True,encoding="utf-8"))
    flags += ['-lopencv_imgcodecs', '-lopencv_imgproc', '-lopencv_core']
    files = sources + [path for directory in includes for path in directory.rglob('*.h')]
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in files}
    versions = subprocess.check_output(['pkg-config', '--modversion', 'opencv4', 'Qt6Core', 'eigen3'], text=True,encoding="utf-8")
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
    def diagnostic_preview(value, key=''):
        # 仅构造HTML展示副本；原始results及原图字节完全保留。
        if isinstance(value, dict):
            return {name: diagnostic_preview(item, name) for name, item in value.items()}
        if isinstance(value, list):
            return [diagnostic_preview(item) for item in value]
        if isinstance(value, str):
            if 'base64' in key.lower() or value.startswith('data:image/'):
                return '[图像编码已折叠，' + str(len(value)) + '字符；完整证据见results.jsonl]'
            if len(value) > 1200:
                return value[:1200] + '[展示已截短；完整证据见results.jsonl]'
        return value

    rendered = []
    for row in rows:
        data = None
        if row.get('display_image'):
            data = 'data:image/png;base64,' + base64.b64encode(
                (manifest / row['display_image']).read_bytes()).decode()
        rendered.append({'row': diagnostic_preview(row), 'image': data})
    payload = json.dumps(rendered, ensure_ascii=False).replace('<', '\\u003c')
    return '''<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>真实检测入口离线回放</title>
<style>*{box-sizing:border-box}body{font:16px system-ui;margin:clamp(12px,3vw,24px);background:#18212b;color:#eef;overflow-wrap:anywhere}h1{font-size:clamp(22px,3vw,30px)}a{color:#9bdbff}.controls{display:flex;align-items:center;gap:12px;margin:16px 0}.controls input{flex:1;min-width:0}.layout{display:grid;grid-template-columns:minmax(0,2fr) minmax(260px,1fr);gap:20px;align-items:start}.layout>*{min-width:0}svg{display:block;width:100%;height:auto;background:#050505}.status{background:#243240;padding:12px;border-radius:6px;margin-bottom:12px;line-height:1.6}.status strong{display:block}details{border:1px solid #526270;border-radius:6px;padding:10px}summary{cursor:pointer}pre{margin-bottom:0;white-space:pre-wrap;overflow-wrap:anywhere;word-break:break-word;font-size:12px;max-height:65vh;overflow:auto}@media(max-width:850px){.layout{grid-template-columns:minmax(0,1fr)}pre{max-height:50vh}}</style>
<h1>真实检测入口：图像分支离线回放</h1><p>真实 process + KCF + 几何 + 投影；排除 PCL 间隙及设备控制。</p>
<p>绿色：已接纳独立几何像素；黄色：估计或跟踪像素；橙色：原始几何候选。颜色仅说明像素来源，不授予空间、独立焊点校验或控制消费资格。最终空间、独立焊点校验和控制消费资格以组合 report.json 为准；单独使用本查看器且无组合报告时，最终资格未知，不授予资格。</p>
<div class="controls"><input id="slider" aria-label="回放记录" type="range" min="0" value="0"><span id="index"></span></div>
<div class="layout"><svg id="canvas" aria-label="当前图像及KCF位置" viewBox="0 0 640 480"></svg><section><div class="status" id="status" aria-live="polite"></div><details><summary>展开当前诊断（图像编码已折叠）</summary><pre id="detail"></pre></details><p><a href="results.jsonl">完整逐帧原始证据 JSONL</a></p></section></div>
<script>const frames=''' + payload + ''';
const sourceLabels={geometry:'原图几何',mother_material_estimate:'母材估计',bounded_prediction:'有限预测',appearance_tracking:'外观续跟',recovery_transition:'恢复过渡',unavailable:'不可用'};
const slider=document.getElementById('slider'),canvas=document.getElementById('canvas');slider.max=Math.max(0,frames.length-1);
function draw(){const f=frames[+slider.value];if(!f)return;document.getElementById('index').textContent=(+slider.value+1)+' / '+frames.length;const r=f.row;canvas.innerHTML='';
const independent=r.available===true&&r.source==='geometry'&&r.trust==='independent_geometry';
const status=document.getElementById('status');status.replaceChildren();const title=document.createElement('strong');title.textContent='来源：'+(sourceLabels[r.source]||r.source||'未知')+'（'+(r.source||'unknown')+'）';status.append(title);
for(const text of ['像素可用：'+(r.available===true?'是':'否')+'；检测层独立几何声明：'+(independent?'是':'否'),'检测层独立校验声明：'+(r.can_independent_validate===true?'是':'否')+'；数值投影可用：'+(r.space_available===true?'是':'否'),'检测层控制资格声明：'+(r.control_eligible===true?'是':'否')+'；本回放不发送运动']){const line=document.createElement('div');line.textContent=text;status.append(line);}
if(f.image){const image=document.createElementNS('http://www.w3.org/2000/svg','image');image.setAttribute('href',f.image);image.setAttribute('width',r.image_size[0]);image.setAttribute('height',r.image_size[1]);canvas.append(image);canvas.setAttribute('viewBox','0 0 '+r.image_size.join(' '));}
for(const [key,color] of [['raw_measurement_pixel','orange'],['final_pixel',independent?'lime':'#ffe066']]){const p=r[key];if(!p)continue;const c=document.createElementNS('http://www.w3.org/2000/svg','circle');c.setAttribute('cx',p[0]);c.setAttribute('cy',p[1]);c.setAttribute('r',6);c.setAttribute('fill','none');c.setAttribute('stroke',color);c.setAttribute('stroke-width',2);canvas.append(c);}
document.getElementById('detail').textContent=JSON.stringify(r,null,2);}slider.oninput=draw;draw();</script>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=pathlib.Path)
    parser.add_argument('--evidence', action='store_true', help='apply offline evidence contract; not production control')
    parser.add_argument('--production', action='store_true', help='run the same C++ evidence policy as the main application')
    parser.add_argument('--policy-config', type=pathlib.Path)
    parser.add_argument('--output', type=pathlib.Path, required=True)
    parser.add_argument('--cache', type=pathlib.Path, default=ROOT/'output/analysis/detector_replay_build')
    args = parser.parse_args()
    if args.output.exists():
        raise RuntimeError('output exists; choose a new directory to preserve previous evidence')
    binary, hashes, versions = build(args.cache)
    inputs = [json.loads(line) for line in args.manifest.read_text(encoding="utf-8").splitlines() if line.strip()]
    if args.production and args.evidence:
        raise ValueError('--production and the historical Python --evidence comparator are mutually exclusive')
    if args.policy_config and not (args.evidence or args.production):
        raise ValueError('--policy-config requires --evidence or --production')
    if args.production:
        config = json.loads(args.policy_config.read_text(encoding='utf-8')) if args.policy_config else {}
        inputs = [dict(frame, evidence_policy=True, evidence_options=config) for frame in inputs]
    def detector(frames, path):
        path.write_text(''.join(json.dumps(frame)+'\n' for frame in frames),encoding='utf-8')
        process=subprocess.run([str(binary),str(path)],text=True,encoding="utf-8",capture_output=True)
        if process.returncode:
            raise RuntimeError(process.stderr)
        return [json.loads(line) for line in process.stdout.splitlines()]
    if args.evidence:
        from evidence_session import EvidenceSession
        config = json.loads(args.policy_config.read_text(encoding="utf-8")) if args.policy_config else None
        session = EvidenceSession(config)
        args.cache.mkdir(parents=True, exist_ok=True)
        transaction_dir=pathlib.Path(tempfile.mkdtemp(prefix='evidence-transactions-',dir=args.cache))
        checkpoint=None
        rows=[]
        for index, original in enumerate(inputs):
            current=dict(original)
            if original.get('image'):
                current['image']=str((args.manifest.parent/original['image']).resolve())
            frames=([checkpoint] if checkpoint else [])+[current]
            transaction=detector(frames,transaction_dir/(str(index)+'.jsonl'))
            raw=transaction[-1]
            reconstruction=None
            if checkpoint:
                seed=transaction[0]
                point=seed.get('final_pixel')
                expected=session.anchor['pixel'] if session.anchor else None
                valid=bool(seed.get('available') and seed.get('trust')=='independent_geometry'
                    and point and expected and math.dist(point,expected)<=4)
                reconstruction=dict(valid=valid, expected_pixel=expected, actual_pixel=point,
                    training_committed=seed.get('training_committed'),
                    template_reinitialized=seed.get('template_reinitialized'))
            raw['checkpoint_reconstruction']=reconstruction
            raw['image']=original.get('image')
            row=session.process(original,raw)
            row['checkpoint_source_frame']=checkpoint.get('frame_id') if checkpoint else None
            row['template_retained_for_next_exposure']=row['anchor_refreshed']
            row['detector_training_policy']='temporary real detector rebuilt from reliable checkpoint; only final independent geometry promotes checkpoint'
            # Both an update and a rebuilt template mutate the temporary detector.
            row['temporary_training_discarded']=bool((raw.get('training_committed')
                or raw.get('template_reinitialized')) and not row['anchor_refreshed'])
            if row['anchor_refreshed']:
                checkpoint=current
            rows.append(row)
    else:
        if args.production:
            # 使用绝对原图路径运输清单；不改变曝光身份，不重复重建检测器或模板。
            args.cache.mkdir(parents=True, exist_ok=True)
            folder = pathlib.Path(tempfile.mkdtemp(prefix='production-session-', dir=args.cache))
            frames = [dict(frame, image=str((args.manifest.parent/frame['image']).resolve()))
                      if frame.get('image') else frame for frame in inputs]
            rows = detector(frames, folder/'manifest.jsonl')
        else:
            process=subprocess.run([str(binary),str(args.manifest.resolve())],text=True,encoding="utf-8",capture_output=True)
            if process.returncode:
                raise RuntimeError(process.stderr)
            rows=[json.loads(line) for line in process.stdout.splitlines()]
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
    (args.output/'results.jsonl').write_text(''.join(json.dumps(row, ensure_ascii=False) + '\n' for row in rows), encoding='utf-8')
    (args.output/'viewer.html').write_text(viewer(rows, args.output), encoding='utf-8')
    metadata = {'scope': 'real CLegacyOnlineKcfDetector::process image branch; PCL gap excluded; no runtime adapter or control',
                'source_revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True,encoding="utf-8").strip(),
                'source_hashes': hashes, 'dependency_versions': versions.splitlines(),
                'native_runtime': runtime_provenance(),
                'manifest_sha256': hashlib.sha256(args.manifest.read_bytes()).hexdigest(),
                'synthetic_or_real': [row.get('kind') for row in rows],
                'missing_metadata': 'null; detector receives frame=-1/reference=0 defaults without claiming archive identity',
                'baseline_revision': '430aaa204fd5319dd7f697d3de2b4e1faddbb06f',
                'baseline_production_diff': subprocess.check_output(
                    ['git', 'diff', '--name-only', '430aaa204fd5319dd7f697d3de2b4e1faddbb06f', '--',
                     'Vision', 'DigitalTwin/include'], cwd=ROOT, text=True,encoding="utf-8").splitlines(),
                'msvc_validated': False, 'evidence_policy_enabled': args.evidence,
                'production_cpp_evidence_enabled': args.production,
                'evidence_transport': 'per-exposure reliable-checkpoint rebuild' if args.evidence else 'single detector full manifest',
                'policy_source_hashes': {name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in
                    ('replay.py','evidence_session.py','identity_evidence.py','IdentityEvidence.cpp')} if args.evidence else None,
                'identity_source_sha256': hashlib.sha256((HERE/'IdentityEvidence.cpp').read_bytes()).hexdigest() if args.evidence else None,
                'evidence_policy_sha256': hashlib.sha256((HERE/'evidence_session.py').read_bytes()).hexdigest() if args.evidence else None,
                'policy_config': session.config if args.evidence else None, 'policy_parameters_field_validated': False}
    (args.output/'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'frames': len(rows), 'available': sum(row['available'] for row in rows), 'output': str(args.output)}))

if __name__ == '__main__':
    main()
