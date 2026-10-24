"""Actual OpenCV pixel observation for the optional offline mother-material policy."""
import base64
import hashlib
import json
import pathlib
import shlex
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def observe(raw, proposal, history, options, candidate=None):
    """Use actual source PNG bytes emitted by process, never manifest measurements."""
    encoded = raw.get('image_png_base64')
    if not encoded or proposal is None:
        return dict(support_rank=0, center_supported=False, learned_model={},
                    estimated_pixel=None, reasons=['image_or_proposal_unavailable'])
    if sys.platform != 'darwin':
        raise RuntimeError('auxiliary mother observer uses macOS clang; production validation is VS2022')
    source = HERE/'MotherObservation.cpp'
    flags = shlex.split(subprocess.check_output(
        ['pkg-config','--cflags','--libs-only-L','opencv4','Qt6Core'],text=True))
    flags += shlex.split(subprocess.check_output(['pkg-config','--libs','Qt6Core'],text=True))
    flags += ['-lopencv_core','-lopencv_imgproc','-lopencv_imgcodecs']
    versions = subprocess.check_output(['pkg-config','--modversion','opencv4','Qt6Core'],text=True).splitlines()
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    key = hashlib.sha256(json.dumps([digest,flags,versions]).encode()).hexdigest()[:16]
    cache = ROOT/'output/analysis/mother_observation_build'
    cache.mkdir(parents=True,exist_ok=True)
    binary = cache/('MotherObservation-'+key)
    if not binary.exists():
        subprocess.run(['clang++','-std=c++17','-O1',str(source),*flags,'-o',str(binary)],check=True)
    request = dict(image_png_base64=encoded,proposal=proposal,history=history or {},
                   options=options,candidate_pixel=candidate)
    result = subprocess.run([str(binary)],input=json.dumps(request),text=True,capture_output=True)
    if result.returncode:
        raise RuntimeError(result.stderr)
    value = json.loads(result.stdout)
    value.update(image_png_sha256=hashlib.sha256(base64.b64decode(encoded)).hexdigest(),
                 helper_sha256=digest,wrapper_sha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),
                 dependency_versions=versions,history_used=history,
                 scope='pixel support only; inferred target offset is a historical assumption')
    return value


def render_viewer(rows, directory):
    """Standalone pixel support overlays; keep original detector records unchanged."""
    frames = []
    for row in rows:
        image = row.get('display_image')
        frames.append(dict(row=row,image=('data:image/png;base64,'+base64.b64encode(
            (directory/image).read_bytes()).decode()) if image else None))
    payload = json.dumps(frames,ensure_ascii=False).replace('<','\\u003c')
    return '''<!doctype html><meta charset="utf-8"><title>母材像素证据</title>
<style>body{background:#15232b;color:#eef;font:15px system-ui;margin:24px}svg{width:67%;background:black;vertical-align:top}pre{display:inline-block;width:31%;white-space:pre-wrap;font-size:11px;max-height:80vh;overflow:auto}input{width:80%}</style>
<h1>母材部分约束：实际像素与历史假设</h1><p>青/紫：实际中心外支撑像素与拟合；绿色：业务 KCF 点；橙色：生产原始几何点；白色十字：预测提案。外推仅是估计，单侧切向仍预测，时间/行程预算不续期。</p>
<input id="slider" type="range" min="0" value="0"><span id="count"></span><br><svg id="canvas"></svg><pre id="detail"></pre>
<script>const frames='''+payload+''';const slider=document.getElementById('slider'),svg=document.getElementById('canvas');slider.max=frames.length-1;
function element(name,attrs){const e=document.createElementNS('http://www.w3.org/2000/svg',name);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,v);svg.append(e);return e;}
function draw(){const f=frames[+slider.value],r=f.row,o=r.mother_observation||{};svg.innerHTML='';svg.setAttribute('viewBox','0 0 '+(r.image_size||[640,480]).join(' '));if(f.image)element('image',{href:f.image,width:(r.image_size||[640,480])[0],height:(r.image_size||[640,480])[1]});
for(const [key,color]of [['left','cyan'],['right','violet']]){const s=o[key]||{};for(const p of(s.points||[]))element('circle',{cx:p[0],cy:p[1],r:1.3,fill:color});if(s.valid&&s.points.length){const x1=s.points[0][0],x2=s.points.at(-1)[0];element('line',{x1,y1:s.slope*x1+s.intercept,x2,y2:s.slope*x2+s.intercept,stroke:color,'stroke-width':1.4});}}
for(const[key,color]of [['raw_measurement_pixel','orange'],['final_pixel','lime']]){const p=r[key];if(p)element('circle',{cx:p[0],cy:p[1],r:6,stroke:color,fill:'none','stroke-width':2});}const p=o.proposal;if(p){element('path',{d:`M${p[0]-7},${p[1]}h14M${p[0]},${p[1]-7}v14`,stroke:'white'});}
document.getElementById('count').textContent=(+slider.value+1)+' / '+frames.length;document.getElementById('detail').textContent=JSON.stringify(r,(k,v)=>k==='image_png_base64'?'[PNG shown in image]':v,2);}slider.oninput=draw;draw();</script>'''
