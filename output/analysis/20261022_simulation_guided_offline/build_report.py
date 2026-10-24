"""Build a standalone desktop evidence report without altering source images."""
from pathlib import Path
import base64
import csv
import html
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
SESSION = ROOT / 'output/online_correction/20261021_235225_324'


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def main():
    baseline = read_json(OUT / 'baseline.json')
    conclusions = read_json(OUT / 'conclusions.json')
    frames = []
    selected = {'3400', '5661', '7891', '15216', '17035', '17861', '18854'}
    with (SESSION / 'vision_diagnostics.tsv').open(encoding='utf-8-sig') as stream:
        for row in csv.DictReader(stream, delimiter='\t'):
            if row['frame_id'] not in selected or not row['source_image']:
                continue
            image = SESSION / row['source_image']
            frame = {key: row[key] for key in [
                'frame_id', 'camera_timestamp_ms', 'image_width_px', 'image_height_px',
                'geometry_valid', 'geometry_status', 'geometry_ambiguous_column_ratio',
                'point_source', 'degraded_frames', 'before_fusion_x_px', 'before_fusion_y_px',
                'final_x_px', 'final_y_px', 'sim_x_px', 'sim_y_px', 'geometry_x_px', 'geometry_y_px']}
            frame['image'] = 'data:image/png;base64,' + base64.b64encode(image.read_bytes()).decode('ascii')
            frames.append(frame)
    checks = read_json(OUT / 'existing_executable_checks.json')
    adversarial = read_json(OUT / 'review/vision_adversarial_red.json')
    synthetic_path = OUT / 'vision/experiment_v4_review/weaker_model_like_reflection_with_real_shape_mismatch.png'
    synthetic_uri = 'data:image/png;base64,' + base64.b64encode(synthetic_path.read_bytes()).decode('ascii')
    truth_x, truth_y = adversarial['actual_seam_apex_px']
    found_x, found_y = adversarial['result']['point']
    adversarial_section = (
        '<section><h2>保留的合成反例：反光更符合模型形状时，原型仍会选错</h2>'
        '<p class="muted">独立生成的合成图，不是现场照片。真实条纹峰值215，反光180；反光较弱，但其形状更符合名义模型。</p>'
        '<svg viewBox="0 0 281 241" style="max-width:560px;max-height:490px">'
        '<image href="' + synthetic_uri + '" width="281" height="241"/>'
        f'<circle cx="{truth_x}" cy="{truth_y}" r="5" fill="none" stroke="#54e976" stroke-width="1.8"/>'
        f'<circle cx="{found_x}" cy="{found_y}" r="5" fill="none" stroke="#ff4d57" stroke-width="1.8"/>'
        f'<path d="M {truth_x} {truth_y} L {found_x} {found_y}" stroke="#ffd861" stroke-width="1" stroke-dasharray="3 2"/>'
        '</svg><p>绿圈：生成器定义的真实接缝 (138,134)；红圈：原型错误输出 (120,160)。误差31.623像素，左右与中心覆盖均为100%。</p>'
        '<p class="muted">该失败保持未解决。形状匹配分数、覆盖率和模型距离均不能单独证明真实条纹身份。</p></section>'
    )
    sections = []
    for section in conclusions['sections']:
        items = ''.join('<li>' + html.escape(item) + '</li>' for item in section['items'])
        sections.append('<section><h2>' + html.escape(section['title']) + '</h2><ul>' + items + '</ul></section>')
    links = []
    for name, folder in [('视觉候选实验', 'vision'), ('表面定位实验', 'surface'), ('独立审查', 'review')]:
        paths = sorted((OUT / folder).rglob('*')) if (OUT / folder).exists() else []
        paths = [p for p in paths if p.is_file() and p.suffix.lower() in ['.json', '.csv', '.py', '.html', '.txt']]
        entries = ''.join('<li><a href="' + html.escape(path.relative_to(OUT).as_posix()) + '">' + html.escape(path.name) + '</a></li>' for path in paths)
        links.append('<details><summary>' + name + '</summary><ul>' + entries + '</ul></details>')
    document = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>在线纠偏 · 仿真辅助接缝离线验证</title><style>
:root{color-scheme:light;font-family:"Microsoft YaHei",sans-serif;color:#172b3b;background:#edf2f6}
*{box-sizing:border-box}body{margin:0}main{max-width:1280px;margin:auto;padding:30px}
header{padding:30px;background:#16364a;border-radius:16px;color:white}h1{font-size:28px;margin:0 0 15px;line-height:1.4}
header p{color:#d6e7ee;line-height:1.8;margin:8px 0}.tag{display:inline-block;padding:5px 12px;border:1px solid #6f9aad;border-radius:20px;font-size:13px}
section{background:white;padding:24px;border-radius:13px;margin-top:20px;border:1px solid #dce4ea}h2{font-size:20px;margin:0 0 16px}p,li{line-height:1.9}li{margin:7px 0}ul{padding-left:24px}
.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin-top:20px}.card{background:#fff;border:1px solid #dce4ea;border-radius:12px;padding:18px}.card b{display:block;font-size:26px;color:#165e78}.card span{font-size:13px;color:#526676}
.controls{display:flex;gap:20px;align-items:center;flex-wrap:wrap;margin-bottom:15px}select{padding:8px;font-size:15px}label{font-size:14px;white-space:nowrap}
svg{display:block;width:100%;max-height:700px;background:#101518;border-radius:9px}#frame-meta{font-size:14px;background:#f1f5f7;padding:12px;margin:13px 0 0;border-radius:7px;font-variant-numeric:tabular-nums}
.muted{font-size:14px;color:#536775}.warning{border-left:4px solid #c9902f;padding-left:15px}.decision{font-size:18px;font-weight:bold;color:#12627d}
details{margin:10px 0}summary{cursor:pointer;padding:9px;background:#f1f5f7;border-radius:6px}a{color:#116184}footer{margin:20px 0;color:#526676;font-size:13px}table{width:100%;border-collapse:collapse;font-size:14px}td,th{text-align:left;padding:11px;border-bottom:1px solid #dde5ea}
</style></head><body><main>
<header><span class="tag">2026-10-22 · 第一阶段：离线验证</span><h1>仿真辅助接缝识别与表面定位解耦</h1>
<p>目标：经过预焊点时沿原接缝连续走向跨过凸起。仿真负责提供结构参考，真实图像与表面负责约束修正。</p>
<p>本阶段仅新增离线脚本和证据；未修改生产实现，未重新构建，未启动相机、RobotSim 或机器人。</p></header>
<div class="cards"><div class="card"><b>7056</b><span>本批有效在线输出</span></div><div class="card"><b>41.1%</b><span>包含辅助估计或纯预测</span></div><div class="card"><b>28</b><span>稀疏保存原图</span></div><div class="card"><b>52</b><span>冻结局部定位候选</span></div></div>
<section><h2>主 Agent 汇总结论</h2><p class="decision">__VERDICT__</p><p>__SUMMARY__</p><p class="muted warning">__LIMIT__</p></section>
<section><h2>原始图像与当前输出：来源必须区分</h2><p class="muted">以下显示原始保存图像，标记来自原始日志，不是新原型检测结果，也不是真值标注。所有坐标均为原图像素。</p>
<div class="controls"><label>保存帧 <select id="frame-select"></select></label><label><input type="checkbox" data-layer="sim" checked> 红：仿真点</label><label><input type="checkbox" data-layer="coarse" checked> 橙：粗 KCF</label><label><input type="checkbox" data-layer="final" checked> 绿：最终输出</label><label><input type="checkbox" data-layer="geometry" checked> 蓝：原图几何点</label></div>
<svg id="frame-svg" xmlns="http://www.w3.org/2000/svg"><image id="source-image" x="0" y="0"/><g id="overlays"></g></svg><div id="frame-meta"></div></section>
__ADVERSARIAL__
__SECTIONS__
<section><h2>验证记录与复现入口</h2><table><thead><tr><th>既有可执行测试</th><th>结果</th><th>本阶段重编译</th></tr></thead><tbody>__CHECKS__</tbody></table>
<p class="muted">既有测试通过只说明原有行为仍可复现，不证明新原型的生产效果。各实验使用的数据、判据和限制见分项文件。</p>
<p><a href="collect_baseline.py">基线采集脚本</a> · <a href="baseline.json">基线数据</a> · <a href="incident_input_hashes.json">输入 SHA-256</a> · <a href="conclusions.json">主 Agent 结论</a> · <a href="verification.json">最终核查</a></p>__LINKS__</section>
<footer>所有运行记录时间为 Asia/Shanghai。合成真值、冻结现场诊断、人工定性观察和实际闭环效果分别记录。</footer>
</main><script>
const frames=__FRAMES__;
const pristine=JSON.stringify(frames.map(({image,...r})=>r));
const ns='http://www.w3.org/2000/svg';
const selector=document.getElementById('frame-select');
for(const frame of frames){const o=document.createElement('option');o.value=frame.frame_id;o.textContent='第 '+frame.frame_id+' 帧';selector.append(o)}
selector.value=frames.some(r=>r.frame_id==='17861')?'17861':frames[0].frame_id;
const colors={sim:'#ff4d57',coarse:'#ffad39',final:'#54e976',geometry:'#59bcff'};
function render(){
 const row=frames.find(r=>r.frame_id===selector.value);const svg=document.getElementById('frame-svg');
 svg.setAttribute('viewBox',`0 0 ${row.image_width_px} ${row.image_height_px}`);
 const image=document.getElementById('source-image');image.setAttribute('href',row.image);image.setAttribute('width',row.image_width_px);image.setAttribute('height',row.image_height_px);
 const layer=document.getElementById('overlays');layer.replaceChildren();
 const xy={sim:[row.sim_x_px,row.sim_y_px],coarse:[row.before_fusion_x_px,row.before_fusion_y_px],final:[row.final_x_px,row.final_y_px],geometry:row.geometry_valid==='1'?[row.geometry_x_px,row.geometry_y_px]:null};
 for(const input of document.querySelectorAll('[data-layer]')){const key=input.dataset.layer,p=xy[key];if(!input.checked||!p||p.some(v=>v===''))continue;
  const g=document.createElementNS(ns,'g');g.dataset.layer=key;g.setAttribute('stroke',colors[key]);g.setAttribute('stroke-width','2.5');
  const x=Number(p[0]),y=Number(p[1]);const circle=document.createElementNS(ns,'circle');circle.setAttribute('cx',x);circle.setAttribute('cy',y);circle.setAttribute('r','8');circle.setAttribute('fill','none');g.append(circle);
  const path=document.createElementNS(ns,'path');path.setAttribute('d',`M ${x-15} ${y} h 30 M ${x} ${y-15} v 30`);g.append(path);layer.append(g);
 }
 const source={1:'原图几何及融合',2:'KCF 辅助估计',3:'仿真辅助预测'}[row.point_source]||'不可用';
 document.getElementById('frame-meta').textContent=`帧 ${row.frame_id} ｜ 最终输出来源：${source} ｜ 几何有效：${row.geometry_valid==='1'?'是':'否'} ｜ 竞争条纹比例：${(Number(row.geometry_ambiguous_column_ratio)*100).toFixed(2)}% ｜ 连续几何退化：${row.degraded_frames} 帧`;
}
selector.addEventListener('change',render);for(const input of document.querySelectorAll('[data-layer]'))input.addEventListener('change',render);render();
window.reportReady=true;window.reportDataSnapshot=()=>JSON.stringify(frames.map(({image,...r})=>r));window.reportPristine=pristine;
</script></body></html>'''
    replacements = {
        '__VERDICT__': html.escape(conclusions['verdict']),
        '__SUMMARY__': html.escape(conclusions['summary']),
        '__LIMIT__': html.escape(conclusions['limitation']),
        '__SECTIONS__': ''.join(sections),
        '__ADVERSARIAL__': adversarial_section,
        '__CHECKS__': ''.join('<tr><td>' + html.escape(check['name']) + '</td><td>' + html.escape(check['status']) + '</td><td>否</td></tr>' for check in checks),
        '__LINKS__': ''.join(links),
        '__FRAMES__': json.dumps(frames, ensure_ascii=False).replace('<', '\\u003c'),
    }
    for key, value in replacements.items():
        document = document.replace(key, value)
    (OUT / 'report.html').write_text(document, encoding='utf-8')
    print(json.dumps({'report': str(OUT / 'report.html'), 'frames': [f['frame_id'] for f in frames],
                      'baseline_session': baseline['session']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
