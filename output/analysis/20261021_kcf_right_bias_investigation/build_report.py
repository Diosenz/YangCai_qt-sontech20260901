"""Build a self-contained visual audit with original images and SVG overlays."""
from pathlib import Path
import base64
import csv
import html
import json
import numpy as np
from analyze import OUT, ROOT, RUN, sha


def main():
    result = json.loads((OUT / "results.json").read_text(encoding="utf-8"))
    geometry = json.loads((OUT / "geometry_results.json").read_text(encoding="utf-8"))
    stats = result["statistics"]
    chosen = [6469, 7570, 9411, 10878, 12363, 14847, 16227]
    view = []
    for s in result["samples"]:
        if s["frame"] not in chosen:
            continue
        item = dict(s)
        item["images"] = {key: "data:image/png;base64," + base64.b64encode((RUN / s[key]).read_bytes()).decode() for key in ["source", "enhanced"]}
        view.append(item)
    table = []
    for s in result["samples"]:
        def n(k):
            return f"{s[k]:.2f}" if k in s and s[k] is not None else "—"
        table.append(f'<tr><td>{s["frame"]}</td><td>{"稳定" if s["break_eligible"] else "未纳入断点统计"}</td><td>{n("sim_x")}</td><td>{n("raw_x")}</td><td>{n("corner_x")}</td><td>{n("break_x")}</td><td>{n("break_jump_y")}</td><td>{n("raw_break_dx")}</td><td>{n("raw_break_endpoint_min_mm")}</td></tr>')
    stable = [s for s in result["samples"] if s["break_eligible"]]
    dots = []
    for s in stable:
        x = 55 + (s["frame"] - 3000) / 16000 * 850
        y = 155 - s["raw_break_dx"] * 5
        dots.append(f'<circle cx="{x}" cy="{y}" r="5" fill="#d88929"><title>帧{s["frame"]}: {s["raw_break_dx"]:.2f}px</title></circle>')
    chart = '<svg viewBox="0 0 960 240" role="img" aria-label="11个稳定断点样本的KCF水平差，正值向右">'
    for v in [-10, 0, 10, 20]:
        y = 155 - v * 5
        chart += f'<line x1="55" y1="{y}" x2="920" y2="{y}" stroke="#d7dfe5"/><text x="10" y="{y+5}" fill="#53636c">{v}px</text>'
    chart += ''.join(dots) + '<text x="55" y="225">帧3000</text><text x="835" y="225">帧19000</text></svg>'
    b = stats["sample_raw_to_visible_break_dx"]
    page = """<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>KCF 持续向右偏差 · 离线排查</title>
<style>body{font:16px/1.65 'Microsoft YaHei',sans-serif;color:#1d303a;background:#edf2f4;margin:0}main{max-width:1120px;margin:35px auto;padding:0 22px}h1{font-size:30px;line-height:1.3}h2{font-size:21px;margin-top:34px}.card{background:white;border-radius:12px;padding:25px;margin:20px 0;border:1px solid #dce4e8}.muted{color:#5b6c75;font-size:14px}.metrics{display:flex;gap:20px;flex-wrap:wrap}.metric{flex:1;min-width:160px}.metric b{font-size:29px;display:block;color:#185c70}.note{border-left:4px solid #d88929;padding-left:16px}button,select{font:inherit;padding:8px 13px;border:1px solid #bccbd2;border-radius:6px;background:white}label{margin-right:20px}#viewer{width:100%;background:#000;display:block;border-radius:8px;margin:18px 0}#readout{font-variant-numeric:tabular-nums}.legend span{margin-right:20px;white-space:nowrap}table{border-collapse:collapse;width:100%;font-size:13px}td,th{text-align:right;border-bottom:1px solid #dce4e8;padding:8px}th{background:#f1f5f7}th:first-child,td:first-child{text-align:left}.scroll{overflow:auto}code{font-family:Consolas,monospace}a{color:#19667b}li{margin:7px 0}</style>
<main><p class="muted">2026-10-21 · 20:14 复测 · 离线证据核查 · 未修改生产代码</p><h1>KCF 的持续向右偏差<br>已定位到二维检测输出，精确成因仍需连续回放</h1>
<div class="card"><div class="metrics"><div class="metric"><b>7776帧</b>正式在线阶段</div><div class="metric"><b>94.8%</b>原始KCF在仿真点右侧</div><div class="metric"><b>+21.64 px</b>KCF−仿真点中位数</div><div class="metric"><b>29对</b>原图与增强图</div></div></div>
<div class="card note"><b>本轮校正了上一轮的测量口径。</b><p>两侧条纹存在台阶/断口时，延长线交点不等于可见转折。上一轮的连续V形拟合会高估部分帧的“右偏”。本报告同时显示连续交点与允许台阶的断点；二者都是图像估计，均不等同于经过独立标定的焊缝真值。</p><p>允许台阶后，11个通过多窗口与残差检查的样本中，9个KCF仍在可见断点右侧，水平差中位数为 <b>+10.00 px</b>，范围−4.80～+17.92 px。稀疏且偏异常的样本不能代表整段真实精度分布。</p></div>
<div class="card"><h2>原始图像逐帧核查</h2><p class="muted">图像未经重采样保存；下面只通过SVG视窗放大显示。标记坐标与原图一致。转折处的青色短线连接两侧条纹端点估计。</p><label>代表帧 <select id="frame"></select></label><label>图像 <select id="layer"><option value="source">原图</option><option value="enhanced">增强图</option></select></label><label><input id="models" type="checkbox" checked>显示拟合模型</label><svg id="viewer" xmlns="http://www.w3.org/2000/svg"></svg><div class="legend"><span style="color:#f04d5e">● 仿真点</span><span style="color:#e9a04a">＋ 原始KCF</span><span style="color:#36a45d">＋ 最终输出</span><span style="color:#169fac">│ 可见台阶/转折</span><span style="color:#a76bdb">◇ 延长线交点</span></div><p id="readout"></p></div>
<div class="card"><h2>为什么不是同帧显示或三维投影造成的</h2><ul><li>绿色点直接显示检测输出，偏差在融合前已经存在。</li><li>8127条输出按当前标定和各帧机器人姿态重建，最大三维差小于0.000001 mm。这确认计算重建与归档一致，不代表标定的物理精度达到这个值。</li><li>5个同时有图像和ICP输入的帧，仿真三维焊点重投影与记录红点相差0.19～0.80 px，符合整数像素记录的尺度。</li><li>153个模板重建首帧中，96.1%的原始KCF在仿真点右侧；中位水平差+20.61 px。它不能由“仅长时间积累才出现的漂移”解释，但重建保留旧位置，不能据此排除历史影响。</li></ul></div>
<div class="card"><h2>当前根因范围</h2><p><b>确认的结构限制：</b>KCF匹配整个区域，输出是跟踪框中的固定模板锚点；没有在输出前用真实图像校核焊缝转折。局部纹理、条纹宽度、台阶、模板形状与搜索位置的变化，都可以使最佳区域匹配位置偏离可见转折。</p><p><b>支持但未完全定因：</b>独立FHOG参考实验中，同图匹配与理想同形匹配没有强制向右偏移；正负平移控制均跟随对应方向。替换为真实两侧角度的理想模板能显著改变某些帧的位置，因此模板/输入形状不一致值得优先检查。参考实验使用NumPy FHOG与已安装OpenCV图像原语，不等价于生产C++连续回放，不能据此宣布某一参数就是唯一根因。</p><p>Gabor增强会改变条纹外观，部分连续交点向右移动，但其量级通常小于生产KCF偏差；不能仅凭这些样本把责任全部归给滤波。固定减X、频繁重建或调大仿真权重都没有得到本轮证据支持。</p></div>
<div class="card"><h2>稳定断点样本：KCF−可见转折</h2>CHART<p class="muted">仅画离散观测，不连接成连续轨迹；正值表示向右。</p></div>
<div class="card"><h2>对在线纠偏与ICP的影响</h2><p>最终点会进入三维纠偏输入，原始KCF点会进入ICP独立焊点门槛和后验检查，因此偏差存在影响路径。11个稳定样本中，KCF投影到两侧可见端点中较近一个的距离中位数约0.81 mm；这只是观测差异，不能直接称为机器人轨迹误差。</p><p>本轮有3个连续交点估计可与ICP候选原始输入精确配对，但只有10878帧同时具有稳定的可见断点估计。该帧替换观测后，“不明显恶化”的单帧判断没有翻转，所在候选仍因原表面验收条件被拒。<b>本轮没有证实某次ICP实际发生了误拒绝或误放行。</b></p><p>后续应先确定真实焊点的几何语义（连续折点、两侧端点或间隙中心），再验证KCF附近的局部几何定位。稀疏图像无法准确复现每帧训练与融合反馈；精确归因需要连续片段、原始虚拟二维模板点及跟踪内部状态。</p></div>
<div class="card"><h2>全部29对图像：保留失败和反例</h2><div class="scroll"><table><thead><tr><th>帧</th><th>断点估计</th><th>仿真X</th><th>KCF X</th><th>连续交点X</th><th>断点X</th><th>台阶Y</th><th>KCF−断点</th><th>近端点mm</th></tr></thead><tbody>TABLE</tbody></table></div><p class="muted">稳定条件：85/100/115像素半径都可拟合，断点X跨度≤4像素、Huber平均损失&lt;2.5。条件仅筛选此分析可比较的图像，不是生产质量阈值。未纳入样本仍完整保留，原始诊断没有被清洗或修改。</p></div>
<div class="card"><h2>验证与复现</h2><ul><li>Windows现有 <code>i2r_rlc_vision_kcf_tracker_tests.exe</code>：通过。它的灰度平移断言容差为12 px，不能证明焊点精确定位。</li><li>旧penzui服务回放：退出码1；所需 <code>data/replay/penzui</code> 不存在，因此未取得有效回放结果。</li><li>无MSVC/CMake重新构建，无机器人或RobotSim写入，无生产代码修改。</li></ul><p><a href="results.json">全部测量与拟合</a> · <a href="samples.csv">CSV</a> · <a href="geometry_results.json">投影与候选核查</a> · <a href="reference_kcf_results.json">独立参考实验</a> · <a href="template_results.json">虚拟模板对照</a> · <a href="input_hashes.json">输入哈希</a></p><p class="muted">复现顺序：analyze.py → geometry_check.py → reference_kcf.py → template_check.py → build_report.py。使用已有Python、NumPy、Pillow以及本机OpenCV 3.4.5 DLL，未安装依赖。Notion工作流于本轮通过公开接口实际读取，快照见notion_workflow_snapshot.json。</p></div></main>
<script>const frames=DATA; const sel=document.getElementById('frame');for(const s of frames){const o=document.createElement('option');o.value=s.frame;o.textContent=s.frame+(s.break_eligible?' · 断点稳定':' · 参考样本');sel.appendChild(o)}sel.value='7570';const $=id=>document.getElementById(id);function draw(){const s=frames.find(x=>x.frame==sel.value),v=$('viewer'),cx=s.break_x??s.sim_x,cy=s.break_left_y??s.sim_y;v.setAttribute('viewBox',`${cx-115} ${cy-72} 230 144`);let out=`<image href="${s.images[$('layer').value]}" x="0" y="0" width="1292" height="964"/>`;const cross=(x,y,c,r=4)=>`<path d="M${x-r} ${y}H${x+r}M${x} ${y-r}V${y+r}" stroke="${c}" stroke-width=".8" fill="none"/>`;out+=`<circle cx="${s.sim_x}" cy="${s.sim_y}" r="3" fill="none" stroke="#ff4c65" stroke-width=".8"/>`;out+=cross(s.raw_x,s.raw_y,'#ffb354',5)+cross(s.final_x,s.final_y,'#42ff7a',3);if($('models').checked){const f=s.fits.source_image[1];if(f){out+=`<path d="M${f.x-90} ${f.y-90*f.left_slope} L${f.x} ${f.y} L${f.x+90} ${f.y+90*f.right_slope}" fill="none" stroke="#ba85ff" stroke-width=".5" stroke-dasharray="2 2"/>`;out+=`<path d="M${f.x} ${f.y-3}l3 3 -3 3 -3 -3Z" fill="none" stroke="#ba85ff" stroke-width=".6"/>`}const b=s.breaks.source_image[1];if(b){out+=`<path d="M${b.x-90} ${b.left_y-90*b.left_slope} L${b.x} ${b.left_y} M${b.x} ${b.right_y} L${b.x+90} ${b.right_y+90*b.right_slope}" fill="none" stroke="#43efff" stroke-width=".6"/>`;out+=`<path d="M${b.x} ${b.left_y-3}V${b.right_y+3}" stroke="#43efff" stroke-width="1.2"/>`}}v.innerHTML=out;$('readout').textContent=`帧 ${s.frame}｜原始KCF X=${s.raw_x.toFixed(2)}｜可见断点 X=${s.break_x?.toFixed(2)??'不可估计'}｜水平差=${s.raw_break_dx?.toFixed(2)??'—'} px｜${s.break_eligible?'通过断点稳定性检查':'未纳入断点统计'}。台阶两端均显示，不把延长线交点等同于焊缝真值。`}for(const id of ['frame','layer','models'])$(id).addEventListener('change',draw);draw();</script></html>"""
    page = page.replace("CHART", chart).replace("TABLE", ''.join(table)).replace("DATA", json.dumps(view, ensure_ascii=False).replace("</", "<\\/"))
    (OUT / "index.html").write_text(page, encoding="utf-8")
    # Refresh CSV with the second model without recomputing image fits.
    cols = ["frame", "phase", "break_eligible", "sim_x", "raw_x", "corner_x", "break_x", "break_jump_y", "raw_break_dx", "raw_break_endpoint_min_mm", "raw_break_endpoint_max_mm"]
    with (OUT / "samples.csv").open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(result["samples"])
    # Check source data content and all referenced local evidence before delivery.
    hashes = json.loads((OUT / "input_hashes.json").read_text(encoding="utf-8"))
    assert all(sha(ROOT / name) == digest for name, digest in hashes.items())
    assert len(view) == 7 and len(stable) == 11
    assert sum(s["raw_break_dx"] > 0 for s in stable) == 9
    verification = {"input_hashes_unchanged": len(hashes), "viewer_frames": [s["frame"] for s in view], "stable_breaks": len(stable), "production_code_modified": False, "native_kcf_test": "passed existing executable", "legacy_service_replay": "exit 1; required penzui fixture absent", "browser_visual_verification": "pending"}
    (OUT / "verification.json").write_text(json.dumps(verification, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(verification, ensure_ascii=False, indent=2))
    print("report_bytes", (OUT / "index.html").stat().st_size)


if __name__ == "__main__":
    main()
