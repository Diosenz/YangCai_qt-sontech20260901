"""Follow-up using the user's qualitative seam reference for frame 6469 only.

Does not turn a qualitative screenshot into a precise Y or physical ground truth.
All controls use an independent model; no production code or recordings change.
"""
import base64
import csv
import hashlib
import html
import json
import statistics
import numpy as np
from PIL import Image
from analyze import OUT, ROOT, RUN, camera_point
from native_cv import lines
from reference_kcf import fhog, patch, response


def main():
    inputs = json.loads((OUT / "input_hashes.json").read_text(encoding="utf-8"))
    assert all(hashlib.sha256((ROOT / n).read_bytes()).hexdigest() == h for n, h in inputs.items())
    prior = json.loads((OUT / "results.json").read_text(encoding="utf-8"))
    selected = [s for s in prior["samples"] if s["frame"] == 6469 or s["break_eligible"]]
    cal = {}
    for line in (ROOT / "config/calibration/SENSORPARA.txt").read_text(encoding="utf-8-sig").splitlines():
        bits = line.split()
        if len(bits) > 1:
            try:
                cal[bits[0]] = float(bits[1])
            except ValueError:
                pass
    decomposition = []
    controls = []
    for s in selected:
        b = s["breaks"]["source_image"][1]
        total = s["raw_x"] - s["sim_x"]
        reference_to_sim = b["x"] - s["sim_x"]
        raw_to_reference = s["raw_x"] - b["x"]
        assert abs(total - reference_to_sim - raw_to_reference) < 1e-9
        decomposition.append({"frame": s["frame"], "user_reference_available": s["frame"] == 6469, "stable_break": s["break_eligible"], "sim_x": s["sim_x"], "visible_break_x": b["x"], "raw_x": s["raw_x"], "total_raw_minus_sim_px": total, "visible_break_minus_sim_px": reference_to_sim, "raw_minus_visible_break_px": raw_to_reference, "raw_y": s["raw_y"], "visible_left_endpoint_y": b["left_y"], "visible_right_endpoint_y": b["right_y"], "peak": s["peak"], "since_init": s["since_init"]})
        # Both templates use the same slopes and nominal seam X, with the left
        # endpoint used only as a coordinate anchor. No midpoint assumption.
        sx, sy = s["sim_x"], s["sim_y"]
        xx = np.arange(1292)
        left, right = xx <= sx, xx >= sx
        row = {"frame": s["frame"], "input_status": "independent single-frame reset control, not native history replay", "visible_break_x": b["x"], "production_raw_x": s["raw_x"]}
        for layer in ["source", "enhanced"]:
            im = np.asarray(Image.open(RUN / s[layer]).convert("L"), dtype=np.float32)
            observed = fhog(patch(im, (sx, sy)))
            row[layer] = {}
            for kind, jump in [("continuous_v", 0), ("step", b["jump_y"])]:
                a = lines(np.column_stack([xx[left], sy + b["left_slope"] * (xx[left] - sx)]))
                c = lines(np.column_stack([xx[right], sy + jump + b["right_slope"] * (xx[right] - sx)]))
                match = response(fhog(patch(np.maximum(a, c), (sx, sy))), observed)
                row[layer][kind] = {"x": float(sx + match["dx"]), "y_left_endpoint_anchor": float(sy + match["dy"]), "peak": match["peak"]}
        controls.append(row)
    s = next(s for s in selected if s["frame"] == 6469)
    b = s["breaks"]["source_image"][1]
    at = camera_point(499, 591.5, cal)
    sensitivity = {"evaluation_pixel": [499, 591.5], "evaluation_note": "Convenient local evaluation location, not an accepted seam point.", "one_x_pixel_mm": float(np.linalg.norm(camera_point(500, 591.5, cal) - at)), "one_y_pixel_mm": float(np.linalg.norm(camera_point(499, 592.5, cal) - at))}
    endpoint_envelope = np.array([camera_point(x, y, cal) for x in np.linspace(498.5, 499, 11) for y in np.linspace(b["right_y"], b["left_y"], 201)])
    distance_ranges = {}
    for name in ["raw", "final", "sim"]:
        p = camera_point(s[name + "_x"], s[name + "_y"], cal)
        distances = np.linalg.norm(endpoint_envelope - p, axis=1)
        distance_ranges[name] = {"min_mm": float(distances.min()), "max_mm": float(distances.max())}
    rows = list(csv.DictReader((RUN / "vision_diagnostics.tsv").open(encoding="utf-8-sig"), delimiter="\t"))
    frame_row = next(r for r in rows if r["frame_id"] == "6469")
    online = [r for r in rows if r["phase"] == "online_running"]
    jumps = {"reinitialized": [], "other": []}
    for a, c in zip(online, online[1:]):
        if int(c["frame_id"]) - int(a["frame_id"]) > 4:
            continue
        dx = (float(c["before_fusion_x_px"]) - float(c["sim_x_px"])) - (float(a["before_fusion_x_px"]) - float(a["sim_x_px"]))
        jumps["reinitialized" if c["template_reinitialized"] == "1" else "other"].append(dx)
    reset = {k: {"n": len(v), "median_delta_raw_minus_sim_x": statistics.median(v), "p10_p90": np.percentile(v, [10, 90]).tolist()} for k, v in jumps.items()}
    output = {"human_reference": {"frame": 6469, "meaning": "actual joint, roughly below the red simulated point; not an extrapolated intersection", "exact_pixel_annotation": False, "scope": "this frame only; other frames have geometric estimates without user labels"}, "decomposition": decomposition, "controlled_matches": controls, "local_projection_sensitivity": sensitivity, "frame6469_fusion": {k: frame_row[k] for k in ["response_peak", "normalized_peak", "kcf_weight_x", "before_fusion_x_px", "final_x_px"]}, "frame6469_endpoint_envelope_sensitivity": {"note": "Testing envelope of visible stripe endpoints, not human annotation uncertainty, not actual robot error.", "x_px": [498.5, 499], "y_px": [b["right_y"], b["left_y"]], "distances": distance_ranges}, "reinitialization_jumps": reset, "findings": ["KCF horizontal deviation remains in the user-confirmed example.", "Red-green distance mixes virtual-model displacement and KCF localization displacement.", "Response peak is not a calibrated measure of seam-position accuracy.", "Fresh geometry-aware templates improve some frames but fail on others; no validated production fix.", "Y must preserve actual seam semantics, not default to intersection/midpoint/constant offset."]}
    (OUT / "seam_reference_followup.json").write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    with (OUT / "seam_reference_decomposition.csv").open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(decomposition[0]))
        w.writeheader()
        w.writerows(decomposition)
    build_page(output, s)
    print(json.dumps({"human_reference": output["human_reference"], "sensitivity": sensitivity, "reset": reset, "frame6469_fusion": output["frame6469_fusion"], "selected_frames": len(selected)}, ensure_ascii=False, indent=2))


def build_page(result, sample):
    b = sample["breaks"]["source_image"][1]
    panels = []
    for layer, title in [("source", "原图"), ("enhanced", "增强图")]:
        uri = "data:image/png;base64," + base64.b64encode((RUN / sample[layer]).read_bytes()).decode()
        svg = f'<svg viewBox="450 548 110 84"><image href="{uri}" width="1292" height="964"/><rect x="498.2" y="{b["right_y"]}" width="1.6" height="{b["left_y"]-b["right_y"]}" fill="#43efff" opacity=".75"/><circle cx="500" cy="586" r="1.5" fill="none" stroke="#ff556c" stroke-width=".5"/>'
        for x, y, c, r in [(sample["raw_x"], sample["raw_y"], "#ffc05b", 2), (sample["final_x"], sample["final_y"], "#46ff7a", 1)]:
            svg += f'<path d="M{x-r} {y}h{2*r}M{x} {y-r}v{2*r}" stroke="{c}" stroke-width=".4"/>'
        svg += '</svg>'
        panels.append(f'<section><h3>{title} · 6469帧</h3>{svg}</section>')
    table = []
    for r in result["decomposition"]:
        table.append('<tr>' + ''.join(f'<td>{v}</td>' for v in [r["frame"], "用户已说明" if r["user_reference_available"] else "仅图像几何参考", f'{r["total_raw_minus_sim_px"]:+.2f}', f'{r["visible_break_minus_sim_px"]:+.2f}', f'{r["raw_minus_visible_break_px"]:+.2f}']) + '</tr>')
    control_table = []
    for r in result["controlled_matches"]:
        if r["frame"] not in [6469, 7570, 9411, 12363, 14847]:
            continue
        e = r["enhanced"]
        control_table.append('<tr>' + ''.join(f'<td>{v}</td>' for v in [r["frame"], f'{r["visible_break_x"]:.2f}', f'{e["continuous_v"]["x"]:.2f}', f'{e["step"]["x"]:.2f}', f'{r["production_raw_x"]:.2f}']) + '</tr>')
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>接缝参考明确后的继续分析</title><style>body{font:16px/1.7 'Microsoft YaHei',sans-serif;color:#21353f;background:#edf2f4;margin:0}main{max-width:1060px;padding:28px 22px;margin:auto}article{background:white;border:1px solid #d8e2e7;border-radius:12px;padding:24px;margin:22px 0}h1{font-size:28px}h2{font-size:21px}h3{font-size:16px}.pair{display:flex;gap:18px}.pair section{width:50%;min-width:0}svg{width:100%;background:black;border-radius:8px}table{width:100%;border-collapse:collapse;font-size:14px}td,th{padding:9px;border-bottom:1px solid #dbe4e8;text-align:right}th:first-child,td:first-child{text-align:left}.muted{color:#60727c;font-size:14px}.note{border-left:4px solid #169fac;padding-left:16px}a{color:#196b80}@media(max-width:700px){.pair{display:block}.pair section{width:100%}}</style><main>
<p class="muted">2026-10-21 · 基于用户对6469帧接缝位置的说明 · 继续分析，未修改生产代码</p><h1>横向偏差成立；纵向位置必须符合实际接缝定义</h1>
<article><p class="note"><b>用户参考：</b>真实焊点位于实际接缝处，大致在红色仿真点下方一点。这不是两侧条纹的延长线交点。本轮不把“下面一点”转成未经确认的精确Y，也不推广成其他帧的人工真值。</p><div class="pair">PANELS</div><p class="muted">红圈：仿真点；橙十字：原始KCF；绿十字：最终输出；青色短条：可见条纹上下端点包络，仅供观察，不代表唯一焊点。</p><p>6469帧，仿真X=500.00，原始KCF X=510.69。三种窗口估计的接缝X为498.5～499.0，与用户所指位置横向接近。KCF横向仍偏右约11～12 px；纵向两侧端点约为587.5与595.7，不能直接取中点或任一端点作为用户认可的精确Y。</p></article>
<article><h2>红绿间距实际上包含两部分</h2><p>原始KCF−仿真点 =（可见接缝−仿真点）+（原始KCF−可见接缝）。下表只分解X，正数向右；除6469帧外，参考是图像几何估计。</p><table><thead><tr><th>帧</th><th>参考来源</th><th>KCF−仿真</th><th>接缝−仿真</th><th>KCF−接缝</th></tr></thead><tbody>TABLE</tbody></table><p>例如9411帧红绿相差25.96 px，但KCF比可见接缝略靠左；因此“绿点在红点右边”并不必然代表“绿点比真实焊缝偏右”。6469帧则相反：仿真X已很接近接缝，KCF本身的横向偏差占主要部分。</p></article>
<article><h2>高响应和正常融合不能证明焊点准确</h2><p>6469帧相关响应峰值为0.839，自动融合的KCF权重约99.00%。融合只把X从510.69拉到510.58，约0.11 px。算法认为区域匹配可信，并没有独立确认接缝点准确。当前权重主要依据相关响应，不是经过标定的焊点误差。</p><p>151次可与前帧比较的模板重建，KCF−仿真X变化中位数仅+0.45 px，且有正有负。没有发现每次重建都固定向右跳约12 px的证据；但重建保留旧位置，不能因此排除历史影响。</p></article>
<article><h2>“下方一点”对三维结果可能很重要</h2><p>按已与归档交叉核对的标定，在本帧附近，横向1 px约对应0.073 mm，纵向1 px约对应0.226 mm，局部纵向敏感程度约为横向的3.1倍。这个比例只适用于该图像区域，不能作为全图换算常数。</p><p>因此，横向看似贴近的红点也不一定在三维上更准确；必须同时确认Y。把可见上下端点范围用于敏感性试算时，仿真点和KCF到该范围的三维距离区间存在重叠，现有模糊Y标记不足以给两者的三维准确性排出确定先后。</p></article>
<article><h2>单帧模板对照：支持几何定位方向，但没有得到可直接使用的修复</h2><p>保持同一图像和名义接缝位置，只改变模板是否允许台阶。下表使用增强图，列出独立参考实现的输出X；这不是生产C++连续回放，模板斜率来自同帧拟合，只用于检查机制。</p><table><thead><tr><th>帧</th><th>可见接缝X</th><th>连续V模板X</th><th>台阶模板X</th><th>生产原始KCF X</th></tr></thead><tbody>CONTROLS</tbody></table><p>6469、7570等帧的台阶模板能更接近可见接缝，但9411、12363等帧仍有明显误差。这说明仅“把模板改成台阶”不足以解决问题，也不能把单帧对照的改善说成生产修复已验证。</p></article>
<article><h2>下一步建议的验证边界</h2><ol><li>KCF继续提供搜索范围和时间连续性；在原图上识别真实接缝X、两侧端点及局部条纹形态。</li><li>焊点Y依据实际接缝语义选择；不默认采用延长线交点、两端中点、仿真Y或固定像素偏移。精确验收应使用少量人工标注的代表帧，并包含台阶、连续折点、反光和偏左反例。</li><li>几何证据不足时，报告“未确认接缝”，区分跟踪框有效与焊点有效；不得为了曲线稳定直接把未校核点当作准确焊点。</li><li>比较新的二维定位和三维投影结果，再检查在线纠偏、ICP独立验收和后验检查；当前保护门槛暂不调整。</li></ol><p>本轮仍未证明具体一次ICP误拒绝/误放行，也未获得原始连续视频和每次完整模板状态，因此没有宣称精确主因已经唯一确定。</p></article>
<article><h2>材料与边界</h2><p>原始归档和生产源码未修改；输入哈希再次核对一致。对照使用已有Python、NumPy、Pillow和本机OpenCV，不安装依赖、不构建、不操作机器人、不调用鼠标。</p><p><a href="seam_reference_followup.json">完整数值与实验</a> · <a href="seam_reference_decomposition.csv">X偏差分解CSV</a> · <a href="index.html">上一轮逐帧报告</a></p></article></main></html>'''
    page = page.replace("PANELS", ''.join(panels)).replace("TABLE", ''.join(table)).replace("CONTROLS", ''.join(control_table))
    (OUT / "seam_reference_followup.html").write_text(page, encoding="utf-8")


if __name__ == "__main__":
    main()
