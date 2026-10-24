"""离线诊断原型：生成图像证据与单文件回放，不连接机器人或修改生产参数。"""
import base64
import hashlib
import io
import json
import pathlib
import re
import subprocess
import sys

import numpy as np
from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parents[3]
OUT = ROOT / "output/analysis/online_kcf_prototype_20261024"
REPRO = ROOT / "output/analysis/online_correction_repro_20261024"


def image_uri(image):
    stream = io.BytesIO()
    Image.fromarray(image.astype(np.uint8)).save(stream, format="PNG")
    return "data:image/png;base64," + base64.b64encode(stream.getvalue()).decode()


def peaks(image, x):
    column = image[:, x].astype(float)
    # 分离相邻峰，而不是只取最亮像素；阈值仅为合成场景候选值。
    indices = np.where(column > 95)[0]
    groups = np.split(indices, np.where(np.diff(indices) > 1)[0] + 1)
    return [float(np.average(g, weights=column[g])) for g in groups if 1 <= len(g) <= 9]


def side_lines(image, xs):
    samples = [(x, y) for x in xs for y in peaks(image, x)]
    if len(samples) < 10:
        return []
    lines = []
    # 两端组合提出线模型，其余原图列验收；不读取合成真值。
    for x1, y1 in samples[:6]:
        for x2, y2 in samples[-6:]:
            if abs(x2 - x1) < 25:
                continue
            a = (y2 - y1) / (x2 - x1)
            b = y1 - a * x1
            support = [(x, y) for x, y in samples if abs(y - a * x - b) < 1.8]
            if len({x for x, _ in support}) < .7 * len(xs):
                continue
            a, b = np.polyfit(*zip(*support), 1)
            if not any(abs(a - p[0]) < .035 and abs(b + a * 160 - p[1] - p[0] * 160) < 3
                       for p in lines):
                lines.append([float(a), float(b)])
    return lines


def extract(image):
    left = [l for l in side_lines(image, range(70, 119)) if .1 < l[0] < .7]
    right = [l for l in side_lines(image, range(207, 256)) if -.7 < l[0] < -.1]
    candidates = []
    for a, b in left:
        for c, d in right:
            x = (d - b) / (a - c)
            y = a * x + b
            if 130 < x < 190 and 85 < y < 170:
                central = peaks(image, round(x))
                visible = any(abs(p - y) < 2.5 for p in central)
                candidates.append({"point": [x, y], "visible": visible})
    return {"candidates": candidates, "lines": left + right,
            "geometry": candidates[0]["point"] if len(candidates) == 1 and candidates[0]["visible"] else None,
            "partial": candidates[0]["point"] if len(candidates) == 1 else None,
            "ambiguous": len(candidates) > 1,
            "singleLine": (left + right)[0] if len(left) + len(right) == 1 else None}


def make_sequence(name, description, kinds, pose_jump=False, sim_missing=False, hidden_shift=False):
    frames = []
    yy, xx = np.mgrid[:220, :340]
    for i, kind in enumerate(kinds):
        truth = [160 + .32 * i, 130 + .06 * i]
        if hidden_shift and i >= 8:
            truth[0] += 5
        line = truth[1] - .32 * np.abs(xx - truth[0])
        stripe = 200 * np.exp(-.5 * ((yy - line) / 1.1) ** 2)
        if kind in ("tack", "blind", "single"):
            stripe[np.abs(xx - truth[0]) < 30] = 0
        if kind == "tack":
            bulge = truth[1] - 13 + .012 * (xx - truth[0]) ** 2
            stripe += (np.abs(xx - truth[0]) < 30) * 240 * np.exp(-.5 * ((yy - bulge) / 1.8) ** 2)
        if kind == "single":
            stripe[xx >= truth[0]] = 0
        if kind == "blind":
            stripe[:] = 0
        if kind == "reflection":
            false_line = truth[1] - 19 - .32 * np.abs(xx - truth[0] - 9)
            stripe += 250 * np.exp(-.5 * ((yy - false_line) / 1.0) ** 2)
        rng = np.random.default_rng(891 + i)
        image = np.clip(stripe + rng.normal(10, 2, stripe.shape), 0, 255).astype(np.uint8)
        evidence = extract(image)
        # CAD 与真值各自定义；有刻意比例误差，防止恒定偏置模型假装精确。
        sim = [160 + .35 * i + (35 if pose_jump and i >= 10 else 0), 130 + .07 * i]
        frames.append(dict(id=i, t=i * .1, s=i * .2, kind=kind, truth=truth,
                           sim=None if sim_missing and i >= 8 else sim,
                           revision=2 if pose_jump and i >= 10 else 1,
                           image=image_uri(image), **evidence))
    return dict(name=name, description=description, synthetic=True, frames=frames)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    sequences = [
        make_sequence("正常实测", "原图双侧母材与中心可见；建立锚点后跟随真实变化。", ["clear"] * 24),
        make_sequence("预焊凸起与恢复", "凸起遮挡中心，母材仍可见；虚拟交点只能支持估计。", ["clear"] * 8 + ["tack"] * 7 + ["clear"] * 9),
        make_sequence("反光竞争", "另一条更亮的完整折线出现；多个候选时拒绝切换。", ["clear"] * 8 + ["reflection"] * 7 + ["clear"] * 9),
        make_sequence("遮挡预算耗尽", "长遮挡中任何估计均不能刷新原锚点；到限停止输出。", ["clear"] * 8 + ["blind"] * 18 + ["clear"] * 9),
        make_sequence("参考位姿跳变", "遮挡期 CAD 绝对位置突变 35 px；重对齐该参考，不拖动检测点。", ["clear"] * 8 + ["tack"] * 7 + ["clear"] * 9, pose_jump=True),
        make_sequence("单侧母材", "单侧直线只约束一个方向；沿线分量仍是预测。", ["clear"] * 8 + ["single"] * 7 + ["clear"] * 9),
        make_sequence("无锚点启动", "启动即被遮挡：不能用 CAD 绝对位置制造可靠检测锚。", ["blind"] * 10 + ["clear"] * 9),
        make_sequence("仿真中断", "独立原图几何仍可实测；失去几何后无法进行本原型的仿真续接。", ["clear"] * 10 + ["blind"] * 5 + ["clear"] * 9, sim_missing=True),
        make_sequence("预测偏差与恢复过渡", "合成压力场景：失测后焊缝偏离预测 5 px；观察预测误差与恢复时的平滑代价。", ["clear"] * 8 + ["blind"] * 5 + ["clear"] * 9, hidden_shift=True),
    ]
    if len(sys.argv) > 1:
        binary = pathlib.Path(sys.argv[1]).resolve()
        fixture_dir = OUT / "synthetic_images"
        fixture_dir.mkdir(exist_ok=True)
        for number, sequence in enumerate(sequences):
            for frame in sequence["frames"]:
                image_path = fixture_dir / f"scenario_{number}_frame_{frame['id']:03d}.png"
                image_path.write_bytes(base64.b64decode(frame["image"].split(",", 1)[1]))
                # 提示位置为独立 CAD 提案，不提供合成真值给基线算法。
                hint = frame["sim"] or [160, 130]
                result = subprocess.run([str(binary), str(image_path), *map(str, hint)], capture_output=True, text=True)
                match = re.search(r"valid=(\d).*point=\(([-\d.]+),([-\d.]+)\)", result.stdout)
                point = [float(match[2]), float(match[3])] if match and match[1] == "1" else None
                frame["baseline"] = dict(point=point, stdout=result.stdout,
                                         errorPixel=float(np.linalg.norm(np.array(point)-frame["truth"])) if point else None)
    # 真图只呈现生产代码回放结果；不把合成专用固定带提取器冒充现场算法。
    real = []
    for case in ("field", "anchor"):
        logs = sorted(REPRO.glob(f"refiner_{case}_*.json"), reverse=True)
        record = next(json.loads(p.read_text()) for p in logs
                      if not json.loads(p.read_text())["minimized"]
                      and all(v is None or v == 0 for v in json.loads(p.read_text())["probe_options"].values()))
        fixture = REPRO / "snapshot" / record["fixture"]
        real.append(dict(name=case, image="data:image/png;base64," + base64.b64encode(fixture.read_bytes()).decode(),
                         hint=record["hint"], result=record["stdout"], revision=record["revision"],
                         sha256=record["hashes"][record["fixture"]], truth=None))
    payload = dict(sequences=sequences, real=real,
                   provenance=dict(revision="430aaa204fd5319dd7f697d3de2b4e1faddbb06f",
                                   baselineBinarySha256=hashlib.sha256(binary.read_bytes()).hexdigest() if len(sys.argv)>1 else None,
                                   syntheticSeedBase=891,
                                   scope="合成专用固定搜索带提取；基线仅生产单帧精定位；不含完整KCF和控制"))
    core = pathlib.Path(__file__).with_name("prototype_logic.js").read_text()
    data_path = OUT / "inputs.json"
    data_path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    # 与交互页面运行完全相同的纯函数，逐帧保存实际产生的结果。
    run = subprocess.run(["node", str(pathlib.Path(__file__).with_name("prototype_logic.js")), str(data_path)],
                         capture_output=True, text=True, check=True)
    (OUT / "replay.json").write_text(run.stdout, encoding="utf-8")
    template = pathlib.Path(__file__).with_name("viewer.html").read_text()
    html = template.replace("/* CORE */", core).replace("/* INPUTS */", "const DATA=" + json.dumps(payload, ensure_ascii=False) + ";")
    (OUT / "prototype.html").write_text(html, encoding="utf-8")
    summary = json.loads(run.stdout)
    (OUT / "summary.json").write_text(json.dumps(summary["summary"], ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary["summary"], ensure_ascii=False, indent=2))
    print(OUT / "prototype.html")


if __name__ == "__main__":
    main()
