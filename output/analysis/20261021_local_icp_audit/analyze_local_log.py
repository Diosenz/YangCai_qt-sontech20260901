"""Parse only workpiece local surface ICP; preserve log and session source files.

Usage: bundled-python analyze_local_log.py /absolute/path/i2r_rlc.log
Fixed gates are taken from the current config/code, not a saved runtime snapshot.
Search distance and translation limits are reconstructed from logged UI changes.
"""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re

from PIL import Image, ImageDraw, ImageFont
import numpy as np
import pandas as pd

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
parser = argparse.ArgumentParser()
parser.add_argument("log", type=Path)
args = parser.parse_args()
source = args.log.read_bytes()
digest = hashlib.sha256(source).hexdigest()
lines = source.decode("utf-8-sig").splitlines()
rows, events = [], []
session, search, limit = "unknown", 5.0, 5.0
for line_number, line in enumerate(lines, 1):
    if "在线局部定位：" in line:
        search = float(re.search(r"对应距离 ([\d.]+)", line)[1])
        limit = float(re.search(r"最大平移修正 ([\d.]+)", line)[1])
    if "局部 ICP 最大对应距离已调整为" in line:
        search = float(re.search(r"调整为 ([\d.]+)", line)[1])
    if "局部 ICP 最大平移修正已调整为" in line:
        limit = float(re.search(r"调整为 ([\d.]+)", line)[1])
    if "在线纠偏运行记录：" in line:
        session = line.rsplit("/", 1)[-1]
    if any(k in line for k in ("在线局部定位：", "局部 ICP 最大", "在线纠偏运行记录：")):
        events.append(dict(line=line_number, text=line))
    if "[在线局部定位]" not in line:
        continue
    m = re.search(r"前=([\d.eE+-]+)，后=([\d.eE+-]+) mm，平移=([\d.eE+-]+) mm，旋转=([\d.eE+-]+) 度，求解=(\d+) ms", line)
    if m is None:
        raise ValueError(f"Unparsed local ICP record at line {line_number}: {line}")
    before, after, translation, rotation, solve = map(float, m.groups())
    failures = [name for name, bad in (
        ("residual_gt_2mm", after > 2), ("improvement_lt_5pct", after > before * .95),
        ("translation_gt_runtime_limit", translation > limit),
        ("rotation_gt_1deg", rotation > 1), ("solve_gt_300ms", solve > 300)) if bad]
    rows.append(dict(line=line_number, time=line[1:24], session=session,
        frame=int(re.search(r"frame=(\d+)", line)[1]),
        before_mm=before, after_mm=after, translation_mm=translation,
        rotation_deg=rotation, solve_ms=solve,
        processing_ms=int(re.search(r"采集及处理=(\d+)", line)[1]),
        search_distance_mm=search, translation_limit_mm=limit,
        improvement_ratio=1-after/before, failure_gates=failures,
        logged_rejected="局部配准未通过" in line,
        logged_pipeline_completed="局部定位离线流程完成" in line))


def stats(values):
    return dict(minimum=float(np.min(values)), median=float(np.median(values)), maximum=float(np.max(values)))


summaries = []
sessions = list(dict.fromkeys(r["session"] for r in rows))
image = Image.new("RGB", (1600, 1130), "white")
draw = ImageDraw.Draw(image)
font = ImageFont.load_default(size=16)
small = ImageFont.load_default(size=13)
draw.text((35, 16), "Online workpiece local surface ICP: logged candidates and acceptance gates", fill="#202020", font=font)
draw.text((35, 42), "Blue: candidate; grey: before ICP; red: acceptance limit. Local point-cloud ICP only.", fill="#555555", font=small)
for column, name in enumerate(sessions):
    group = [r for r in rows if r["session"] == name]
    data_file = next(iter((ROOT / "output/online_correction").glob(name+"*/vision_diagnostics.tsv")), None)
    if data_file:
        data = pd.read_csv(data_file, sep="\t", low_memory=False).set_index("frame_id")
        for row in group:
            row["s2_deg"] = float(data.loc[row["frame"], "s2_deg"])
    summary = dict(session=name, attempts=len(group), rejected=sum(r["logged_rejected"] for r in group),
        pipeline_completed=sum(r["logged_pipeline_completed"] for r in group),
        joined_saved_vision_frames=sum("s2_deg" in r for r in group),
        search_distance_mm=sorted(set(r["search_distance_mm"] for r in group)),
        translation_limit_mm=sorted(set(r["translation_limit_mm"] for r in group)),
        failure_counts=dict(Counter(g for r in group for g in r["failure_gates"])),
        residual_worsened=sum(r["after_mm"] > r["before_mm"] for r in group),
        processing_gt_1500ms=sum(r["processing_ms"] > 1500 for r in group),
        translation_rotation_correlation=float(np.corrcoef([r["translation_mm"] for r in group], [r["rotation_deg"] for r in group])[0, 1]),
        metrics={k:stats([r[k] for r in group]) for k in ("before_mm", "after_mm", "translation_mm", "rotation_deg", "solve_ms", "processing_ms")},
        only_residual_gate_failed=sum(r["failure_gates"] == ["residual_gt_2mm"] for r in group),
        fixed_candidate_residual_threshold_sensitivity={str(t):sum(r["after_mm"] <= t and all(g == "residual_gt_2mm" for g in r["failure_gates"]) for r in group) for t in (2, 3, 4, 5)},
        residual_pass_examples=[r for r in group if r["after_mm"] <= 2])
    summaries.append(summary)
    start = datetime.fromisoformat(group[0]["time"])
    x = [(datetime.fromisoformat(r["time"])-start).total_seconds() for r in group]
    for index, (key, gate, label) in enumerate((
        ("after_mm",2,"Mean NN residual (mm)"), ("translation_mm",group[0]["translation_limit_mm"],"Translation norm (mm)"),
        ("rotation_deg",1,"Rotation angle (deg)"), ("solve_ms",300,"Pipeline solve time (ms)"))):
        left, top = 85 + column * 790, 150 + index * 232
        width, height = 650, 160
        values = [r[key] for r in group]
        maximum = max(values + [gate] + ([r["before_mm"] for r in group] if index == 0 else [])) * 1.1
        def point(t, v):
            return (left + t / max(x[-1], 1) * width, top + height - v / maximum * height)
        for tick in range(5):
            value = maximum * tick / 4
            y = point(0, value)[1]
            draw.line((left, y, left+width, y), fill="#e8e8e8")
            draw.text((left-58, y-7), f"{value:.1f}", fill="#555555", font=small)
        for tick in range(5):
            value = x[-1] * tick / 4
            xx = point(value, 0)[0]
            draw.text((xx-12, top+height+8), f"{value:.0f}", fill="#555555", font=small)
        draw.line((left, top, left, top+height, left+width, top+height), fill="#333333", width=1)
        ygate = point(0, gate)[1]
        draw.line((left, ygate, left+width, ygate), fill="#c33b32", width=2)
        draw.text((left+width-100, ygate-18), f"limit {gate:g}", fill="#c33b32", font=small)
        if index == 0:
            draw.line([point(t, r["before_mm"]) for t, r in zip(x, group)], fill="#999999", width=2)
            draw.text((left, 83), name, fill="#202020", font=font)
            draw.text((left, 108), f"{len(group)} attempts; search = {group[0]['search_distance_mm']:g} mm; all rejected", fill="#333333", font=small)
        draw.line([point(t, value) for t, value in zip(x, values)], fill="#1768ac", width=2)
        for t, value in zip(x, values):
            px, py = point(t, value)
            draw.ellipse((px-2, py-2, px+2, py+2), fill="#1768ac")
        draw.text((left, top-24), label, fill="#202020", font=font)
        draw.text((left+170, top+height+28), "Seconds since first local ICP result", fill="#555555", font=small)
image.save(OUT / "local_icp_log_timeline.png")
report = dict(source_log=str(args.log), source_log_sha256=digest,
    log_start=lines[0], log_end=lines[-1],
    scope="Only [在线局部定位] point-cloud ICP records; trajectory ICP excluded.",
    fixed_gate_basis="Current online_local_registration.ini and worker acceptance code: residual<=2mm, improvement>=5%, rotation<=1deg, pipeline solve<=300ms. These four settings are not printed as runtime snapshots in the supplied log.",
    parameter_events=events, sessions=summaries, records=rows,
    limitations=["No source/reference local surface clouds or correction matrices are saved in this log.",
                 "21:37 session is present in the log but its session data directory was not supplied.",
                 "Earlier 16:28/20:22/20:41 local ICP attempts are outside the supplied log's local-ICP coverage.",
                 "Threshold sensitivity rechecks existing candidates only; it is not a new ICP solve or a writeback validation."])
assert hashlib.sha256(args.log.read_bytes()).hexdigest() == digest
(OUT / "local_icp_log_results.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(summaries, ensure_ascii=False, indent=2))
print("Source log unchanged; all local ICP lines parsed:", len(rows))
