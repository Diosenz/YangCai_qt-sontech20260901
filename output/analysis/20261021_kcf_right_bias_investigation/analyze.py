"""Offline evidence audit. Does not change production code or source recordings.

Corner estimates describe visible laser folds, not calibrated seam ground truth.
All input samples and rejected fits remain in results.json and samples.csv.
"""
from pathlib import Path
import csv
import hashlib
import html
import json
import math
import statistics
import urllib.request

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
RUNS = list((ROOT / "output/online_correction").glob("20261021_201400_950*"))
assert len(RUNS) == 1, "Recording identity must resolve uniquely"
RUN = RUNS[0]


def dump(name, value):
    (OUT / name).write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fetch_workflows():
    pages = []
    for pid in ["3f7aa04f-a8a9-8179-8bda-eec496f904f1", "3fdaa04f-a8a9-8191-95d2-dc0eb65ce630", "3faaa04f-a8a9-81d1-b419-d2fc27a13cb4"]:
        payload = {"pageId": pid, "limit": 100, "cursor": {"stack": []}, "chunkNumber": 0, "verticalColumns": False}
        req = urllib.request.Request("https://www.notion.so/api/v3/loadPageChunk", data=json.dumps(payload).encode(), headers={"Content-Type": "application/json", "User-Agent": "Mozilla/5.0"})
        result = json.loads(urllib.request.urlopen(req, timeout=15).read())
        blocks = []
        for item in result.get("recordMap", {}).get("block", {}).values():
            while isinstance(item, dict) and "value" in item:
                item = item["value"]
            if isinstance(item, dict) and item.get("type"):
                blocks.append({"id": item.get("id"), "type": item["type"], "version": item.get("version"), "text": "".join(str(x[0]) for x in item.get("properties", {}).get("title", []))})
        pages.append({"id": pid, "blocks": blocks})
    dump("notion_workflow_snapshot.json", {"source": "live public Notion read", "pages": pages})


def robust_lstsq(a, y):
    co = np.linalg.lstsq(a, y, rcond=None)[0]
    for _ in range(5):
        error = y - a @ co
        # sqrt(weights) for Huber loss, keeping the recorded outliers untouched.
        w = np.sqrt(np.minimum(1, 2.0 / np.maximum(np.abs(error), 0.001)))
        co = np.linalg.lstsq(a * w[:, None], y * w, rcond=None)[0]
    return co


def corner_fit(image, sx, sy, radius=100, clip=None):
    x0, x1 = max(0, int(sx) - radius), min(image.shape[1], int(sx) + radius + 1)
    y0, y1 = max(0, int(sy) - 60), min(image.shape[0], int(sy) + 61)
    if clip is not None:
        rx, ry, rw, rh = clip
        x0, x1 = max(x0, rx + 8), min(x1, rx + rw - 8)
        y0, y1 = max(y0, ry + 8), min(y1, ry + rh - 8)
    xs = np.arange(x0, x1)
    z = image[y0:y1, x0:x1].astype(float)
    smooth = (z[:-2] + z[1:-1] + z[2:]) / 3
    peak = np.max(smooth, axis=0)
    ys = np.argmax(smooth, axis=0) + y0 + 1
    keep = peak > max(45, np.median(peak) * 0.35)
    xx, yy = xs[keep], ys[keep]
    best = None
    for cx in np.arange(max(x0 + 25, sx - 70), min(x1 - 25, sx + 101), 0.5):
        a = np.column_stack([np.ones(len(xx)), np.minimum(xx - cx, 0), np.maximum(xx - cx, 0)])
        co = robust_lstsq(a, yy)
        error = yy - a @ co
        # Huber score; no branch or sample removed from the output.
        ab = np.abs(error)
        score = float(np.mean(np.where(ab <= 2, error ** 2, 4 * ab - 4)))
        if co[1] > 0.05 and co[2] < -0.05 and (best is None or score < best["score"]):
            inliers = np.abs(error) <= 3
            best = {"x": float(cx), "y": float(co[0]), "left_slope": float(co[1]), "right_slope": float(co[2]), "score": score,
                    "inlier_rms": float(np.sqrt(np.mean(error[inliers] ** 2))) if inliers.any() else None,
                    "inlier_fraction": float(inliers.mean()), "n_columns": int(len(xx)), "left_columns": int(np.sum(xx < cx)), "right_columns": int(np.sum(xx > cx))}
    return best


def break_fit(image, sx, sy, radius=100, clip=None):
    """Allow a discontinuous stripe, rather than force a continuous V."""
    x0, x1 = max(0, int(sx) - radius), min(image.shape[1], int(sx) + radius + 1)
    y0, y1 = max(0, int(sy) - 60), min(image.shape[0], int(sy) + 61)
    if clip:
        rx, ry, rw, rh = clip
        x0, x1 = max(x0, rx + 8), min(x1, rx + rw - 8)
        y0, y1 = max(y0, ry + 8), min(y1, ry + rh - 8)
    xx = np.arange(x0, x1)
    z = image[y0:y1, x0:x1].astype(float)
    smooth = (z[:-2] + z[1:-1] + z[2:]) / 3
    yy = np.argmax(smooth, axis=0) + y0 + 1
    best = None
    for cx in np.arange(x0 + 35, x1 - 35, .5):
        left = xx < cx
        right = ~left
        a = np.column_stack([left, left * (xx - cx), right, right * (xx - cx)])
        co = robust_lstsq(a, yy)
        e = yy - a @ co
        ae = np.abs(e)
        score = float(np.mean(np.where(ae <= 2, e * e, 4 * ae - 4)))
        if co[1] > .05 and co[3] < -.05 and (best is None or score < best["score"]):
            best = {"x": float(cx), "left_y": float(co[0]), "right_y": float(co[2]), "left_slope": float(co[1]), "right_slope": float(co[3]), "jump_y": float(co[2] - co[0]), "score": score}
    return best


def camera_point(x, y, cal):
    dx, dy = x - cal["cx"], y - cal["cy"]
    scale = 1 / (1 + cal["Kappa"] * ((cal["sx/m"] * dx) ** 2 + (cal["sy/m"] * dy) ** 2))
    ray = np.array([scale * dx * cal["sx/m"] / cal["f/m"], scale * dy * cal["sy/m"] / cal["f/m"], 1.0])
    return -1000 / np.dot([cal["a"], cal["b"], cal["c"]], ray) * ray


def distribution(values):
    a = np.asarray(values)
    return {"n": len(a), "mean": float(a.mean()), "median": float(np.median(a)), "min": float(a.min()), "max": float(a.max()), "p10": float(np.percentile(a, 10)), "p90": float(np.percentile(a, 90)), "positive_pct": float(100 * (a > 0).mean())}


def main():
    if not (OUT / "notion_workflow_snapshot.json").exists():
        fetch_workflows()
    rows = list(csv.DictReader((RUN / "vision_diagnostics.tsv").open(encoding="utf-8-sig"), delimiter="\t"))
    cal = {}
    for line in (ROOT / "config/calibration/SENSORPARA.txt").read_text(encoding="utf-8-sig").splitlines():
        bits = line.split()
        if len(bits) > 1:
            try:
                cal[bits[0]] = float(bits[1])
            except ValueError:
                pass
    online = [r for r in rows if r["phase"] == "online_running" and r["result_valid"] == "1" and r["correlation_jump_rejected"] == r["fusion_jump_rejected"] == "0"]
    stats = {}
    for label in ["before_fusion", "final"]:
        stats[label] = distribution([float(r[label + "_x_px"]) - float(r["sim_x_px"]) for r in online])
    bins = []
    for lo, hi in [(0, 1), (1, 5), (5, 15), (15, 30), (30, 52)]:
        rs = [r for r in online if lo <= int(r["frames_since_initialize"]) < hi]
        if rs:
            bins.append({"frames_since_init": [lo, hi], "dx": distribution([float(r["before_fusion_x_px"]) - float(r["sim_x_px"]) for r in rs]), "median_peak": statistics.median(float(r["response_peak"]) for r in rs)})
    samples = []
    hashes = {str(p.relative_to(ROOT)): sha(p) for p in [RUN / "vision_diagnostics.tsv", RUN / "session_summary.txt", ROOT / "config/calibration/SENSORPARA.txt", ROOT / "Vision/src/track/KcfTracker.cpp", ROOT / "Vision/src/track/KcfFhog.cpp", ROOT / "Vision/src/track/LegacyOnlineKcfDetector.cpp", ROOT / "Vision/src/preprocess/ImageDenoise.cpp", ROOT / "Vision/src/geometry/WeldImageToSpaceProjector.cpp"]}
    for r in rows:
        if not r["source_image"]:
            continue
        sx, sy = float(r["sim_x_px"]), float(r["sim_y_px"])
        clip = [int(float(r["enhancement_roi_" + k])) for k in ["x", "y", "width", "height"]]
        fits = {}
        breaks = {}
        for key in ["source_image", "enhanced_image"]:
            path = RUN / r[key]
            hashes[str(path.relative_to(ROOT))] = sha(path)
            im = np.asarray(Image.open(path).convert("L"))
            fits[key] = [corner_fit(im, sx, sy, rad, clip) for rad in [85, 100, 115]]
            breaks[key] = [break_fit(im, sx, sy, rad, clip) for rad in [85, 100, 115]]
        ref = fits["source_image"][1]
        enf = fits["enhanced_image"][1]
        valid_fits = [v for v in fits["source_image"] if v]
        spread = max(v["x"] for v in valid_fits) - min(v["x"] for v in valid_fits) if valid_fits else None
        eligible = r["phase"] == "online_running" and len(valid_fits) == 3 and spread <= 4 and ref["inlier_fraction"] >= .8 and min(ref["left_columns"], ref["right_columns"]) >= 35
        item = {"frame": int(r["frame_id"]), "phase": r["phase"], "sim_x": sx, "sim_y": sy, "raw_x": float(r["before_fusion_x_px"]), "raw_y": float(r["before_fusion_y_px"]), "final_x": float(r["final_x_px"]), "final_y": float(r["final_y_px"]), "peak": float(r["response_peak"]), "since_init": int(r["frames_since_initialize"]), "reinitialized": r["template_reinitialized"], "source": r["source_image"], "enhanced": r["enhanced_image"], "clip": clip, "fits": fits, "fit_spread_x": spread, "eligible": bool(eligible)}
        bf = breaks["source_image"]
        valid_breaks = [b for b in bf if b]
        break_spread = max(b["x"] for b in valid_breaks) - min(b["x"] for b in valid_breaks) if valid_breaks else None
        break_eligible = r["phase"] == "online_running" and len(valid_breaks) == 3 and break_spread <= 4 and bf[1]["score"] < 2.5
        item.update(breaks=breaks, break_eligible=bool(break_eligible), break_spread_x=break_spread)
        if bf[1]:
            item.update(break_x=bf[1]["x"], break_left_y=bf[1]["left_y"], break_right_y=bf[1]["right_y"], break_jump_y=bf[1]["jump_y"], raw_break_dx=item["raw_x"] - bf[1]["x"])
            endpoints = [camera_point(bf[1]["x"], bf[1][side], cal) for side in ["left_y", "right_y"]]
            raw_camera = camera_point(item["raw_x"], item["raw_y"], cal)
            item["raw_break_endpoint_min_mm"] = min(float(np.linalg.norm(raw_camera - p)) for p in endpoints)
            item["raw_break_endpoint_max_mm"] = max(float(np.linalg.norm(raw_camera - p)) for p in endpoints)
        if ref:
            item.update(corner_x=ref["x"], corner_y=ref["y"], raw_corner_dx=item["raw_x"] - ref["x"], sim_corner_dx=sx - ref["x"], raw_corner_distance_mm=float(np.linalg.norm(camera_point(item["raw_x"], item["raw_y"], cal) - camera_point(ref["x"], ref["y"], cal))))
        if ref and enf:
            item["enhanced_corner_dx"] = enf["x"] - ref["x"]
            item["enhanced_corner_dy"] = enf["y"] - ref["y"]
        samples.append(item)
    accepted = [s for s in samples if s["eligible"]]
    stats["sample_raw_to_corner_dx"] = distribution([s["raw_corner_dx"] for s in accepted])
    stats["sample_raw_to_corner_mm"] = distribution([s["raw_corner_distance_mm"] for s in accepted])
    stats["sample_enhanced_to_corner_dx"] = distribution([s["enhanced_corner_dx"] for s in accepted if "enhanced_corner_dx" in s])
    stats["near_aligned_samples"] = [s["frame"] for s in accepted if abs(s["sim_corner_dx"]) <= 4]
    stable_breaks = [s for s in samples if s["break_eligible"]]
    stats["break_stable_n"] = len(stable_breaks)
    stats["sample_raw_to_visible_break_dx"] = distribution([s["raw_break_dx"] for s in stable_breaks])
    stats["sample_raw_to_visible_break_endpoint_min_mm"] = distribution([s["raw_break_endpoint_min_mm"] for s in stable_breaks])
    result = {"run": str(RUN.relative_to(ROOT)), "diagnostic_count": len(rows), "online_count": len(online), "saved_pairs": len(samples), "eligible_pairs": len(accepted), "statistics": stats, "reinit_bins": bins, "samples": samples,
              "limitations": ["Sparse saved frames, no full temporal replay.", "Visible fold is an image-derived estimate, not seam ground truth.", "Camera-plane metric uses current SENSORPARA; it is rigid-frame invariant but calibration was not snapshotted at capture.", "No production rebuild, changes, or online robot action."]}
    dump("results.json", result)
    dump("input_hashes.json", hashes)
    cols = ["frame", "phase", "eligible", "corner_x", "corner_y", "sim_x", "sim_y", "raw_x", "raw_y", "final_x", "final_y", "raw_corner_dx", "sim_corner_dx", "raw_corner_distance_mm", "enhanced_corner_dx", "enhanced_corner_dy", "fit_spread_x", "peak", "since_init", "break_eligible", "break_x", "break_jump_y", "raw_break_dx", "raw_break_endpoint_min_mm", "raw_break_endpoint_max_mm"]
    with (OUT / "samples.csv").open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(samples)
    print(json.dumps({k: v for k, v in result.items() if k != "samples"}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
