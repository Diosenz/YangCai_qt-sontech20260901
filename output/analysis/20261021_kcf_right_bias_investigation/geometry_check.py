"""Cross-check archived projection frames and single-frame ICP gate sensitivity."""
from pathlib import Path
import csv
import json
import numpy as np
from analyze import ROOT, OUT, RUN, camera_point, sha


def rot(rx, ry, rz):
    a, b, c = np.deg2rad([rx, ry, rz])
    x = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    y = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    z = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])
    return z @ y @ x


def main():
    rows = list(csv.DictReader((RUN / "vision_diagnostics.tsv").open(encoding="utf-8-sig"), delimiter="\t"))
    by_id = {int(r["frame_id"]): r for r in rows}
    cal = {}
    for line in (ROOT / "config/calibration/SENSORPARA.txt").read_text(encoding="utf-8-sig").splitlines():
        bits = line.split()
        if len(bits) > 1:
            try:
                cal[bits[0]] = float(bits[1])
            except ValueError:
                pass
    def vals(r, keys):
        return np.array([float(r[k]) for k in keys])
    b = np.array([vals(r, ["robot_base_" + k + "_mm" for k in "xyz"]) for r in rows])
    s = np.array([vals(r, ["station_base_" + k + "_mm" for k in "xyz"]) for r in rows])
    u, _, vt = np.linalg.svd((b - b.mean(axis=0)).T @ (s - s.mean(axis=0)))
    rb = vt.T @ u.T
    assert np.linalg.det(rb) > .999
    tb = s.mean(axis=0) - rb @ b.mean(axis=0)
    station_residual = np.linalg.norm(b @ rb.T + tb - s, axis=1)
    offsets = []
    for i, r in enumerate(rows):
        cur = rot(0, -float(r["s1_deg"]), -float(r["s2_deg"]))
        offsets.append(cur @ s[i] - vals(r, ["positioner_tcp_" + k + "_mm" for k in "xyz"]))
    tcp = np.mean(offsets, axis=0)
    rcam = rot(*[cal["pose_CinT[" + str(i) + "]/deg"] for i in range(3, 6)])
    tcam = np.array([cal["pose_CinT[" + str(i) + "]/m"] for i in range(3)]) * 1000
    def context(r):
        cur = rot(0, -float(r["s1_deg"]), -float(r["s2_deg"]))
        rt = rot(*vals(r, ["robot_r" + k + "_deg" for k in "xyz"]))
        tt = vals(r, ["robot_" + k + "_mm" for k in "xyz"])
        return cur @ rb @ rt @ rcam, cur @ (rb @ (rt @ tcam + tt) + tb) - tcp
    errors = []
    for r in rows:
        rr, tt = context(r)
        p = rr @ camera_point(float(r["final_x_px"]), float(r["final_y_px"]), cal) + tt
        errors.append(float(np.linalg.norm(p - vals(r, ["positioner_tcp_" + k + "_mm" for k in "xyz"]))))
    def to_pixels(points, r):
        rr, tt = context(r)
        camera = (np.asarray(points) - tt) @ rr
        uv = camera[:, :2] / camera[:, 2, None] * cal["f/m"]
        radius2 = np.sum(uv * uv, axis=1)
        factor = 2 / (1 + np.sqrt(1 - 4 * cal["Kappa"] * radius2))
        return uv * factor[:, None] / [cal["sx/m"], cal["sy/m"]] + [cal["cx"], cal["cy"]]
    audit = json.loads((OUT / "results.json").read_text(encoding="utf-8"))
    samples = {s["frame"]: s for s in audit["samples"]}
    checks = []
    matched_inputs = []
    hashes = {}
    for p in (ROOT / "output/online_local_registration/20261021_121400_904_session_1").glob("local_icp*.json"):
        j = json.loads(p.read_text(encoding="utf-8-sig"))
        for item in j.get("raw_frames", []):
            f = int(item["frame_id"])
            if f not in samples:
                continue
            r = by_id[f]
            real = np.array(item["independent"]["real_weld_parent_mm"])
            virt = np.array(item["independent"]["virtual_weld_parent_mm"])
            pixels = to_pixels(item["raw_virtual_parent_mm"], r)
            weld_pixel = to_pixels([virt], r)[0]
            record = {"frame": f, "archive": p.name, "virtual_line_pixels": pixels.tolist(), "virtual_weld_reprojected": weld_pixel.tolist(), "virtual_weld_recorded": [float(r["sim_x_px"]), float(r["sim_y_px"])], "virtual_weld_reprojection_error_px": float(np.linalg.norm(weld_pixel - [float(r["sim_x_px"]), float(r["sim_y_px"])])), "raw_projection_error_px": float(np.linalg.norm(to_pixels([real], r)[0] - [float(r["before_fusion_x_px"]), float(r["before_fusion_y_px"])]))}
            matched_inputs.append(record)
            hashes[str(p.relative_to(ROOT))] = sha(p)
            ss = samples[f]
            if not ss["eligible"]:
                continue
            rr, tt = context(r)
            fold = rr @ camera_point(ss["corner_x"], ss["corner_y"], cal) + tt
            trans = np.array(j["virtual_to_real_row_major"]).reshape(4, 4)
            corrected = trans[:3, :3] @ virt + trans[:3, 3]
            rbef, raft = np.linalg.norm(real - virt), np.linalg.norm(real - corrected)
            fbef, faft = np.linalg.norm(fold - virt), np.linalg.norm(fold - corrected)
            limit = j["options"]["maximum_independent_worsening_mm"]
            checks.append({"frame": f, "candidate_frame": int(j["frame_id"]), "raw_before_mm": float(rbef), "raw_after_mm": float(raft), "raw_change_mm": float(raft - rbef), "fold_before_mm": float(fbef), "fold_after_mm": float(faft), "fold_change_mm": float(faft - fbef), "raw_worsening_gate_pass": bool(raft <= rbef + limit), "fold_worsening_gate_pass": bool(faft <= fbef + limit), "actual_candidate_accepted": j["accepted_before_archive"], "reason": j["reason"]})
    result = {"station_transform_fit_max_mm": float(station_residual.max()), "tcp_offset_range_mm": np.ptp(offsets, axis=0).tolist(), "projection_reconstruction_max_mm": max(errors), "projection_reconstruction_median_mm": float(np.median(errors)), "matched_frames": matched_inputs, "single_frame_candidate_checks": checks, "limits": "Fold checks are counterfactual single-frame diagnostics, not full-window replay or proof of seam truth."}
    (OUT / "geometry_results.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    (OUT / "geometry_input_hashes.json").write_text(json.dumps(hashes, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "matched_frames"}, ensure_ascii=False, indent=2))
    print("MATCHED", [(x["frame"], x["virtual_weld_reprojection_error_px"]) for x in matched_inputs])


if __name__ == "__main__":
    main()
