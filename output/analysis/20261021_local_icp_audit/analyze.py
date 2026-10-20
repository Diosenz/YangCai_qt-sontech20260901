"""Audit saved online runs without changing inputs or replaying production ICP.

Run with the bundled Python runtime (numpy and pandas).
The pixel backprojection comparison is a weld-point proxy, NOT a surface-ICP residual.
"""
from pathlib import Path
from collections import Counter
import hashlib
import json
import numpy as np
import pandas as pd

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
HASHES = {}


def read_bytes(path):
    data = path.read_bytes()
    HASHES[str(path.relative_to(ROOT))] = hashlib.sha256(data).hexdigest()
    return data


def table(path):
    read_bytes(path)
    return pd.read_csv(path, sep="\t", low_memory=False)


def stats(values):
    values = np.asarray(values, dtype=float)
    return dict(n=len(values), minimum=float(values.min()),
                median=float(np.median(values)), p95=float(np.quantile(values, .95)),
                maximum=float(values.max()))


def rotation(angles):
    x, y, z = np.deg2rad(angles)
    cx, sx, cy, sy, cz, sz = np.cos(x), np.sin(x), np.cos(y), np.sin(y), np.cos(z), np.sin(z)
    return np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                     [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
                     [-sy, cy*sx, cy*cx]])


sensor = {}
for line in read_bytes(ROOT / "config/calibration/SENSORPARA.txt").decode("utf-8-sig").splitlines():
    fields = line.split()
    if len(fields) == 2:
        try:
            sensor[fields[0]] = float(fields[1])
        except ValueError:
            pass


def project_camera(x, y):
    uv = (np.column_stack([x, y]) - [sensor["cx"], sensor["cy"]]) * [sensor["sx/m"], sensor["sy/m"]]
    uv /= 1 + sensor["Kappa"] * (uv * uv).sum(axis=1)[:, None]
    ray = np.column_stack([uv / sensor["f/m"], np.ones(len(uv))])
    plane = np.array([sensor["a"], sensor["b"], sensor["c"]])
    return -1000 / (ray @ plane)[:, None] * ray


caltext = read_bytes(ROOT / "config/calibration/RBCALIB.DAT").decode("utf-8-sig")
cal = np.array([float(v) for line in caltext.splitlines() if line.startswith("////SRANG ")
                for v in line.split(" ", 1)[1].split(",")])
# Station transform uses inverse Euler order, matching the existing projection contract.
sr = rotation([-cal[3], 0, 0]) @ rotation([0, -cal[4], 0]) @ rotation([0, 0, -cal[5]])
cr = rotation([sensor[f"pose_CinT[{i}]/deg"] for i in (3, 4, 5)])
ct = np.array([sensor[f"pose_CinT[{i}]/m"] * 1000 for i in (0, 1, 2)])
seen = {}
results = []
duplicates = []
for session in sorted((ROOT / "output/online_correction").glob("20261020*")):
    b = table(session / "online_icp_batches.txt")
    d = table(session / "vision_diagnostics.tsv")
    signature = tuple(HASHES[str((session / f).relative_to(ROOT))]
                      for f in ("online_icp_batches.txt", "vision_diagnostics.tsv"))
    if signature in seen:
        duplicates.append(dict(session=session.name, same_batch_and_vision_data_as=seen[signature]))
        continue
    seen[signature] = session.name
    summary_table = table(session / "session_summary.txt")
    summary = dict(zip(summary_table.key, summary_table.value))
    applied = b[b.applied == 1]
    valid = d[(d.phase == "online_running") & (d.result_valid == 1) & (d.tracker_completed == 1)].copy()
    raw = project_camera(valid.before_fusion_x_px, valid.before_fusion_y_px)
    sim = project_camera(valid.sim_x_px, valid.sim_y_px)
    final = project_camera(valid.final_x_px, valid.final_y_px)
    distance = np.linalg.norm(raw - sim, axis=1)
    # Independently verify the reconstruction against saved production 3D coordinates.
    tool = final @ cr.T + ct
    robot = np.array([rotation([row.robot_rx_deg, row.robot_ry_deg, row.robot_rz_deg]) @ p
                      + [row.robot_x_mm, row.robot_y_mm, row.robot_z_mm]
                      for row, p in zip(valid.itertuples(), tool)])
    station = (robot - cal[:3]) @ sr.T
    logged = valid[[f"station_base_{a}_mm" for a in "xyz"]].to_numpy()
    projection_error = float(np.linalg.norm(station - logged, axis=1).max())
    assert projection_error < 2e-6, projection_error
    sectors = []
    for low in range(0, 360, 60):
        mask = (valid.s2_deg.to_numpy() % 360 >= low) & (valid.s2_deg.to_numpy() % 360 < low + 60)
        if mask.any():
            sectors.append(dict(s2_degrees=[low, low + 60], proxy_distance_mm=stats(distance[mask])))
    results.append(dict(
        session=session.name, start_time=summary.get("start_time"),
        recorded_trajectory_icp_enabled=summary.get("enable_online_icp"),
        local_surface_icp_runtime_options_recorded=False,
        trajectory_icp_batches=len(b), trajectory_icp_applied=len(applied),
        trajectory_icp_messages=dict(Counter(b.message)),
        trajectory_icp_applied_rms_mm=stats(applied.rms_mm),
        weld_point_proxy_raw_mm=stats(distance),
        weld_point_proxy_final_mm=stats(np.linalg.norm(final - sim, axis=1)),
        weld_point_proxy_fraction_above_5mm=float(np.mean(distance > 5)),
        weld_point_proxy_sectors=sectors,
        projection_verification_max_error_mm=projection_error,
        online_vision_processing_ms=stats(valid.processing_finished_ms-valid.processing_started_ms),
        local_icp_residuals_and_clouds_available=False,
    ))

read_bytes(ROOT / "config/digital_twin/online_local_registration.ini")
unchanged = all(hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == digest
                for path, digest in HASHES.items())
assert unchanged
report = dict(
    scope="2026-10-20 saved online correction sessions; duplicate batch/vision data excluded",
    limitations=["Saved ICP batches belong to trajectory ICP, not workpiece surface ICP.",
                 "Current calibration reproduces logged production projection; no per-session calibration snapshot is saved.",
                 "Weld-point pixel backprojection is only a proxy; not ICP residual, surface overlap, or estimated workpiece pose.",
                 "No local surface cloud pairs, rejection metrics, live thresholds or SET_POSE confirmations were found in session files."],
    sessions=results, duplicates=duplicates, input_sha256=HASHES,
    source_hashes_unchanged=unchanged,
)
(OUT / "results.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
for result in results:
    print(result["session"], "trajectory ICP", result["trajectory_icp_applied"], "/", result["trajectory_icp_batches"],
          "proxy median/p95 mm", result["weld_point_proxy_raw_mm"]["median"], result["weld_point_proxy_raw_mm"]["p95"],
          "projection error mm", result["projection_verification_max_error_mm"])
print("Source hashes unchanged:", unchanged)
