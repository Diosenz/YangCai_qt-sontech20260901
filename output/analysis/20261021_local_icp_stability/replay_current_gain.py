"""Read archived candidate windows; no ICP rerun or hypothetical trajectory replay."""
import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
SESSIONS = ["20261021_072328_003_session_1", "20261021_083204_186_session_3"]

rows = []
for session in SESSIONS:
    for written in (ROOT / "online_local_registration" / session).glob("*.json.writeback.json"):
        source = Path(str(written).removesuffix(".writeback.json"))
        data = json.loads(source.read_text(encoding="utf-8-sig"))
        matrix = data["virtual_to_real_row_major"]
        frame_errors = []
        for frame in data["raw_frames"]:
            observation = frame["independent"]
            assert observation["valid"]
            real = observation["real_weld_parent_mm"]
            virtual = observation["virtual_weld_parent_mm"]
            corrected = [sum(matrix[axis * 4 + col] * virtual[col] for col in range(3))
                         + matrix[axis * 4 + 3] for axis in range(3)]
            frame_errors.append({"frame": int(frame["frame_id"]),
                                 "before": math.dist(real, virtual),
                                 "after": math.dist(real, corrected)})
        before = sum(frame["before"] for frame in frame_errors) / len(frame_errors)
        after = sum(frame["after"] for frame in frame_errors) / len(frame_errors)
        stable = before <= 2.0 and data["before_residual_mm"] <= 2.0
        minimum_gain = max(0.2 if stable else 0.1, 0.05 * before)
        worsening_limit = data["options"]["maximum_independent_worsening_mm"]
        ceiling = data["options"]["maximum_independent_error_mm"]
        reasons = []
        if any(frame["after"] > frame["before"] + worsening_limit + 1e-6
               for frame in frame_errors):
            reasons.append("current_worsening_exceeds_original_tolerance")
        if any(frame["after"] > ceiling for frame in frame_errors):
            reasons.append("current_error_exceeds_original_ceiling")
        if before - after + 1e-6 < minimum_gain:
            reasons.append("mean_gain_insufficient")
        # Cross-check independently computed errors against the archived worker maxima.
        assert abs(max(frame["before"] for frame in frame_errors)
                   - data["before_independent_error_mm"]) < 1e-8
        assert abs(max(frame["after"] for frame in frame_errors)
                   - data["after_independent_error_mm"]) < 1e-8
        rows.append({"session": session, "frame": int(data["frame_id"]),
                     "before_surface_mm": data["before_residual_mm"],
                     "after_surface_mm": data["after_residual_mm"],
                     "before_mean_mm": before, "after_mean_mm": after,
                     "gain_mm": before - after, "minimum_gain_mm": minimum_gain,
                     "stable_region": stable, "original_worsening_limit_mm": worsening_limit,
                     "maximum_frame_worsening_mm": max(frame["after"] - frame["before"]
                                                        for frame in frame_errors),
                     "accepted_by_current_gain_gate": not reasons, "reasons": reasons,
                     "source": str(source), "frame_errors": frame_errors})

rows.sort(key=lambda row: (row["session"], row["frame"]))
report = {"scope": "Counter-check each originally written candidate on its own frozen window. "
                   "Does not resimulate subsequent windows after rejecting an earlier update; "
                   "historical-anchor veto is not included because old verified archives lack coordinates.",
          "candidate_count": len(rows), "rows": rows}
(OUT / "gain_gate_replay.json").write_text(json.dumps(report, ensure_ascii=False, indent=2),
                                          encoding="utf-8")
fields = [key for key in rows[0] if key not in ("frame_errors", "reasons")] + ["reasons"]
with (OUT / "gain_gate_replay.csv").open("w", encoding="utf-8-sig", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    for row in rows:
        writer.writerow({**{key: row[key] for key in fields}, "reasons": ";".join(row["reasons"])})
for session in SESSIONS:
    subset = [row for row in rows if row["session"] == session]
    rejected = [row["frame"] for row in subset if not row["accepted_by_current_gain_gate"]]
    print(session, "written", len(subset), "retained", len(subset) - len(rejected),
          "rejected", rejected)
for frame in (5991, 11188, 11846, 21613):
    row = next(row for row in rows if row["frame"] == frame)
    print(frame, {key: value for key, value in row.items() if key not in ("source", "frame_errors")})
