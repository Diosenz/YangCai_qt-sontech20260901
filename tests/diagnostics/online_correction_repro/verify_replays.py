"""Check reproducibility and single-variable probes; baseline symptoms stay RED."""
import json
import pathlib
import subprocess
import sys
import time

from replay import OUT, REVISION, materialize


def main():
    here = pathlib.Path(__file__).resolve().parent
    cases = [
        ("field_repeat_1", "replay.py", ["--case", "field"], 1, "status=2"),
        ("field_repeat_2", "replay.py", ["--case", "field"], 1, "ambiguous_ratio=0.497382"),
        ("field_repeat_3", "replay.py", ["--case", "field"], 1, "ambiguous_ratio=0.497382"),
        ("field_minimized", "replay.py", ["--case", "field", "--minimized"], 1, "input=201x121"),
        ("accepted_control", "replay.py", ["--case", "anchor"], 0, "valid=1 status=6"),
        ("ambiguity_probe", "replay.py", ["--case", "field", "--ambiguity-limit", "0.6"], 0, "point=(445.500000,607.745053)"),
        ("height_probe", "replay.py", ["--case", "field", "--half-height", "40"], 1, "status=2"),
        ("hint_up_probe", "replay.py", ["--case", "field", "--hint-dy", "-5"], 1, "status=2"),
        ("hint_down_probe", "replay.py", ["--case", "field", "--hint-dy", "5"], 1, "status=2"),
        ("surface_replay", "surface_replay.py", [], 1, "numerical_metrics_match_archive=1"),
        ("surface_minimized", "surface_replay.py", ["--minimized"], 1, "decision=2"),
        ("coverage_probe", "surface_replay.py", ["--coverage", "0.4"], 0, "written_pose_geometry_verified=1"),
    ]
    records = []
    for name, script, options, expected, signal in cases:
        command = [sys.executable, str(here / script), *options]
        result = subprocess.run(command, capture_output=True, text=True)
        passed = result.returncode == expected and signal in result.stdout
        print(f"{'PASS' if passed else 'FAIL'} {name} exit={result.returncode} expected={expected}")
        records.append({"name": name, "command": command, "expected_exit": expected,
                        "exit": result.returncode, "passed": passed,
                        "stdout": result.stdout, "stderr": result.stderr})
    full_trace, trace_sha = materialize(
        "output/online_correction/20261023_194602_142/vision_diagnostics.tsv")
    import csv
    with full_trace.open(encoding="utf-8-sig") as source:
        frame = next(row for row in csv.DictReader(source, delimiter="\t") if row["frame_id"] == "10689")
    (OUT / "frame10689.json").write_text(json.dumps(
        {"revision": REVISION, "trace_sha256": trace_sha, "record": frame},
        ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    record = {"revision": REVISION, "all_checks_passed": all(r["passed"] for r in records),
              "meaning": "Probe assertions passed; production baseline symptoms remain unfixed and RED.",
              "checks": records}
    destination = OUT / f"verification_{time.time_ns()}.json"
    destination.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{sum(r['passed'] for r in records)}/{len(records)} checks; production baselines remain RED")
    print(destination)
    return 0 if record["all_checks_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
