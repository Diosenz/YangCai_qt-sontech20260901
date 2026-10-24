"""Recompute captured surface clouds with pinned production validation code."""
import hashlib
import argparse
import json
import pathlib
import shlex
import subprocess
import sys
import time

from replay import materialize, OUT, ROOT, REVISION


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--coverage", type=float)
    parser.add_argument("--minimized", action="store_true")
    args = parser.parse_args()
    if sys.platform != "darwin":
        raise RuntimeError("macOS auxiliary harness; production validation belongs in VS2022")
    sources = ["Localization/include/LocalizationTypes.h",
               "Localization/include/LocalizationSurfaceValidation.h",
               "Localization/src/LocalizationSurfaceValidation.cpp",
               "UI/include/OnlineLocalSurfaceVerification.h"]
    base = ("output/online_local_registration/20261023_114602_096_session_1/"
            "local_icp_1_10466_f67ff4bc-f211-4cea-af37-febb45a73aef.json")
    hashes = {}
    for relative in sources + [base, base + ".retained.json"]:
        _, hashes[relative] = materialize(relative)
    harness = pathlib.Path(__file__).with_name("SurfaceReplay.cpp")
    flags = shlex.split(subprocess.check_output(["pkg-config", "--cflags", "--libs", "Qt6Core"], text=True))
    eigen = subprocess.check_output(["pkg-config", "--cflags", "eigen3"], text=True)
    flags += shlex.split(eigen)
    key = hashlib.sha256(harness.read_bytes() + json.dumps(hashes, sort_keys=True).encode()
                         + repr(flags).encode()).hexdigest()[:16]
    binary = OUT / f"SurfaceReplay-{key}"
    if not binary.exists():
        subprocess.run(["clang++", "-std=c++17", "-O2", "-I", str(OUT / "snapshot/Localization/include"),
                        "-I", str(OUT / "snapshot/UI/include"), str(harness),
                        str(OUT / "snapshot/Localization/src/LocalizationSurfaceValidation.cpp"),
                        "-o", str(binary), *flags], cwd=ROOT, check=True)
    started = time.perf_counter()
    command = [str(binary), str(OUT / "snapshot" / base),
               str(OUT / "snapshot" / (base + ".retained.json"))]
    if args.coverage is not None or args.minimized:
        command.append(str(args.coverage if args.coverage is not None else 0.7))
    if args.minimized:
        command.append("minimized")
    result = subprocess.run(command,
                            capture_output=True, text=True, cwd=ROOT)
    elapsed = time.perf_counter() - started
    print(result.stdout, end="")
    print(result.stderr, file=sys.stderr, end="")
    print(f"replay_seconds={elapsed:.4f} exit={result.returncode}")
    record = {"revision": REVISION, "hashes": hashes, "stdout": result.stdout,
              "coverage_probe": args.coverage, "minimized": args.minimized,
              "stderr": result.stderr, "returncode": result.returncode, "replay_seconds": elapsed,
              "scope": "Actual surface evaluate/measure/compare and posterior checker; no ICP solve or RobotSim SET_POSE."}
    (OUT / f"surface_{time.time_ns()}.json").write_text(
        json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
