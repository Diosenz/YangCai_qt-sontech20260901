"""Pinned production-source replay; prebuilt native programs or auxiliary macOS compiler."""
import argparse
import hashlib
import json
import pathlib
import shlex
import subprocess
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "online_detector_replay"))
from native_runtime import configured_native

ROOT = pathlib.Path(__file__).resolve().parents[3]
REVISION = "430aaa204fd5319dd7f697d3de2b4e1faddbb06f"
OUT = ROOT / "output/analysis/online_correction_repro_20261024"
SOURCES = [
    "Vision/include/feature/OnlineSeamRefiner.h",
    "Vision/src/feature/OnlineSeamRefiner.cpp",
    "Vision/include/stripe/SubpixelCenterlineTools.h",
    "Vision/src/stripe/SubpixelCenterlineTools.cpp",
    "Vision/include/common/VisionImageTypes.h",
    "Vision/include/stripe/LaserStripeExtractor.h",
]
CASES = {
    "field": (
        "output/online_local_registration/20261023_114602_096_session_1/"
        "local_icp_1_10726_fb7e0934-3e27-469c-9bf8-b09fdc477c05.json.source.png",
        443.790314, 609.482733,
    ),
    "anchor": ("tests/fixtures/online_image_geometry/frame_14591.png", 435.5, 608.562989),
}


def materialize(relative):
    """Retain an exact Git blob, never silently replace an existing artifact."""
    data = subprocess.check_output(["git", "show", f"{REVISION}:{relative}"], cwd=ROOT)
    target = OUT / "snapshot" / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        existing = target.read_bytes()
        # Git on Windows may check out text snapshots with CRLF. Keep binary
        # artifacts byte-exact and reject every text change except line endings.
        if target.suffix in {".h", ".cpp", ".json", ".txt", ".cmake"}:
            existing = existing.replace(b"\r\n", b"\n")
            expected = data.replace(b"\r\n", b"\n")
        else:
            expected = data
        if existing != expected:
            raise RuntimeError(f"existing snapshot differs: {target}")
    else:
        target.write_bytes(data)
    return target, hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, default="field")
    parser.add_argument("--minimized", action="store_true")
    parser.add_argument("--ambiguity-limit", type=float)
    parser.add_argument("--half-height", type=int)
    parser.add_argument("--hint-dy", type=float, default=0.0)
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    hashes = {}
    for source in SOURCES:
        _, hashes[source] = materialize(source)
    image_path, x, y = CASES[args.case]
    y += args.hint_dy
    fixture, hashes[image_path] = materialize(image_path)
    harness = pathlib.Path(__file__).with_name("RefinerReplay.cpp")
    native = configured_native("RefinerReplay", {relative: hashlib.sha256(
        (OUT / "snapshot" / relative).read_bytes().replace(b"\r\n", b"\n")).hexdigest() for relative in SOURCES})
    if native:
        binary, provenance = native
        version = json.dumps(provenance["dependency_versions"])
    else:
        version = subprocess.check_output(["pkg-config", "--modversion", "opencv4"], text=True,encoding="utf-8").strip()
        # Only link the exercised modules; the local optional viz/sfm installation is incomplete.
        flags = shlex.split(subprocess.check_output(["pkg-config", "--cflags", "--libs-only-L", "opencv4"], text=True,encoding="utf-8"))
        flags += ["-lopencv_imgcodecs", "-lopencv_imgproc", "-lopencv_core"]
        key = hashlib.sha256(harness.read_bytes() + json.dumps(hashes, sort_keys=True).encode()
                             + version.encode() + repr(flags).encode()).hexdigest()[:16]
        binary = OUT / f"RefinerReplay-{key}"
        command = ["clang++", "-std=c++17", "-O2", "-I", str(OUT / "snapshot/Vision/include"),
                   str(harness), str(OUT / "snapshot/Vision/src/feature/OnlineSeamRefiner.cpp"),
                   str(OUT / "snapshot/Vision/src/stripe/SubpixelCenterlineTools.cpp"),
                   "-o", str(binary), *flags]
        if not binary.exists():
            subprocess.run(command, cwd=ROOT, check=True)
    started = time.perf_counter()
    run_command = [str(binary), str(fixture), str(x), str(y)]
    if args.minimized:
        run_command.append("--crop")
    if args.ambiguity_limit is not None:
        run_command += ["--ambiguity-limit", str(args.ambiguity_limit)]
    if args.half_height is not None:
        run_command += ["--half-height", str(args.half_height)]
    result = subprocess.run(run_command,
                            cwd=ROOT, capture_output=True, text=True,encoding="utf-8")
    elapsed = time.perf_counter() - started
    print(f"revision={REVISION} OpenCV={version} case={args.case}")
    print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, file=sys.stderr, end="")
    print(f"replay_seconds={elapsed:.4f} exit={result.returncode}")
    record = {"revision": REVISION, "opencv": version, "case": args.case,
              "fixture": image_path, "hint": [x, y], "minimized": args.minimized,
              "probe_options": {"ambiguity_limit": args.ambiguity_limit,
                                "half_height": args.half_height, "hint_dy": args.hint_dy},
              "hashes": hashes,
              "stdout": result.stdout, "stderr": result.stderr,
              "returncode": result.returncode, "replay_seconds": elapsed,
              "scope": "Actual production refiner; not full detector, robot, or MSVC replay."}
    log = OUT / f"refiner_{args.case}_{time.time_ns()}.json"
    log.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
