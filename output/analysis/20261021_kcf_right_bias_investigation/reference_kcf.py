"""Independent NumPy diagnostic model of Standard FHOG KCF, not native replay.

Follows Vision/src/track/KcfFhog.cpp and KcfTracker.cpp parameter/layout logic.
The FHOG algorithm derives from OpenCV LatentSVM (2010-2013 University of
Nizhny Novgorod), under its BSD license retained in that source file.
Rendering, interpolation and precision differ from production OpenCV.
Use to test mechanisms on controlled inputs, not to certify native equivalence.
"""
from pathlib import Path
import json
import numpy as np
from PIL import Image
from native_cv import resize

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
SCALE = 375 / 104


def patch(image, center):
    # Production patch is int(375) x int(80 * scale), resized to 104 x 80.
    h, w = 288, 375
    x0, y0 = np.floor(np.asarray(center) - [w / 2, h / 2] + .5).astype(int)
    xs = np.clip(np.arange(x0, x0 + w), 0, image.shape[1] - 1)
    ys = np.clip(np.arange(y0, y0 + h), 0, image.shape[0] - 1)
    im = image[np.ix_(ys, xs)]
    return resize(im, 104, 80)


def fhog(im):
    h, w = im.shape
    dx, dy = np.zeros_like(im), np.zeros_like(im)
    dx[1:-1, 1:-1] = im[1:-1, 2:] - im[1:-1, :-2]
    dy[1:-1, 1:-1] = im[2:, 1:-1] - im[:-2, 1:-1]
    angles = np.arange(9, dtype=np.float32) * np.float32(np.pi / 9)
    dots = dx[..., None] * np.cos(angles) + dy[..., None] * np.sin(angles)
    idx = np.argmax(np.abs(dots), axis=-1)
    signed = idx + 9 * (np.take_along_axis(dots, idx[..., None], axis=-1)[..., 0] < 0)
    mag = np.sqrt(dx * dx + dy * dy)
    yy, xx = np.indices((h, w))
    cy, cx = yy // 4, xx // 4
    ny, nx = np.where(yy % 4 < 2, -1, 1), np.where(xx % 4 < 2, -1, 1)
    weight = np.array([.625, .875, .875, .625], dtype=np.float32)
    wy, wx = weight[yy % 4], weight[xx % 4]
    hist = np.zeros((h // 4, w // 4, 27), dtype=np.float32)
    for oy, ox, factor in [(0, 0, wy * wx), (ny, 0, (1 - wy) * wx), (0, nx, wy * (1 - wx)), (ny, nx, (1 - wy) * (1 - wx))]:
        y, x = cy + oy, cx + ox
        valid = (y >= 0) & (y < h // 4) & (x >= 0) & (x < w // 4)
        for channel in [idx, signed + 9]:
            np.add.at(hist, (y[valid], x[valid], channel[valid]), (mag * factor)[valid])
    norm = np.sum(hist[..., :9] ** 2, axis=-1)
    inner = hist[1:-1, 1:-1]
    norms = []
    for oy, ox in [(1, 1), (-1, 1), (1, -1), (-1, -1)]:
        ys = np.arange(1, h // 4 - 1)
        xs = np.arange(1, w // 4 - 1)
        n = norm[np.ix_(ys, xs)] + norm[np.ix_(ys + oy, xs)] + norm[np.ix_(ys, xs + ox)] + norm[np.ix_(ys + oy, xs + ox)]
        norms.append(np.sqrt(n) + np.finfo(np.float32).eps)
    un = np.stack([np.minimum(inner[..., :9] / n[..., None], .2) for n in norms], axis=2)
    si = np.stack([np.minimum(inner[..., 9:] / n[..., None], .2) for n in norms], axis=2)
    f = np.concatenate([si.sum(axis=2) * .5, un.sum(axis=2) * .5, si.sum(axis=3) / np.sqrt(18)], axis=2)
    hann = np.hanning(f.shape[0])[:, None] * np.hanning(f.shape[1])[None, :]
    return np.moveaxis(f * hann[..., None], -1, 0)


def kernel(a, b):
    corr = np.fft.fftshift(np.fft.ifft2(np.fft.fft2(a) * np.conj(np.fft.fft2(b)), axes=(-2, -1)).real, axes=(-2, -1)).sum(axis=0)
    dist = np.maximum(0, (np.sum(a * a) + np.sum(b * b) - 2 * corr) / a.size)
    return np.exp(-dist / .6 ** 2)


def response(model, observed):
    h, w = model.shape[1:]
    yy, xx = np.indices((h, w))
    sig = np.sqrt(h * w) / 1.5 * .125
    prob = np.fft.fft2(np.exp(-.5 / sig ** 2 * ((xx - w // 2) ** 2 + (yy - h // 2) ** 2)))
    den = np.fft.fft2(kernel(model, model)) + .0001
    alpha = prob * np.conj(den) / (abs(den) ** 2 + 1e-7)
    r = np.fft.ifft2(alpha * np.fft.fft2(kernel(observed, model))).real
    y, x = np.unravel_index(np.argmax(r), r.shape)
    def delta(l, c, right):
        d = 2 * c - l - right
        return .5 * (right - l) / d if abs(d) > 1e-7 else 0
    fx, fy = float(x), float(y)
    if 0 < x < w - 1:
        fx += delta(r[y, x - 1], r[y, x], r[y, x + 1])
    if 0 < y < h - 1:
        fy += delta(r[y - 1, x], r[y, x], r[y + 1, x])
    return {"dx": (fx - w / 2) * 4 * SCALE, "dy": (fy - h / 2) * 4 * SCALE, "peak": float(r[y, x])}


def stripe(center=(300., 250.), slopes=(.3, -.28), width=10, blur=False):
    yy, xx = np.indices((500, 600))
    line_y = center[1] + np.where(xx < center[0], slopes[0], slopes[1]) * (xx - center[0])
    dist = abs(yy - line_y)
    return (255 * np.exp(-.5 * (dist / (width / 2.355)) ** 2) if blur else 255 * (dist <= width / 2)).astype(np.float32)


def run():
    tests = []
    for slopes in [(.3, -.28), (.45, -.28), (.28, -.45)]:
        model = fhog(patch(stripe(slopes=slopes), (300, 250)))
        for kind, image, expected in [("identical", stripe(slopes=slopes), 0), ("translate_right_12", stripe(center=(312, 250), slopes=slopes), 12), ("translate_left_12", stripe(center=(288, 250), slopes=slopes), -12), ("thin_same_fold", stripe(slopes=slopes, width=3), 0), ("gaussian_same_fold", stripe(slopes=slopes, width=3, blur=True), 0)]:
            tests.append({"slopes": slopes, "kind": kind, "expected_dx": expected, **response(model, fhog(patch(image, (300, 250))))})
    d = json.loads((OUT / "results.json").read_text(encoding="utf-8"))
    run_dir = ROOT / d["run"]
    if not run_dir.exists():
        candidates = list((ROOT / "output/online_correction").glob("20261021_201400_950*"))
        assert len(candidates) == 1
        run_dir = candidates[0]
    actual = []
    for item in d["samples"]:
        if not item["eligible"]:
            continue
        ref = item["fits"]["source_image"][1]
        # Deliberately put ideal template fold at measured real fold, isolating
        # appearance and context from model position errors. This is NOT the
        # actual virtual template and does not reproduce production history.
        yy, xx = np.indices((964, 1292))
        line = ref["y"] + np.where(xx < ref["x"], ref["left_slope"], ref["right_slope"]) * (xx - ref["x"])
        template = (255 * (np.abs(yy - line) <= 5)).astype(np.float32)
        center = (ref["x"], ref["y"])
        model = fhog(patch(template, center))
        entry = {"frame": item["frame"], "production_dx_to_fold": item["raw_corner_dx"]}
        for key in ["source", "enhanced"]:
            im = np.asarray(Image.open(run_dir / item[key]).convert("L"), dtype=np.float32)
            entry[key] = response(model, fhog(patch(im, center)))
        # Same-image self match verifies there is no forced positive offset.
        same = fhog(patch(im, center))
        entry["self"] = response(same, same)
        actual.append(entry)
    assert max(abs(t["dx"]) for t in tests if t["kind"] == "identical") < .01
    assert all(t["dx"] > 5 for t in tests if t["kind"] == "translate_right_12")
    assert all(t["dx"] < -5 for t in tests if t["kind"] == "translate_left_12")
    result = {"status": "independent reference model; not native binary equivalence", "synthetic_controls": tests, "real_single_frame_controls": actual}
    (OUT / "reference_kcf_results.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    run()
