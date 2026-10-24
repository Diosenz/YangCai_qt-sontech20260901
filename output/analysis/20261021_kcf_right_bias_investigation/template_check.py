"""Fresh-template controls using archived virtual 3D lines reprojected to image.

Only the first frame with a fresh template and no prior history is comparable
to production directly. All later frames are deliberately reset controls.
"""
import json
import numpy as np
from PIL import Image
from analyze import OUT, RUN
from native_cv import lines
from reference_kcf import fhog, patch, response


def main():
    geo = json.loads((OUT / "geometry_results.json").read_text(encoding="utf-8"))
    result = json.loads((OUT / "results.json").read_text(encoding="utf-8"))
    samples = {x["frame"]: x for x in result["samples"]}
    checks = []
    for g in geo["matched_frames"]:
        ss = samples[g["frame"]]
        sim = (ss["sim_x"], ss["sim_y"])
        im = np.asarray(Image.open(RUN / ss["enhanced"]).convert("L"), dtype=np.float32)
        pts = np.asarray(g["virtual_line_pixels"])
        template = lines(pts)
        model = fhog(patch(template, sim))
        fresh = response(model, fhog(patch(im, sim)))
        actual = np.array([ss["raw_x"], ss["raw_y"]])
        pred = np.array(sim) + [fresh["dx"], fresh["dy"]]
        check = {"frame": ss["frame"], "since_init": ss["since_init"], "fresh_point": pred.tolist(), "production_point": actual.tolist(), "difference_px": float(np.linalg.norm(pred - actual)), "fresh_response": fresh, "production_peak": ss["peak"], "input_kind": "reprojected archived downsampled virtual line, fresh model"}
        # Replace virtual shape with an ideal V fitted directly on the real image,
        # preserving the same center, then isolate shape vs location.
        if ss["eligible"]:
            ref = ss["fits"]["source_image"][1]
            x = np.arange(0, im.shape[1])
            y = sim[1] + np.where(x < sim[0], ref["left_slope"], ref["right_slope"]) * (x - sim[0])
            fitted_template = lines(np.column_stack([x, y]))
            control = response(fhog(patch(fitted_template, sim)), fhog(patch(im, sim)))
            check["real_angle_template_fresh_point"] = (np.array(sim) + [control["dx"], control["dy"]]).tolist()
            check["reference_fold"] = [ref["x"], ref["y"]]
        checks.append(check)
    output = {"limitations": "Sparse downsampled virtual geometry, independent FHOG model, no native temporal replay.", "checks": checks}
    (OUT / "template_results.json").write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(output, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
