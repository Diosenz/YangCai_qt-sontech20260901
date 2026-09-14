# IBVS Data Layout And Config

This note records the repo-local IBVS data layout used after moving away from
the absolute `C:\Feature_detection` dependency.

## Data Root

All paths below are relative to:

```text
data/IBVS
```

The directory is intentionally local runtime/test data. It is ignored by Git, so
large models and images can live here without entering normal source control.

## Config Files

Tracked default txt files live under:

```text
config/IBVS
```

Local runtime/test data may also carry overrides under:

```text
data/IBVS/config
```

Both locations use the same file names and format:

```text
workpiece_profiles.txt
ibvs_runtime_defaults.txt
```

`workpiece_profiles.txt` centralizes legacy `workpieceType` mappings:

- supported legacy type aliases
- workpiece YOLO model path
- cropped test image directory
- curve and weld sampling txt files
- start-point method hint
- baseline template-alignment pose `rvec0/tvec0`
- legacy safety constraint ranges
- unified LVS/IBVS position constraint ranges used by
  `IbvsUnifiedVelocitySolver`
- optional workpiece-specific feature-geometry overrides

`ibvs_runtime_defaults.txt` centralizes shared IBVS defaults:

- torch YOLO model path
- hand-eye calibration paths
- torch feature crop set
- velocity replay txt files
- YOLO and red-feature detector defaults
- legacy template-matching search parameters
- default feature geometry, sensor-fusion thresholds, LVS weights, unified
  velocity limits, IBVS control parameters, and KF parameters

The production parser is `IbvsTextConfigReader`. It is intentionally separate
from `IbvsSensorFusion` and `IbvsUnifiedVelocitySolver`: the parser performs
file IO and converts txt values into plain algorithm structs, while the solver
and fusion classes remain pure calculation interfaces.

## Current Data Status

The current local data set contains four workpieces:

```text
workpieces/lingjian2
workpieces/penzuicone
workpieces/chufengkouxiao
workpieces/workpiece3_simM
```

Each workpiece has a workpiece ONNX model, crop-image directory, curve sample,
and weld sample. Only `lingjian2` currently carries an `init_sample.txt`; this
is acceptable for the current phase because init samples are not required.

Torch feature testing uses:

```text
models/torch_yolo/best.onnx
torch_feature/crops/set_001
```

The test images are YOLO-cropped images. This means the current start-point
tests should treat them as post-ROI images and should not require another
workpiece YOLO crop step.

## Legacy Hardcoding Candidates

The following legacy hardcoded values should be treated as config-owned before
they are migrated into new production code:

- `PicCapture.cpp`: `workpieceType` to workpiece name/key mapping.
- `PicCapture.cpp`: `workpieceType` to weld sample, init sample, and workpiece
  ONNX path mapping.
- `PicCapture.cpp`: per-workpiece safety constraint ranges.
- `PicCapture.cpp`: hand-eye and ROI hand-eye calibration filenames.
- `PicCapture.cpp`: YOLO defaults and red-feature detector defaults.
- `InitPointDetector.cpp`: baseline `rvec0/tvec0` and curve-file selection.
- `InitPointDetector.cpp`: template-matching search parameters.

The current new `IbvsStartPointDetector` only contains pure geometry helpers.
The full legacy image chain for real start-point recognition has not yet been
migrated into the new module.

## Start-Point Image Chain

The first repo-local image-chain migration is
`IbvsStartPointImageDetector`. It intentionally works on cropped workpiece
images under `workpieces/<id>/images` and does not run YOLO. The detector
uses the configured 3D `curve_sample.txt` as the CAD/template input for
DT-Chamfer alignment. After alignment succeeds, it projects `weld_sample.txt`
with the optimized pose and uses that projection as the weld/second-edge prior.
This matches the legacy split between whole-workpiece template alignment and
local weld extraction.

Because the current crop images do not carry their original full-image ROI
rectangles, this stage verifies only crop-local behavior. It writes template
overlays, edge images, weld-point overlays, and start-point overlays so the
coordinate assumption can be checked visually before any full-image or robot
coordinate write-back is introduced.

Each tested image writes a numbered visual trace under
`output/ibvs_offline_validation/i2r_rlc_ibvs_start_point_image_chain_tests/<timestamp>/<workpiece>/<frame>/`:

- `01_edges_binary.png`: binary edge map used by the downstream chain.
- `02_edges_contours.jpg`: extracted edge contours over the crop image.
- `03_template_initial_overlay.jpg`: baseline `curve_sample` projection before matching.
- `04_template_aligned_overlay.jpg`: aligned `curve_sample` projection after DT-Chamfer matching.
- `05_distance_map.png`: distance-transform cost field used to understand template matching.
- `06_weld_prior.jpg`: optimized-pose `weld_sample` projection before edge extraction.
- `07_weld_subpixel.jpg`: accepted weld/second-edge points after extraction.
- `08_start_point.jpg`: final start/end point overlay.
- `process_overview.jpg`: one contact sheet containing the full sequence above.

The same folder also keeps CSV point dumps:
`data_curve_template_initial_points.csv`,
`data_curve_template_aligned_points.csv`, `data_weld_prior_points.csv`,
`data_second_edge_prior_points.csv`, `data_weld_points.csv`, and
`data_second_edge_points.csv`.
