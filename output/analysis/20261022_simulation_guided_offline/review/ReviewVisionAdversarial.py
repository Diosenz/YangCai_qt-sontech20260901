"""独立合成反例：真实条纹与仿真形状不完全一致，较弱反光恰好符合仿真。

科学合成测试数据，不修改用户图像。只输出JSON，不写生产文件。
退出1表示原型错误接受了反光，作为明确的RED证据保留。
"""
from pathlib import Path
import hashlib
import json
import runpy
import sys

import numpy as np


def main():
    source = Path(__file__).resolve().parents[1] / "vision" / "validate_candidates.py"
    api = runpy.run_path(str(source))
    model_x = np.arange(30, 211)
    model_y = 140 + np.where(model_x < 120,
                             .35 * (model_x - 120), -.30 * (model_x - 120))
    yy, _ = np.indices((241, 281))
    columns = np.arange(281)
    actual_apex = np.array([138.0, 134.0])
    reflection_apex = np.array([120.0, 160.0])
    actual_y = 134 + np.where(columns < 138, .45 * (columns - 138),
                             -.45 * (columns - 138))
    reflection_y = 160 + np.where(columns < 120, .35 * (columns - 120),
                                 -.30 * (columns - 120))
    actual_signal = 215 * np.exp(-.5 * ((yy - actual_y) / 1.3) ** 2)
    reflection_signal = 180 * np.exp(-.5 * ((yy - reflection_y) / 1.3) ** 2)
    image = np.maximum(14 + actual_signal, 14 + reflection_signal)
    image = np.clip(image, 0, 255).astype("uint8")
    result = api["profile_match"](image, model_x, model_y, [120, 140])
    physical_error = None if result["point"] is None else float(
        np.linalg.norm(np.asarray(result["point"]) - actual_apex))
    false_accept = bool(result["accepted"] and physical_error > 10)
    evidence = {
        "evidence_kind": "synthetic_independently_specified_physical_seam",
        "prototype_source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "actual_seam_apex_px": actual_apex.tolist(),
        "actual_profile_slopes": [.45, -.45],
        "actual_signal_peak_above_background": 215,
        "reflection_apex_px": reflection_apex.tolist(),
        "reflection_profile_slopes": [.35, -.30],
        "reflection_signal_peak_above_background": 180,
        "nominal_model_apex_px": [120, 140],
        "nominal_model_slopes": [.35, -.30],
        "sigma_px": 1.3,
        "result": result,
        "physical_seam_error_px": physical_error,
        "false_accept": false_accept,
        "expected_behavior": "真实线和反光线同时有支撑，模型形状偏好不能单独确认真实身份；应保留歧义或引入额外跨帧/空间证据。",
        "scope": "不是现场精度数据；证明当前固定形状匹配原型在一个生成条件下错误接受，不能据此否定两链解耦方向。",
        "production_modified": False,
        "device_calls": False,
        "mouse_used": False
    }
    print(json.dumps(evidence, ensure_ascii=False, indent=2))
    return 1 if false_accept else 0


if __name__ == "__main__":
    sys.exit(main())
