"""独立解析反例：不读取设备，不修改生产源码，不把合成结果当现场精度。

运行只输出 JSON；将 stdout 保存到新文件由调用者显式负责。
"""
import json

import numpy as np


def main():
    # 名义接缝 z=0.5*x+10，固定扫描平面 x=0；工件整体平移(4,1)。
    slope, intercept = 0.5, 10.0
    shift = np.array([4.0, 1.0])
    before = np.array([0.0, intercept])
    actual_after = np.array([0.0, intercept + shift[1] - slope * shift[0]])
    transformed_before = before + shift
    inverse_current = actual_after - shift
    assert abs(actual_after[0]) < 1e-12
    assert abs(transformed_before[0]) > 1.0
    assert np.linalg.norm(inverse_current - before) > 1.0

    # XYZ+Rz 的表面法向线性约束；绕Z微转的导数为(-y,x,0)。
    def row(p, normal):
        return [normal[0], normal[1], normal[2],
                -normal[0] * p[1] + normal[1] * p[0]]

    grid = [(float(x), float(y), 0.0)
            for x in range(-10, 11) for y in range(-10, 11)]
    plane_j = np.array([row(p, (0.0, 0.0, 1.0)) for p in grid])
    # 两侧V面沿Y方向拉伸，法向没有Y分量，沿缝Y平移无法由此几何决定。
    v_rows = []
    for side in [-1.0, 1.0]:
        n = np.array([side * 0.5, 0.0, 1.0])
        n /= np.linalg.norm(n)
        for x in range(1, 11):
            for y in range(-10, 11):
                p = (side * x, float(y), -0.5 * x)
                v_rows.append(row(p, n))
    v_j = np.array(v_rows)
    assert np.linalg.matrix_rank(plane_j) == 1
    assert np.linalg.matrix_rank(v_j) == 3
    assert np.allclose(v_j @ np.array([0.0, 1.0, 0.0, 0.0]), 0.0)

    # 仿真辅助目标若被直接拿来验证仿真，将始终报告零误差。
    physical_seam = np.array([0.0, 10.0])
    wrong_simulation = np.array([0.0, 15.0])
    inferred_from_model_only = wrong_simulation.copy()
    self_error = float(np.linalg.norm(inferred_from_model_only - wrong_simulation))
    physical_error = float(np.linalg.norm(inferred_from_model_only - physical_seam))
    assert self_error == 0.0 and physical_error == 5.0

    # 位姿已吸收的全局偏差，不能再次作为局部偏差累加。
    original_sim, true_seam, old_bias = 0.0, 5.0, 5.0
    corrected_sim = 5.0
    naive_target = corrected_sim + old_bias
    rebased_bias = true_seam - corrected_sim
    rebased_target = corrected_sim + rebased_bias
    assert naive_target - true_seam == 5.0 and rebased_target == true_seam

    result = {
        "evidence_kind": "analytic_synthetic_counterexamples_not_field_replay",
        "units": "geometry examples mm; Jacobian Rz column uses radians",
        "laser_intersection_is_not_material_point": {
            "nominal_seam_equation": "z=0.5*x+10; laser_plane x=0",
            "workpiece_translation_x_z": shift.tolist(),
            "old_intersection_x_z": before.tolist(),
            "new_intersection_recomputed_x_z": actual_after.tolist(),
            "rigidly_transformed_old_intersection_x_z": transformed_before.tolist(),
            "inverse_transformed_new_intersection_x_z": inverse_current.tolist(),
            "true_old_scene_vs_new_intersection_distance": float(np.linalg.norm(actual_after-before)),
            "inverse_transform_counterfactual_distance": float(np.linalg.norm(actual_after-inverse_current)),
            "conclusion": "变换旧扫描交点或逆变换新交点，不等价于固定激光平面与另一个位姿模型重新求交。"
        },
        "surface_observability": {
            "allowed_dofs": ["Tx", "Ty", "Tz", "Rz"],
            "flat_plane_rank": int(np.linalg.matrix_rank(plane_j)),
            "v_surfaces_extruded_along_seam_rank": int(np.linalg.matrix_rank(v_j)),
            "v_surfaces_null_direction": [0, 1, 0, 0],
            "conclusion": "二维覆盖足够或双侧母材可见，仍不证明四自由度可观测；边界/采样最近邻约束不能直接当真实形状约束。"
        },
        "model_self_validation": {
            "reported_model_consistency_error": self_error,
            "known_synthetic_physical_error": physical_error,
            "conclusion": "仅模型推导的接缝目标不能当作模型位姿正确的独立验收证据。"
        },
        "global_and_local_compensation": {
            "original_simulation": original_sim,
            "true_seam": true_seam,
            "bias_before_pose_update": old_bias,
            "simulation_after_pose_update": corrected_sim,
            "target_if_bias_kept": naive_target,
            "target_after_rebase": rebased_target,
            "conclusion": "补偿必须绑定工件位姿版本；位姿晋升后重估局部残差，不能重复补偿已被全局位姿吸收的偏差。"
        },
        "assertions_passed": 8,
        "production_changes": False,
        "device_calls": False,
        "mouse_used": False
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
