"""Analyze the supplied five-control-point experiment using the previous metric definitions."""
from pathlib import Path
import hashlib
import html
import importlib.util
import json
import sys

import numpy as np
import pandas as pd


sys.dont_write_bytecode = True
OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
PREVIOUS = OUT.parent / '20261019_110509_938_station_base'
SESSION = ROOT / 'output/online_correction/20261019_211208_621_采用CFKONTEST3五个P点测试结果'
spec = importlib.util.spec_from_file_location('previous_analysis_math', PREVIOUS / 'analyze.py')
mathlib = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mathlib)


def resolve_recorded_path(value):
    normalized = value.replace('\\', '/')
    for category in ['data/', 'config/']:
        if category in normalized:
            candidate = ROOT / normalized.split(category, 1)[1]
            candidate = ROOT / category / candidate.relative_to(ROOT)
            if candidate.is_file():
                return candidate
    raise FileNotFoundError(value)


summary_frame = pd.read_csv(SESSION / 'session_summary.txt', sep='\t')
summary = dict(zip(summary_frame.key, summary_frame.value))
work_file = resolve_recorded_path(summary['work_path_file'])
teach_file = resolve_recorded_path(summary['teach_pose_file'])
calibration_file = resolve_recorded_path(summary['rbcalib_file'])
inputs = [SESSION / name for name in ['session_summary.txt', 'actual_robot_path.txt',
          'initial_correction.txt', 'realtime_correction.txt']] + [work_file, teach_file, calibration_file]
input_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in inputs}
robot_all = pd.read_csv(SESSION / 'actual_robot_path.txt', sep='\t')
events = pd.read_csv(SESSION / 'realtime_correction.txt', sep='\t')
initial = pd.read_csv(SESSION / 'initial_correction.txt', sep='\t')
work = np.loadtxt(work_file)
teach = np.loadtxt(teach_file)
calibration = np.array([float(value) for line in calibration_file.read_text(
    encoding='utf-8-sig').splitlines() if line.startswith('////SRANG ')
    for value in line.split(' ', 1)[1].split(',')])
assert calibration.shape == (6,)
mount_rotation = mathlib.rx(np.deg2rad(-calibration[3])) @ mathlib.ry(
    np.deg2rad(-calibration[4])) @ mathlib.rz(np.deg2rad(-calibration[5]))
mask = ((robot_all.phase == 'online_running') & (robot_all.corflag == 1)
        & (robot_all.detflag == 1) & (robot_all.s_start == 1)
        & (robot_all.s_hold == 0) & (robot_all.alarm == 0))
robot = robot_all[mask].copy().reset_index(drop=True)
vision = events[(events.event == 'vision_sample') & (events.phase == 'online_running')
                & (events.vision_valid == 1)].copy().reset_index(drop=True)
assert len(robot) > 10 and len(vision) > 10
tcp_fixed = (robot[['x_mm', 'y_mm', 'z_mm']].to_numpy() - calibration[:3]) @ mount_rotation.T
tcp_local = mathlib.external_rotation(tcp_fixed, robot.s1_deg.to_numpy(),
                                      robot.s2_deg.to_numpy(), inverse=True)
kcf_local = vision[['vision_x_mm', 'vision_y_mm', 'vision_z_mm']].to_numpy() + [0., 0., 231.]
states = robot_all.sort_values('timestamp_ms').drop_duplicates('timestamp_ms', keep='last')
vision_s1 = np.interp(vision.timestamp_ms, states.timestamp_ms, states.s1_deg)
vision_s2 = np.rad2deg(np.interp(vision.timestamp_ms, states.timestamp_ms,
                                  np.unwrap(np.deg2rad(states.s2_deg))))
kcf_fixed = mathlib.external_rotation(kcf_local, vision_s1, vision_s2)
assert np.allclose(mathlib.external_rotation(tcp_local, robot.s1_deg.to_numpy(),
                                            robot.s2_deg.to_numpy()), tcp_fixed, atol=1e-9)
for name, frame, local, fixed in [('tcp', robot, tcp_local, tcp_fixed),
                                  ('kcf', vision, kcf_local, kcf_fixed)]:
    output = frame.copy()
    for index, axis in enumerate('xyz'):
        output[f'{axis}_material_frame_mm'] = local[:, index]
        output[f'{axis}_station_base_mm'] = fixed[:, index]
    if name == 'kcf':
        output['s1_interpolated_deg'] = vision_s1
        output['s2_interpolated_deg'] = vision_s2
        output['fixed_frame_pose_is_approximate'] = True
    output.to_csv(OUT / f'{name}_coordinates.csv', index=False, encoding='utf-8-sig')

teach_seed = mathlib.circle(work[:, :3])
kcf_seed = mathlib.circle(kcf_local)
teach_circle = mathlib.circle(mathlib.representatives(work[:, :3], teach_seed['center']))
kcf_representatives = mathlib.representatives(kcf_local, kcf_seed['center'])
kcf_circle = mathlib.circle(kcf_representatives)
alignment = mathlib.circle_pose_alignment(teach_circle, kcf_circle)
relocated = work[:, :3] @ alignment['rotation'].T + alignment['translation']
relocated_circle = dict(center=kcf_circle['center'], radius_mm=teach_circle['radius_mm'],
                        normal=kcf_circle['normal'])
s1, s2 = float(robot.s1_deg.iloc[0]), float(robot.s2_deg.iloc[0])
common_rotation = mathlib.ry(np.deg2rad(s1)) @ mathlib.rz(np.deg2rad(s2))
pd.DataFrame(relocated, columns=['x_mm', 'y_mm', 'z_mm']).to_csv(
    OUT / 'relocated_teach_material_frame.csv', index=False, encoding='utf-8-sig')
pd.DataFrame(relocated @ common_rotation.T, columns=['x_mm', 'y_mm', 'z_mm']).to_csv(
    OUT / 'relocated_teach_common_station_base.csv', index=False, encoding='utf-8-sig')
matrix = np.eye(4)
matrix[:3, :3], matrix[:3, 3] = alignment['rotation'], alignment['translation']
np.savetxt(OUT / 'teach_to_current_material_frame_transform.csv', matrix, delimiter=',', fmt='%.12f')
corrected = initial[initial.event == 'corrected_path_point'].drop_duplicates('point_index')
corrected_xyz = corrected[['aligned_first_x_mm', 'aligned_first_y_mm', 'aligned_first_z_mm']].to_numpy()
assert corrected_xyz.shape == work[:, :3].shape
startup_delta = np.median(corrected_xyz - work[:, :3], axis=0)
startup_shape_residual = float(np.max(np.linalg.norm(
    corrected_xyz - work[:, :3] - startup_delta, axis=1)))
assert startup_shape_residual < 2e-6

metrics = {}
for name, query, reference, times in [
        ('TCP_to_KCF', tcp_local, kcf_local, robot.time),
        ('KCF_to_TCP', kcf_local, tcp_local, vision.time),
        ('TCP_to_original_teach', tcp_local, work[:, :3], robot.time),
        ('KCF_to_original_teach', kcf_local, work[:, :3], vision.time),
        ('TCP_to_relocated_teach', tcp_local, relocated, robot.time),
        ('KCF_to_relocated_teach', kcf_local, relocated, vision.time)]:
    distance, projected, segments, fractions = mathlib.closest_polyline(query, reference)
    maximum = int(distance.argmax())
    metrics[name] = dict(mathlib.metric(distance), max_time=str(times.iloc[maximum]))
    details = pd.DataFrame(dict(time=times, distance_mm=distance,
        reference_segment_index_0based=segments, reference_segment_fraction=fractions))
    for index, axis in enumerate('xyz'):
        details[f'source_{axis}_material_mm'] = query[:, index]
        details[f'projected_{axis}_material_mm'] = projected[:, index]
        details[f'residual_{axis}_material_mm'] = query[:, index] - projected[:, index]
    if name == 'TCP_to_KCF':
        tcp_to_kcf = distance.copy()
        details['nearest_kcf_segment_start_time'] = vision.time.iloc[segments].to_numpy()
        metrics[name]['closest_kcf_record_time_at_max'] = str(vision.time.iloc[segments[maximum]])
    if name == 'TCP_to_relocated_teach':
        tcp_to_moved = distance.copy()
    details.to_csv(OUT / f'{name}_distances.csv', index=False, encoding='utf-8-sig')
for name, query, fitted, times in [
        ('TCP_to_relocated_teach_circle', tcp_local, relocated_circle, robot.time),
        ('KCF_to_relocated_teach_circle', kcf_local, relocated_circle, vision.time),
        ('KCF_to_fitted_circle', kcf_local, kcf_circle, vision.time)]:
    distance = mathlib.circle_distance(query, fitted)
    metrics[name] = dict(mathlib.metric(distance), max_time=str(times.iloc[int(distance.argmax())]))

def coverage(points, center):
    angle = np.rad2deg(np.unwrap(np.arctan2(points[:, 1] - center[1],
                                          points[:, 0] - center[0])))
    return float(np.ptp(angle))

segments = []
for line, indices in robot.groupby('job_line', sort=False).groups.items():
    indices = np.array(list(indices))
    if len(indices) < 10:
        continue
    segments.append(dict(job_line=int(line), count=len(indices),
        start=str(robot.time.iloc[indices[0]]), end=str(robot.time.iloc[indices[-1]]),
        tcp_to_kcf=mathlib.metric(tcp_to_kcf[indices]),
        tcp_to_relocated_teach=mathlib.metric(tcp_to_moved[indices]),
        tool_rpy_start_deg=robot.iloc[indices[0]][['rx_deg', 'ry_deg', 'rz_deg']].to_numpy(float),
        tool_rpy_end_deg=robot.iloc[indices[-1]][['rx_deg', 'ry_deg', 'rz_deg']].to_numpy(float)))
angles = np.sort(np.mod(np.rad2deg(np.arctan2(kcf_local[:, 1] - kcf_circle['center'][1],
                                            kcf_local[:, 0] - kcf_circle['center'][0])), 360))
gaps = np.diff(np.r_[angles, angles[0] + 360])
gap_index = int(gaps.argmax())
max_query = int(tcp_to_kcf.argmax())
max_query_angle = float(np.mod(np.rad2deg(np.arctan2(
    tcp_local[max_query, 1] - kcf_circle['center'][1],
    tcp_local[max_query, 0] - kcf_circle['center'][0])), 360))
closing_distance, *_ = mathlib.closest_polyline(tcp_local, kcf_local[[-1, 0]])
closed_distance = np.minimum(tcp_to_kcf, closing_distance)
gap_diagnostic = dict(largest_unsampled_sector_deg=float(gaps[gap_index]),
    gap_start_deg=float(angles[gap_index]), gap_end_deg=float(angles[gap_index] + gaps[gap_index]),
    max_tcp_to_kcf_query_angle_deg=max_query_angle,
    kcf_endpoint_gap_mm=float(np.linalg.norm(kcf_local[-1] - kcf_local[0])),
    synthetic_closing_segment_affected_tcp_count=int((closed_distance < tcp_to_kcf - 1e-9).sum()),
    synthetic_closed_metric_not_primary=mathlib.metric(closed_distance),
    warning='Synthetic closure sensitivity only; no measured closing segment is added to main distances or curves.')

result = dict(session='20261019_211208_621', experiment_label='采用 CFKONTEST3 五个 P 点测试',
    source_session_directory=str(SESSION.relative_to(ROOT)),
    work_path_file=str(work_file.relative_to(ROOT)), teach_pose_file=str(teach_file.relative_to(ROOT)),
    recorded_windows_paths={key: summary[key] for key in ['work_path_file', 'teach_pose_file', 'rbcalib_file']},
    tcp_count=len(robot), kcf_count=len(vision), teach_work_count=len(work), teach_control_count=len(teach),
    tcp_start=str(robot.time.iloc[0]), tcp_end=str(robot.time.iloc[-1]),
    kcf_start=str(vision.time.iloc[0]), kcf_end=str(vision.time.iloc[-1]),
    raw_session_robot_count=len(robot_all), raw_session_vision_count=int((events.event == 'vision_sample').sum()),
    excluded_wait_stop_tcp_count=int((robot_all.phase == 'waiting_robot_program_stop').sum()),
    common_station_base_pose_deg=[s1, s2], calibration_srangle=calibration,
    original_teach_circle=teach_seed, kcf_circle=kcf_seed,
    teach_fitted_circle_balanced=teach_circle, kcf_fitted_circle_balanced=kcf_circle,
    display_registration=alignment, metrics=metrics,
    teach_original_circle_center_common_station_base_mm=common_rotation @ teach_circle['center'],
    kcf_circle_center_common_station_base_mm=common_rotation @ kcf_circle['center'],
    circle_radius_difference_mm=kcf_circle['radius_mm'] - teach_circle['radius_mm'],
    tcp_coverage_deg=coverage(tcp_local, kcf_circle['center']),
    kcf_coverage_deg=coverage(kcf_local, kcf_circle['center']),
    startup_translation_not_analysis_registration_mm=startup_delta,
    startup_corrected_shape_max_residual_mm=startup_shape_residual,
    job_line_segments=segments, endpoint_gap_diagnostic=gap_diagnostic,
    session_end_state=summary['session_state'], session_end_message=summary['message'],
    metric_definition='3D point-to-recorded-polyline minimum over all segments; sample weighted; all valid online points',
    registration_method='same as morning: 2-degree angular medians, PCA plane, Huber geometric circle; rigid center/normal alignment, scale=1, no yaw fit',
    reference_source='KCF-derived internal reference; not independently measured seam truth',
    input_sha256=input_hashes,
    provenance_limit='Recorded session contains file paths but not runtime calibration hashes or executable/JBI version; current local calibration is used.')
(OUT / 'results.json').write_text(json.dumps(result, ensure_ascii=False, indent=2,
    default=mathlib.serializable), encoding='utf-8')
pd.DataFrame([dict(comparison=key, **value) for key, value in metrics.items()]).to_csv(
    OUT / 'metrics.csv', index=False, encoding='utf-8-sig')
old = json.loads((PREVIOUS / 'results.json').read_text(encoding='utf-8'))
selected = [('TCP → 移动后原始示教折线', 'TCP_to_relocated_teach'),
            ('KCF → 移动后原始示教折线', 'KCF_to_relocated_teach'),
            ('TCP → 整圈原始 KCF 折线', 'TCP_to_KCF'),
            ('KCF → 整圈 TCP 折线', 'KCF_to_TCP'),
            ('TCP → 移动后示教拟合圆', 'TCP_to_relocated_teach_circle')]
rows = ''.join('<tr><td>' + label + '</td>' + ''.join(
    f'<td>{metrics[key][field]:.3f}</td>' for field in ['mean_mm', 'max_mm', 'rmse_mm', 'p95_mm'])
    + '</tr>' for label, key in selected)
comparison_rows = ''.join(f'<tr><td>{label}</td><td>{old["metrics"][key]["mean_mm"]:.3f}</td>'
    f'<td>{metrics[key]["mean_mm"]:.3f}</td><td>{old["metrics"][key]["max_mm"]:.3f}</td>'
    f'<td>{metrics[key]["max_mm"]:.3f}</td></tr>' for label, key in selected[:-1])
segment_rows = ''.join(f'<tr><td>JBI 行 {item["job_line"]}</td><td>{item["count"]}</td>'
    f'<td>{item["tcp_to_kcf"]["mean_mm"]:.3f}</td><td>{item["tcp_to_kcf"]["max_mm"]:.3f}</td>'
    f'<td>{item["tcp_to_relocated_teach"]["mean_mm"]:.3f}</td></tr>' for item in segments)
report = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>21:12 五个 P 点实验分析</title><style>body{{font:16px/1.8 system-ui,sans-serif;color:#203345;max-width:1100px;margin:30px auto;padding:0 22px}}table{{border-collapse:collapse;width:100%;margin:18px 0}}td,th{{padding:9px;border-bottom:1px solid #d4dfe8;text-align:right}}td:first-child,th:first-child{{text-align:left}}a{{color:#0072b2}}iframe{{width:100%;height:860px;border:1px solid #d4dfe8}}</style>
<h1>21:12 五个 P 点实验：轨迹与距离分析</h1>
<p><a href="interactive_3d/index.html">打开可旋转四曲线三维图</a> · <a href="metrics.csv">全部指标</a> · <a href="results.json">数据口径与来源</a></p>
<p>工件示教：{html.escape(work_file.name)}（{len(work)} 点）；控制点：{html.escape(teach_file.name)}（{len(teach)} 点）。在线 ICP={summary['enable_online_icp']}。</p>
<p>TCP：{result['tcp_start']} 至 {result['tcp_end']}，{len(robot)} 点；KCF：{result['kcf_start']} 至 {result['kcf_end']}，{len(vision)} 点。排除结束等待阶段 {result['excluded_wait_stop_tcp_count']} 条 TCP。</p>
<h2>本轮距离，单位 mm</h2><table><tr><th>查询点 → 参考</th><th>平均</th><th>最大</th><th>RMS</th><th>P95</th></tr>{rows}</table>
<p>逐点到全部参考线段求最短三维距离，不按行号或相同时刻配对，不删异常点、不平滑。拟合圆距离单独列出。</p>
<h2>与上午实验对照，单位 mm</h2><table><tr><th>比较项</th><th>上午平均</th><th>本轮平均</th><th>上午最大</th><th>本轮最大</th></tr>{comparison_rows}</table>
<h2>首段姿态变化与分段观察</h2><p>本轮 S2 速度中位约 1.591°/s，上午约 0.525°/s。首段（JBI 行 13，21:13:00.618—21:13:56.940）工具 Rx 约 −176.19°→−160.68°，Ry 约 41.34°→30.57°；后面三段姿态接近恒定。初始下发的 P1/P2 目标也存在此姿态差，因此不能沿用上午全程近固定姿态的前提。</p>
<table><tr><th>反馈程序行</th><th>TCP 点数</th><th>TCP→KCF 平均</th><th>TCP→KCF 最大</th><th>TCP→移动示教平均</th></tr>{segment_rows}</table>
<p>首段 TCP/KCF 几何差距更大，但移动示教最大距离发生在第三段 {metrics['TCP_to_relocated_teach']['max_time']}；不是同一种偏差。速度和姿态变化与差距同时存在，现有数据不能据此单独认定原因。</p>
<h2>9.423 mm 最大距离的首尾缺口影响</h2><p>KCF 原始轨迹没有闭合：首末点相距 {gap_diagnostic['kcf_endpoint_gap_mm']:.3f} mm，最大未采样扇区 {gap_diagnostic['gap_start_deg']:.4f}°—{gap_diagnostic['gap_end_deg']:.4f}°（{gap_diagnostic['largest_unsampled_sector_deg']:.4f}°）。TCP 最大距离点的材料角为 {max_query_angle:.4f}°，落在该缺口中。主结果保留原始开放折线，没有人为补线。</p>
<p>只作敏感性检验：人工连接 KCF 末点与首点会影响 {gap_diagnostic['synthetic_closing_segment_affected_tcp_count']} 个 TCP 点，最大距离变为 {gap_diagnostic['synthetic_closed_metric_not_primary']['max_mm']:.3f} mm。补线不是实测数据，不替代主指标；9.423 mm 不能直接称为纯机器人跟随误差。</p>
<p>日志中 ICP 共尝试 383 次、应用 366 次；17 次回退中有 13 次集中在闭环尾段附近。尾段目标可能更多回退到原始检测点，这不等于证明 KCF 不准确。</p>
<h2>参考与解释边界</h2><p>原示教平衡拟合半径 {teach_circle['radius_mm']:.6f} mm；本轮 KCF 平衡拟合半径 {kcf_circle['radius_mm']:.6f} mm。移动后示教只做刚体变换，保留原半径。TCP 不参与参考定位。TCP/KCF 角度覆盖约 {result['tcp_coverage_deg']:.2f}°/{result['kcf_coverage_deg']:.2f}°。</p>
<p>移动参考由本轮 KCF 的圆心与法线确定，属于内部参考，不能证明绝对焊缝精度。近圆轮廓无法唯一确定绕法线转角；本图采用最小法线对齐，不另拟合圆周相位。整圈最近距离不代表实时延迟误差。</p>
<p>动态 KCF 固定基座坐标使用结果记录时间插值外轴，只是近似；主距离在材料/末节系计算，图中则统一到首个有效 TCP 外轴姿态 S1={s1:.4f}°、S2={s2:.4f}°。</p>
<p>实验名称来自用户提供目录；日志不包含现场 JBI/exe 版本，不能仅由名称确认版本差异。日志仅记录配置路径；示教形状已由初始轨迹逐点验证，标定参数仍缺现场冻结副本。</p>
<p>会话最终状态：{html.escape(summary['session_state'])}；{html.escape(summary['message'])}。结束等待超时不参与主轨迹统计。</p>
<iframe src="interactive_3d/index.html" title="四曲线三维交互图"></iframe></html>'''
(OUT / 'report.html').write_text(report, encoding='utf-8')
assert all(hashlib.sha256(path.read_bytes()).hexdigest() == input_hashes[str(path.relative_to(ROOT))]
           for path in inputs)
print(json.dumps(dict(counts=[len(robot), len(vision)], metrics=metrics,
    radii_mm=[teach_circle['radius_mm'], kcf_circle['radius_mm']],
    coverage_deg=[result['tcp_coverage_deg'], result['kcf_coverage_deg']]),
    ensure_ascii=False, indent=2))
