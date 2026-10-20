"""Build an offline viewer using the existing analysis, without changing its inputs."""
from pathlib import Path
import hashlib
import json

import numpy as np
import pandas as pd


OUT = Path(__file__).resolve().parent
ANALYSIS = OUT.parent
results = json.loads((ANALYSIS / 'results.json').read_text(encoding='utf-8'))
angle1, angle2 = np.deg2rad(results['common_station_base_pose_deg'])
rotation_y = np.array([[np.cos(angle1), 0, np.sin(angle1)], [0, 1, 0],
                       [-np.sin(angle1), 0, np.cos(angle1)]])
rotation_z = np.array([[np.cos(angle2), -np.sin(angle2), 0],
                       [np.sin(angle2), np.cos(angle2), 0], [0, 0, 1]])
rotation = rotation_y @ rotation_z


def sample_circle(model):
    normal = np.array(model['normal'])
    normal /= np.linalg.norm(normal)
    basis_u = np.array([1., 0., 0.]) - normal * normal[0]
    basis_u /= np.linalg.norm(basis_u)
    basis_v = np.cross(normal, basis_u)
    angle = np.linspace(0, 2 * np.pi, 1441)
    points = np.array(model['center']) + model['radius_mm'] * (
        np.cos(angle)[:, None] * basis_u + np.sin(angle)[:, None] * basis_v)
    assert np.allclose(np.linalg.norm(points - model['center'], axis=1),
                       model['radius_mm'], atol=1e-9)
    assert np.allclose((points - model['center']) @ normal, 0, atol=1e-9)
    return points


def trace(points, name, color, times=None, dash='solid'):
    common = points @ rotation.T
    result = dict(type='scatter3d', mode='lines', name=name,
                  x=common[:, 0].tolist(), y=common[:, 1].tolist(),
                  z=common[:, 2].tolist(), line=dict(color=color, width=3, dash=dash),
                  hovertemplate=name + '<br>X=%{x:.4f} mm<br>Y=%{y:.4f} mm'
                  '<br>Z=%{z:.4f} mm<extra></extra>')
    if times is not None:
        result['customdata'] = times
        result['hovertemplate'] = result['hovertemplate'].replace(
            '<extra>', '<br>%{customdata}<extra>')
    return result


traces = []
for filename, label, color in [('tcp_coordinates.csv', 'TCP 实际轨迹（原始点连线）', '#0072b2'),
                               ('kcf_coordinates.csv', 'KCF 原始检测点（未拟合）', '#d55e00')]:
    frame = pd.read_csv(ANALYSIS / filename)
    points = frame[[axis + '_material_frame_mm' for axis in 'xyz']].to_numpy()
    assert np.isfinite(points).all()
    hover_times = frame.time.tolist()
    if filename == 'tcp_coordinates.csv':
        hover_times = [f'{row.time}<br>JBI 行 {int(row.job_line)}<br>'
                       f'Rx/Ry/Rz={row.rx_deg:.3f}/{row.ry_deg:.3f}/{row.rz_deg:.3f}°'
                       for row in frame.itertuples()]
    traces.append(trace(points, label, color, hover_times))

original = sample_circle(results['teach_fitted_circle_balanced'])
alignment = results['display_registration']
relocated = original @ np.array(alignment['rotation']).T + alignment['translation']
assert np.allclose(np.linalg.norm(relocated - np.array(
    results['kcf_fitted_circle_balanced']['center']), axis=1),
    results['teach_fitted_circle_balanced']['radius_mm'], atol=1e-8)
traces.append(trace(original, '原始示教拟合圆（未补偿工件位置）', '#888888', dash='dash'))
traces.append(trace(relocated, '移动后示教拟合圆（KCF 圆定位）', '#009e73'))
teach_path = OUT.parents[3] / 'data/WorkPiece/20260404CHUFENGKOU1_W.txt'
teach_points = np.loadtxt(teach_path)[:, :3]
relocated_points = pd.read_csv(ANALYSIS / 'relocated_teach_material_frame.csv').to_numpy()
assert len(teach_points) == len(relocated_points) == results['teach_work_count']
assert np.allclose(teach_points @ np.array(alignment['rotation']).T
                   + alignment['translation'], relocated_points, atol=1e-9)
for points, label, color in [
        (teach_points, '原始示教实际点（未补偿工件位置）', '#888888'),
        (relocated_points, '移动后示教实际点（KCF 圆定位）', '#009e73')]:
    item = trace(points, label, color)
    item['mode'] = 'lines+markers'
    item['marker'] = dict(size=2, color=color)
    traces.append(item)
traces[4]['visible'] = False
traces[5]['visible'] = False
all_points = np.vstack([np.column_stack([item[axis] for axis in 'xyz']) for item in traces])
low, high = all_points.min(axis=0), all_points.max(axis=0)
span = high - low
ranges = np.column_stack([low - span * .06, high + span * .06]).tolist()
metadata = dict(session=results['session'],
                common_station_base_pose_deg=results['common_station_base_pose_deg'],
                original_teach_file='20260404CHUFENGKOU1_W.txt',
                original_radius_mm=results['teach_fitted_circle_balanced']['radius_mm'],
                relocated_radius_mm=results['teach_fitted_circle_balanced']['radius_mm'],
                trace_counts=[len(item['x']) for item in traces],
                inputs_sha256={name: hashlib.sha256((ANALYSIS / name).read_bytes()).hexdigest()
                    for name in ['results.json', 'tcp_coordinates.csv', 'kcf_coordinates.csv',
                                 'relocated_teach_material_frame.csv']})
metadata['original_teach_sha256'] = hashlib.sha256(teach_path.read_bytes()).hexdigest()
(OUT / 'viewer_metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
library = (OUT / 'plotly-2.35.2.min.js').read_text(encoding='utf-8')
template = (OUT / 'viewer_template.html').read_text(encoding='utf-8')
page = template.replace('/* PLOTLY_LIBRARY */', library).replace(
    '/* TRACE_DATA */', json.dumps(traces, ensure_ascii=False)).replace(
    '/* AXIS_RANGES */', json.dumps(ranges)).replace('/* AXIS_SPANS */', json.dumps(span.tolist()))
page = page.replace('COMMON_POSE', ', '.join(f'{value:.4f}°' for value in metadata['common_station_base_pose_deg']))
page = page.replace('RADIUS_VALUE', f'{metadata["original_radius_mm"]:.6f}')
page = page.replace('20261019_110509_938', results['session'])
page = page.replace('TCP 7,807 点、KCF 10,156 点',
                    f'TCP {results["tcp_count"]:,} 点、KCF {results["kcf_count"]:,} 点')
page = page.replace('在线纠偏：四曲线三维观察', '五个 P 点实验：四曲线三维观察')
page = page.replace('在线纠偏｜四曲线三维观察', '21:12 五个 P 点实验｜四曲线三维观察')
page = page.replace('</header>',
    f'<p>TCP → 移动后原始示教折线：平均 {results["metrics"]["TCP_to_relocated_teach"]["mean_mm"]:.3f} mm，'
    f'最大 {results["metrics"]["TCP_to_relocated_teach"]["max_mm"]:.3f} mm。</p>'
    '<p>重点观察首段 21:13:00—21:13:57 的工具姿态变化；悬停蓝线可查看时间、程序行和姿态。</p></header>')
page = page.replace('<footer>', '<footer><a href="../report.html">查看距离指标与上午实验对照</a> · ')
(OUT / 'index.html').write_text(page, encoding='utf-8')
print(json.dumps(metadata, ensure_ascii=False, indent=2))
