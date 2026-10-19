"""Offline trajectory analysis; reads original records and writes derived artifacts only."""
from pathlib import Path
import argparse
import hashlib
import html
import json
import os
import sys

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
SESSION = ROOT / 'output/online_correction/20261019_110509_938'


def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def external_rotation(points, s1, s2, inverse=False):
    """Row vectors: S = Ry(s1) Rz(s2) L; angles in degrees."""
    q = np.asarray(points, float)
    a, b = np.deg2rad(s1), np.deg2rad(s2)
    if inverse:
        x = np.cos(a) * q[:, 0] - np.sin(a) * q[:, 2]
        z = np.sin(a) * q[:, 0] + np.cos(a) * q[:, 2]
        return np.c_[np.cos(b) * x + np.sin(b) * q[:, 1],
                     -np.sin(b) * x + np.cos(b) * q[:, 1], z]
    x = np.cos(b) * q[:, 0] - np.sin(b) * q[:, 1]
    y = np.sin(b) * q[:, 0] + np.cos(b) * q[:, 1]
    return np.c_[np.cos(a) * x + np.sin(a) * q[:, 2],
                 y, -np.sin(a) * x + np.cos(a) * q[:, 2]]


def closest_polyline(query, reference, batch=128):
    """Exact Euclidean projection onto each recorded segment, no curve smoothing."""
    query, reference = np.asarray(query, float), np.asarray(reference, float)
    a = reference[:-1]
    delta = reference[1:] - a
    lengths2 = np.maximum(np.einsum('ij,ij->i', delta, delta), 1.e-24)
    distance, projected, segment, fraction = [], [], [], []
    for start in range(0, len(query), batch):
        diff = query[start:start + batch, None, :] - a[None, :, :]
        t = np.clip(np.einsum('ijk,jk->ij', diff, delta) / lengths2, 0., 1.)
        diff -= t[:, :, None] * delta[None, :, :]
        norm2 = np.einsum('ijk,ijk->ij', diff, diff)
        j = norm2.argmin(axis=1)
        f = t[np.arange(len(j)), j]
        distance.append(np.sqrt(norm2[np.arange(len(j)), j]))
        projected.append(a[j] + f[:, None] * delta[j])
        segment.append(j)
        fraction.append(f)
    return tuple(np.concatenate(x) for x in (distance, projected, segment, fraction))


def metric(distance):
    d = np.asarray(distance)
    return {'n': int(len(d)), 'mean_mm': float(d.mean()),
            'max_mm': float(d.max()), 'rmse_mm': float(np.sqrt(np.mean(d ** 2))),
            'median_mm': float(np.median(d)), 'p95_mm': float(np.quantile(d, .95))}


def circle(points):
    """Descriptive circle fit in the least-squares plane, without altering trajectories."""
    center = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - center, full_matrices=False)
    uv = (points - center) @ vt[:2].T
    solution, *_ = np.linalg.lstsq(np.c_[2 * uv, np.ones(len(uv))],
                                   np.einsum('ij,ij->i', uv, uv), rcond=None)
    radius = np.sqrt(solution[2] + solution[:2] @ solution[:2])
    # Refine geometric radial distances, with bounded influence on fitting only.
    xy = solution[:2].copy()
    for _ in range(40):
        delta = uv - xy
        length = np.maximum(np.linalg.norm(delta, axis=1), 1.e-12)
        residual = length - radius
        scale = max(1.4826 * np.median(np.abs(residual - np.median(residual))), .02)
        weight = np.minimum(1., 1.5 * scale / np.maximum(np.abs(residual), 1.e-12))
        jacobian = np.c_[-delta / length[:, None], -np.ones(len(uv))]
        step, *_ = np.linalg.lstsq(jacobian * np.sqrt(weight)[:, None],
                                   -residual * np.sqrt(weight), rcond=None)
        xy += step[:2]
        radius += step[2]
        if np.linalg.norm(step) < 1.e-9:
            break
    normal = vt[2] if vt[2, 2] >= 0 else -vt[2]
    return {'center': center + xy @ vt[:2], 'radius_mm': float(radius),
            'normal': normal}


def circle_pose_alignment(model_circle, target_circle):
    """Map centers and plane normals; preserve radius and add no arbitrary axial yaw."""
    a, b = model_circle['normal'], target_circle['normal']
    v = np.cross(a, b)
    c = np.clip(a @ b, -1., 1.)
    k = np.array([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])
    rotation = np.eye(3) + k + k @ k / max(1. + c, 1.e-12)
    translation = target_circle['center'] - rotation @ model_circle['center']
    return {'rotation': rotation, 'translation': translation,
            'normal_alignment_deg': float(np.rad2deg(np.arccos(c))),
            'axial_rotation_is_identifiable': False, 'scale': 1.}


def circle_distance(points, fitted):
    delta = points - fitted['center']
    height = delta @ fitted['normal']
    radius = np.sqrt(np.maximum(np.einsum('ij,ij->i', delta, delta) - height ** 2, 0.))
    return np.sqrt((radius - fitted['radius_mm']) ** 2 + height ** 2)


def kabsch(source, target):
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - a).T @ (target - b))
    fix = np.eye(3)
    fix[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ fix @ u.T
    return rotation, b - rotation @ a


def representatives(points, center, mask=None):
    angle = np.mod(np.arctan2(points[:, 1] - center[1],
                             points[:, 0] - center[0]), 2 * np.pi)
    bins = np.floor(np.rad2deg(angle) / 2).astype(int)
    if mask is None:
        mask = np.ones(len(points), bool)
    return np.array([np.median(points[(bins == b) & mask], axis=0)
                     for b in np.unique(bins[mask])])


def rigid_fit(model, fit_points, model_center, target_center):
    """Multi-start SE(3) point-to-polyline ICP. No scale or nonrigid deformation."""
    candidates = []
    for seed in range(0, 360, 30):
        rotation = rz(np.deg2rad(seed))
        translation = target_center - rotation @ model_center
        previous = np.inf
        for iteration in range(60):
            current = model @ rotation.T + translation
            d, projected, _, _ = closest_polyline(fit_points, current)
            dr, dt = kabsch(projected, fit_points)
            rotation, translation = dr @ rotation, dr @ translation + dt
            error = float(np.mean(d * d))
            if abs(previous - error) < 1.e-9:
                break
            previous = error
        final = model @ rotation.T + translation
        d, *_ = closest_polyline(fit_points, final)
        candidates.append({'seed_deg': seed, 'rms_mm': float(np.sqrt(np.mean(d * d))),
                           'iterations': iteration + 1, 'rotation': rotation,
                           'translation': translation})
    candidates.sort(key=lambda x: x['rms_mm'])
    best = candidates[0]
    assert np.allclose(best['rotation'].T @ best['rotation'], np.eye(3), atol=1.e-8)
    assert np.linalg.det(best['rotation']) > .999999
    return best, candidates


def serializable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    raise TypeError(type(value))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fit-reference-from-kcf', action='store_true')
    args = parser.parse_args()
    robot_all = pd.read_csv(SESSION / 'actual_robot_path.txt', sep='\t')
    realtime = pd.read_csv(SESSION / 'realtime_correction.txt', sep='\t')
    initial = pd.read_csv(SESSION / 'initial_correction.txt', sep='\t')
    summary_rows = pd.read_csv(SESSION / 'session_summary.txt', sep='\t')
    summary = dict(zip(summary_rows.key, summary_rows.value))
    work_file, teach_file, calibration_file = map(Path, [summary['work_path_file'],
                      summary['teach_pose_file'], summary['rbcalib_file']])
    work = np.loadtxt(work_file)
    teach = np.loadtxt(teach_file)
    values = [line.split(' ', 1)[1] for line in calibration_file.read_text(
        encoding='utf-8-sig').splitlines() if line.startswith('////SRANG ')][0]
    calibration = np.array([float(x) for x in values.split(',')])
    angles = np.deg2rad(-calibration[3:])
    robot_to_station_rotation = rx(angles[0]) @ ry(angles[1]) @ rz(angles[2])
    tcp_mask = ((robot_all.phase == 'online_running') & (robot_all.corflag == 1)
                & (robot_all.detflag == 1) & (robot_all.s_start == 1)
                & (robot_all.s_hold == 0) & (robot_all.alarm == 0))
    robot = robot_all[tcp_mask].copy().reset_index(drop=True)
    vision = realtime[(realtime.event == 'vision_sample')
                      & (realtime.phase == 'online_running')
                      & (realtime.vision_valid == 1)].copy().reset_index(drop=True)
    robot_xyz = robot[['x_mm', 'y_mm', 'z_mm']].to_numpy()
    tcp_fixed = (robot_xyz - calibration[:3]) @ robot_to_station_rotation.T
    tcp_local = external_rotation(tcp_fixed, robot.s1_deg.to_numpy(),
                                  robot.s2_deg.to_numpy(), inverse=True)
    kcf_local = vision[['vision_x_mm', 'vision_y_mm', 'vision_z_mm']].to_numpy(copy=True)
    kcf_local += np.array([0., 0., 231.])
    all_sorted = robot_all.sort_values('timestamp_ms').drop_duplicates('timestamp_ms', keep='last')
    robot_times = all_sorted.timestamp_ms.to_numpy()
    vision_times = vision.timestamp_ms.to_numpy()
    vision_s1 = np.interp(vision_times, robot_times, all_sorted.s1_deg)
    vision_s2 = np.rad2deg(np.interp(vision_times, robot_times,
                                    np.unwrap(np.deg2rad(all_sorted.s2_deg))))
    kcf_fixed = external_rotation(kcf_local, vision_s1, vision_s2)
    assert np.allclose(external_rotation(tcp_local, robot.s1_deg.to_numpy(),
                                        robot.s2_deg.to_numpy()), tcp_fixed, atol=1.e-9)
    assert np.allclose(external_rotation(kcf_fixed, vision_s1, vision_s2, True),
                       kcf_local, atol=1.e-9)
    coordinates = [('tcp', robot, tcp_local, tcp_fixed), ('kcf', vision, kcf_local, kcf_fixed)]
    for name, frame, local, fixed in coordinates:
        frame = frame.copy()
        for index, axis in enumerate('xyz'):
            frame[f'{axis}_station_base_mm'] = fixed[:, index]
            frame[f'{axis}_material_frame_mm'] = local[:, index]
        if name == 'kcf':
            frame['s1_interpolated_deg'] = vision_s1
            frame['s2_interpolated_deg'] = vision_s2
            frame['fixed_frame_pose_is_approximate'] = True
        frame.to_csv(OUT / f'{name}_coordinates.csv', index=False, encoding='utf-8-sig')

    pairs = {}
    for name, source, reference, frame, s1, s2 in [
        ('TCP_to_KCF', tcp_local, kcf_local, robot, robot.s1_deg.to_numpy(), robot.s2_deg.to_numpy()),
        ('KCF_to_TCP', kcf_local, tcp_local, vision, vision_s1, vision_s2),
        ('TCP_to_original_teach', tcp_local, work[:, :3], robot, robot.s1_deg.to_numpy(), robot.s2_deg.to_numpy()),
        ('KCF_to_original_teach', kcf_local, work[:, :3], vision, vision_s1, vision_s2)]:
        distance, projected, segment, fraction = closest_polyline(source, reference)
        source_s = external_rotation(source, s1, s2)
        projected_s = external_rotation(projected, s1, s2)
        assert np.allclose(np.linalg.norm(source_s - projected_s, axis=1), distance, atol=1.e-9)
        detail = pd.DataFrame({'timestamp_ms': frame.timestamp_ms, 'time': frame.time,
                               'distance_mm': distance, 'reference_segment_index_0based': segment,
                               'reference_segment_fraction': fraction})
        for axis, index in zip('xyz', range(3)):
            detail[f'source_{axis}_station_base_mm'] = source_s[:, index]
            detail[f'matched_reference_{axis}_station_base_mm'] = projected_s[:, index]
            detail[f'residual_{axis}_station_base_mm'] = source_s[:, index] - projected_s[:, index]
        detail.to_csv(OUT / f'{name}_distances.csv', index=False, encoding='utf-8-sig')
        result = metric(distance)
        max_index = int(np.argmax(distance))
        result.update(max_time=str(frame.time.iloc[max_index]), max_index=max_index)
        if name == 'TCP_to_KCF':
            result['closest_kcf_record_time_at_max'] = str(vision.time.iloc[segment[max_index]])
        pairs[name] = {'metric': result, 'distance': distance, 'projected': projected,
                       'segment': segment, 'fraction': fraction}

    results = {'session': SESSION.name, 'tcp_count': len(robot), 'kcf_count': len(vision),
               'teach_work_count': len(work), 'teach_control_count': len(teach),
               'tcp_start': str(robot.time.iloc[0]), 'tcp_end': str(robot.time.iloc[-1]),
               'kcf_start': str(vision.time.iloc[0]), 'kcf_end': str(vision.time.iloc[-1]),
               'raw_session_robot_count': len(robot_all),
               'raw_session_vision_count': int((realtime.event == 'vision_sample').sum()),
               'excluded_wait_stop_tcp_count': int((robot_all.phase == 'waiting_robot_program_stop').sum()),
               'calibration_srangle': calibration, 'reference_source': 'not_selected',
               'metric_definition': 'd=3D Euclidean point-to-recorded-polyline distance; mean(d), max(d), sqrt(mean(d^2)); sample weighted',
               'metrics': {name: x['metric'] for name, x in pairs.items()},
               'original_teach_circle': circle(work[:, :3]), 'kcf_circle': circle(kcf_local)}
    corrected = initial[initial.event == 'corrected_path_point'].drop_duplicates('point_index')
    corrected_xyz = corrected[['aligned_first_x_mm', 'aligned_first_y_mm', 'aligned_first_z_mm']].to_numpy()
    if len(corrected_xyz) == len(work):
        results['startup_translation_not_analysis_registration_mm'] = np.median(corrected_xyz - work[:, :3], axis=0)
    # A separate diagnostic: contemporaneous separation is not a tracking-error metric.
    tcp_at_vision = np.column_stack([np.interp(vision_times, robot.timestamp_ms, tcp_fixed[:, i]) for i in range(3)])
    results['contemporaneous_sensor_tcp_separation_not_error_mm'] = metric(np.linalg.norm(kcf_fixed - tcp_at_vision, axis=1))
    source_files = [SESSION / f for f in ['actual_robot_path.txt', 'realtime_correction.txt',
                       'initial_correction.txt', 'session_summary.txt']] + [work_file, teach_file, calibration_file]
    results['input_sha256'] = {str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in source_files}

    aligned_work = None
    if args.fit_reference_from_kcf:
        results['reference_source'] = 'KCF_3D_circle_center_and_plane_relocated_original_teach_internal_reference'
        center = results['kcf_circle']['center']
        model_center = results['original_teach_circle']['center']
        fit_points = representatives(kcf_local, center)
        target_circle = circle(fit_points)
        teach_circle = circle(representatives(work[:, :3], model_center))
        best = circle_pose_alignment(teach_circle, target_circle)
        aligned_work = work[:, :3] @ best['rotation'].T + best['translation']
        results['display_registration'] = best
        results['kcf_fitted_circle_balanced'] = target_circle
        results['teach_fitted_circle_balanced'] = teach_circle
        results['circle_radius_difference_mm'] = target_circle['radius_mm'] - teach_circle['radius_mm']
        results['circle_center_difference_material_mm'] = target_circle['center'] - teach_circle['center']
        results['metrics']['KCF_to_fitted_circle'] = metric(circle_distance(kcf_local, target_circle))
        results['metrics']['TCP_to_fitted_circle'] = metric(circle_distance(tcp_local, target_circle))
        pd.DataFrame(fit_points, columns=['x_mm', 'y_mm', 'z_mm']).to_csv(
            OUT / 'circle_fit_angle_balanced_representatives.csv', index=False, encoding='utf-8-sig')
        sectors = np.floor(np.mod(np.arctan2(kcf_local[:, 1] - center[1],
                          kcf_local[:, 0] - center[0]), 2 * np.pi) / (np.pi / 4)).astype(int) % 2
        tcp_sectors = np.floor(np.mod(np.arctan2(tcp_local[:, 1] - center[1],
                              tcp_local[:, 0] - center[0]), 2 * np.pi) / (np.pi / 4)).astype(int) % 2
        cv_kcf, cv_tcp = np.zeros(len(kcf_local)), np.zeros(len(tcp_local))
        cv_fold_details = []
        for held_fold in [0, 1]:
            train = sectors != held_fold
            fit = circle_pose_alignment(teach_circle, circle(representatives(kcf_local, center, train)))
            fitted = work[:, :3] @ fit['rotation'].T + fit['translation']
            cv_kcf[sectors == held_fold] = closest_polyline(kcf_local[sectors == held_fold], fitted)[0]
            cv_tcp[tcp_sectors == held_fold] = closest_polyline(tcp_local[tcp_sectors == held_fold], fitted)[0]
            cv_fold_details.append({'held_fold': held_fold, 'kcf_train_n': int(train.sum()), 'fit': fit})
        results['registration_cv_folds'] = cv_fold_details
        for name, source, frame in [('TCP_to_relocated_teach', tcp_local, robot),
                                     ('KCF_to_relocated_teach', kcf_local, vision)]:
            distance, projected, segment, fraction = closest_polyline(source, aligned_work)
            results['metrics'][name] = metric(distance)
            results['metrics'][name]['max_time'] = str(frame.time.iloc[int(np.argmax(distance))])
            s1, s2 = (robot.s1_deg.to_numpy(), robot.s2_deg.to_numpy()) if name.startswith('TCP') else (vision_s1, vision_s2)
            source_s, projected_s = external_rotation(source, s1, s2), external_rotation(projected, s1, s2)
            assert np.allclose(np.linalg.norm(source_s - projected_s, axis=1), distance, atol=1.e-9)
            detail = pd.DataFrame({'timestamp_ms': frame.timestamp_ms, 'time': frame.time,
                          'distance_mm': distance, 'reference_segment_index_0based': segment,
                          'reference_segment_fraction': fraction})
            for index, axis in enumerate('xyz'):
                detail[f'source_{axis}_station_base_mm'] = source_s[:, index]
                detail[f'reference_{axis}_station_base_mm'] = projected_s[:, index]
                detail[f'residual_{axis}_station_base_mm'] = source_s[:, index] - projected_s[:, index]
            detail.to_csv(OUT / f'{name}_distances.csv', index=False, encoding='utf-8-sig')
        results['metrics']['TCP_to_relocated_teach_sector_holdout'] = metric(cv_tcp)
        results['metrics']['KCF_to_relocated_teach_sector_holdout'] = metric(cv_kcf)
        pd.DataFrame({'time': vision.time, 'held_sector_parity': sectors,
                      'held_out_distance_mm': cv_kcf}).to_csv(OUT / 'KCF_sector_holdout.csv', index=False, encoding='utf-8-sig')
        matrix = np.eye(4)
        matrix[:3, :3], matrix[:3, 3] = best['rotation'], best['translation']
        np.savetxt(OUT / 'teach_to_current_material_frame_transform.csv', matrix, delimiter=',', fmt='%.12f')
        pd.DataFrame(aligned_work, columns=['x_mm', 'y_mm', 'z_mm']).to_csv(OUT / 'relocated_teach_material_frame.csv', index=False, encoding='utf-8-sig')
        a0, b0 = float(robot.s1_deg.iloc[0]), float(robot.s2_deg.iloc[0])
        b_rotation = ry(np.deg2rad(a0)) @ rz(np.deg2rad(b0))
        results['common_station_base_pose_deg'] = [a0, b0]
        results['kcf_circle_center_common_station_base_mm'] = b_rotation @ target_circle['center']
        results['teach_original_circle_center_common_station_base_mm'] = b_rotation @ teach_circle['center']
        matrix_s = np.eye(4)
        matrix_s[:3, :3] = b_rotation @ best['rotation'] @ b_rotation.T
        matrix_s[:3, 3] = b_rotation @ best['translation']
        np.savetxt(OUT / 'teach_to_current_common_station_base_transform.csv', matrix_s, delimiter=',', fmt='%.12f')
        pd.DataFrame(external_rotation(aligned_work, a0, b0), columns=['x_mm','y_mm','z_mm']).to_csv(
            OUT / 'relocated_teach_common_station_base.csv', index=False, encoding='utf-8-sig')
        results['registration_method'] = '2-degree angular-bin coordinate medians; PCA plane; Huber geometric circle (1.5 MAD, floor 0.02 mm); centers and minimal plane-normal rotation only; scale=1; no axial-yaw fitting'
        print('Circle pose alignment',json.dumps(best, default=serializable),flush=True)

    with (OUT / 'results.json').open('w', encoding='utf-8') as file:
        json.dump(results, file, ensure_ascii=False, indent=2, default=serializable)
    pd.DataFrame([{'comparison': name, **value} for name, value in results['metrics'].items()]).to_csv(
        OUT / 'metrics.csv', index=False, encoding='utf-8-sig')
    print(json.dumps(results['metrics'], ensure_ascii=False, indent=2), flush=True)
    make_figures(results, robot, vision, tcp_local, kcf_local, tcp_fixed, kcf_fixed,
                 work, teach, aligned_work, pairs)
    return results


def make_figures(results, robot, vision, tcp_local, kcf_local, tcp_fixed,
                 kcf_fixed, work, teach, aligned_work, pairs):
    plot_dependencies = Path(os.environ.get('TEMP', '.')) / 'codex-online-analysis-20261019-plotdeps'
    sys.path.insert(0, str(plot_dependencies))
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt, font_manager
    font = Path('C:/Windows/Fonts/msyh.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'axes.unicode_minus': False, 'font.size': 10, 'figure.dpi': 140,
                         'axes.grid': True, 'grid.alpha': .2, 'savefig.facecolor': 'white'})
    a0, b0 = float(robot.s1_deg.iloc[0]), float(robot.s2_deg.iloc[0])
    normalize = lambda q: external_rotation(q, a0, b0)
    data = [(normalize(tcp_local), 'TCP 实际轨迹', '#0072b2'),
            (normalize(kcf_local), 'KCF 原始检测点', '#d55e00'),
            (normalize(work[:, :3]), '原始示教（未补偿工件位置）', '#888888')]
    if aligned_work is not None:
        data.append((normalize(aligned_work), '移动后的示教参考（KCF 圆定位）', '#009e73'))
    fig = plt.figure(figsize=(15, 11), layout='constrained')
    ax = fig.add_subplot(221, projection='3d')
    for q, label, color in data:
        ax.plot(q[:, 0], q[:, 1], q[:, 2], color=color, label=label, lw=1.1, alpha=.8)
    ax.set(xlabel='X / mm', ylabel='Y / mm', zlabel='Z / mm', title='三维轨迹（Z 方向放大以观察高度变化）')
    ax.set_box_aspect((1, 1, .45))
    for index, (x, y, title) in enumerate([(0, 1, 'XY 俯视图'), (0, 2, 'XZ 高度轮廓'), (1, 2, 'YZ 高度轮廓')], 2):
        axis = fig.add_subplot(2, 2, index)
        for q, label, color in data:
            axis.plot(q[:, x], q[:, y], color=color, label=label, lw=1.0, alpha=.75)
        axis.set(xlabel=f'{"XYZ"[x]} / mm', ylabel=f'{"XYZ"[y]} / mm', title=title)
        if y == 1:
            axis.set_aspect('equal', adjustable='box')
        if index == 2:
            axis.legend(fontsize=8, loc='best')
    fig.suptitle(f'在线纠偏轨迹｜统一外轴姿态后的固定变位机基座表达\n'
                 f'S1={a0:.4f}°，S2={b0:.4f}°；只消除已知外轴运动，TCP 不参与拟合', fontsize=14)
    fig.savefig(OUT / '01_trajectories_common_station_pose.png')
    fig.savefig(OUT / '01_trajectories_common_station_pose.svg')
    plt.close(fig)

    historical_teach = external_rotation(work[:, :3], work[:, 6], work[:, 7])
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), layout='constrained')
    for q, label, color in [(tcp_fixed, 'TCP（控制柜反馈）', '#0072b2'),
                             (kcf_fixed, 'KCF（外轴时刻近似重建）', '#d55e00'),
                             (historical_teach, '原始示教按示教外轴历史回放', '#888888')]:
        axes[0].plot(q[:, 0], q[:, 1], label=label, color=color, lw=1)
    if aligned_work is not None:
        q = normalize(aligned_work)
        axes[0].plot(q[:,0], q[:,1], color='#009e73', lw=1,
                     label='移动后整圈示教（在线起始外轴姿态）')
    axes[0].set(xlabel='固定基座 X / mm', ylabel='固定基座 Y / mm', title='各自采样姿态下的物理轨迹（不能直接求跟踪误差）')
    axes[0].set_aspect('equal', adjustable='box')
    axes[0].legend(fontsize=8)
    start = robot.timestamp_ms.iloc[0]
    axes[1].plot((robot.timestamp_ms-start)/1000, tcp_fixed[:, 2], label='TCP Z', color='#0072b2')
    axes[1].plot((vision.timestamp_ms-start)/1000, kcf_fixed[:, 2], label='KCF Z（近似）', color='#d55e00', alpha=.7)
    axes[1].set(xlabel='在线段时间 / s', ylabel='固定基座 Z / mm', title='在线段高度记录')
    axes[1].legend()
    fig.suptitle('实际固定变位机基座轨迹｜TCP 与前视检测点所在位置不同', fontsize=14)
    fig.savefig(OUT / '02_native_station_base.png')
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(14, 11), layout='constrained')
    axes[0].plot((robot.timestamp_ms-start)/1000, pairs['TCP_to_KCF']['distance'], color='#0072b2', lw=.8, label='TCP → KCF 全周期折线')
    axes[0].plot((vision.timestamp_ms-start)/1000, pairs['KCF_to_TCP']['distance'], color='#d55e00', lw=.7, alpha=.6, label='KCF → TCP 全周期折线')
    axes[0].set(title='三维几何贴合距离；不是同时刻点间距，也不是因果控制误差', ylabel='距离 / mm')
    axes[0].legend()
    for name, frame, color in [('TCP_to_original_teach', robot, '#0072b2'),
                                ('KCF_to_original_teach', vision, '#d55e00')]:
        axes[1].plot((frame.timestamp_ms-start)/1000, pairs[name]['distance'], color=color, lw=.8, alpha=.8, label=name)
    axes[1].set(title='未移动原始示教的名义布局差异（包含工件位置变化，不作纠偏误差）', ylabel='距离 / mm')
    axes[1].legend(fontsize=8)
    if aligned_work is not None:
        for name, frame, color in [('TCP_to_relocated_teach', robot, '#0072b2'),
                                    ('KCF_to_relocated_teach', vision, '#d55e00')]:
            detail = pd.read_csv(OUT / f'{name}_distances.csv')
            axes[2].plot((frame.timestamp_ms-start)/1000, detail.distance_mm, label=name, color=color, lw=.8, alpha=.8)
        axes[2].set(title='移动后示教参考的形状残差（当前位置由 KCF 拟合，非独立真值）', ylabel='距离 / mm')
        axes[2].legend(fontsize=8)
    axes[2].set_xlabel('在线段时间 / s')
    fig.savefig(OUT / '03_geometric_distances.png')
    plt.close(fig)


if __name__ == '__main__':
    main()
