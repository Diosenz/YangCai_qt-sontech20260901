"""Compare analysis-reference point relocation with the actual five initial O commands."""
from pathlib import Path
import json

import numpy as np
import pandas as pd


OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
result = json.loads((OUT / 'results.json').read_text(encoding='utf-8'))
original = np.loadtxt(ROOT / result['teach_pose_file'])
initial = pd.read_csv(ROOT / result['source_session_directory'] / 'initial_correction.txt', sep='\t')
commands = initial[initial.event == 'teach_point'].reset_index(drop=True)
assert original.shape == (5, 9) and len(commands) == 5
alignment = result['display_registration']
analysis_matrix = np.eye(4)
analysis_matrix[:3, :3] = alignment['rotation']
analysis_matrix[:3, 3] = alignment['translation']
assert np.allclose(analysis_matrix[:3, :3].T @ analysis_matrix[:3, :3], np.eye(3), atol=1e-10)
initial_point = commands.iloc[0][['converted_x_mm', 'converted_y_mm', 'converted_z_mm']].to_numpy(float)
runtime_matrix = np.eye(4)
runtime_matrix[:3, 3] = initial_point - original[0, :3]
analysis_points = original[:, :3] @ analysis_matrix[:3, :3].T + analysis_matrix[:3, 3]
runtime_points = original[:, :3] + runtime_matrix[:3, 3]
calibration = np.array(result['calibration_srangle'])
def rotation_x(angle):
    return np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)], [0, np.sin(angle), np.cos(angle)]])
def rotation_y(angle):
    return np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
def rotation_z(angle):
    return np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
angles = np.deg2rad(-calibration[3:])
robot_to_station = rotation_x(angles[0]) @ rotation_y(angles[1]) @ rotation_z(angles[2])
robot_points = np.array([robot_to_station.T @ (rotation_y(np.deg2rad(row[6]))
    @ rotation_z(np.deg2rad(row[7])) @ point) + calibration[:3]
    for row, point in zip(original, runtime_points)])
logged = commands[['robot_x_mm', 'robot_y_mm', 'robot_z_mm']].to_numpy()
assert np.max(np.abs(robot_points - logged)) <= 0.000501
table = pd.DataFrame({'point': [f'P{index:03d}' for index in range(1, 6)]})
for prefix, points in [('original_last_link', original[:, :3]),
                       ('analysis_reference_last_link', analysis_points),
                       ('runtime_aligned_last_link', runtime_points),
                       ('recomputed_runtime_robot_base', robot_points),
                       ('logged_runtime_robot_base', logged)]:
    for index, axis in enumerate('xyz'):
        table[f'{prefix}_{axis}_mm'] = points[:, index]
for field in ['robot_rx_deg', 'robot_ry_deg', 'robot_rz_deg', 's1_deg', 's2_deg']:
    table['logged_' + field] = commands[field].to_numpy()
table.to_csv(OUT / 'five_teach_point_transform_comparison.csv', index=False,
             encoding='utf-8-sig', float_format='%.9f')
np.savetxt(OUT / 'runtime_initial_alignment_last_link_transform.csv', runtime_matrix,
           delimiter=',', fmt='%.12f')
print('Verified all five robot XYZ values against O-command log within 0.000501 mm rounding tolerance.')
print(table[['point'] + [f'analysis_reference_last_link_{axis}_mm' for axis in 'xyz']].to_string(index=False))
