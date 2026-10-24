"""Create explicit synthetic and archived-pixel demos without motion devices."""
import argparse
import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=pathlib.Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('choose a new output directory; previous evidence is preserved')
    args.output.mkdir(parents=True)
    pixels = bytes(225 if abs(y-(260-.45*abs(x-360))) < 1.5 else 14
                   for y in range(480) for x in range(640))
    (args.output/'visible.pgm').write_bytes(b'P5\n640 480\n255\n' + pixels)
    (args.output/'blank.pgm').write_bytes(b'P5\n640 480\n255\n' + bytes([14])*640*480)
    rows = [dict(image=name, frame_id=frame, kind='synthetic', timestamp_ms=1000+100*frame,
                 reference_revision=1, bounded_search=frame > 0,
                 projection={'fixture': 'synthetic_identity_1mm_per_pixel',
                             'provenance': 'defined synthetic projection; not physical calibration'})
            for frame, name in enumerate(['visible.pgm', 'visible.pgm', 'blank.pgm', None])]
    synthetic = args.output/'synthetic.jsonl'
    synthetic.write_text(''.join(json.dumps(row)+'\n' for row in rows), encoding='utf-8')
    # Existing legacy replay calibration, with explicitly NOMINAL identity poses.
    # It lets the real projector execute; its 3D coordinates are NOT archived station truth.
    identity = [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0]
    projection = {'calibration': {'focal_length_m': .0122258, 'sx_m': 3.75202e-6,
                                  'sy_m': 3.75e-6, 'cx_pixel': 672.386, 'cy_pixel': 513.657,
                                  'kappa': -2503.95, 'laser_plane_a': .334870,
                                  'laser_plane_b': 10.819546, 'laser_plane_c': -4.521574},
                  'camera_in_tool': identity, 'tool_in_robot_base': identity,
                  'robot_base_to_positioner_tcp': identity,
                  'provenance': 'diagnostic nominal context from existing legacy test; identity poses NOT exposure poses; no physical 3D accuracy claim'}
    sessions = [('synthetic', synthetic)]
    for frame in [14591, 15248]:
        image = ROOT/'tests/fixtures/online_image_geometry'/f'frame_{frame}.png'
        copy = args.output/image.name
        copy.write_bytes(image.read_bytes())
        manifest = args.output/f'archive-{frame}.jsonl'
        manifest.write_text(json.dumps(dict(image=copy.name, frame_id=frame,
            kind='real', timestamp_ms=None, reference_revision=None,
            projection=projection))+'\n', encoding='utf-8')
        sessions.append((f'archive-{frame}', manifest))
    for name, manifest in sessions:
        subprocess.run([sys.executable, str(HERE/'replay.py'), str(manifest),
                        '--output', str(args.output/name)], check=True)
    (args.output/'index.html').write_text('''<!doctype html><meta charset="utf-8">
<h1>检测入口回放</h1><p>实际 process 图像分支，PCL gap 排除，无设备控制。</p>
<ul><li><a href="synthetic/viewer.html">合成：可见、真实 KCF 更新、黑图、缺图</a></li>
<li><a href="archive-14591/viewer.html">真实像素 14591：独立单帧会话</a></li>
<li><a href="archive-15248/viewer.html">真实像素 15248：独立单帧会话</a></li></ul>
<p>真实像素只验证图像检测行为。投影使用已声明的诊断标定与单位姿态，并非原曝光机器人位姿；三维输出不能作为现场精度证据。时间、参考版本缺失记为未知。两张真实图不拼成连续视频。</p>''', encoding='utf-8')

if __name__ == '__main__':
    main()
