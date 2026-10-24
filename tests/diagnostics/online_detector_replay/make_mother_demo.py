"""Generate raster-only mother-constraint demonstrations and separate truth evaluation."""
import argparse
import html
import json
import math
import pathlib
import subprocess
import sys
from mother_observation import render_viewer

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def raster(left=True,right=True,cap=False,shift_y=0,slope=.45,cap_radius=32,cap_height=13):
    values = bytearray()
    for y in range(480):
        for x in range(640):
            dx=x-360
            mother=260+shift_y-slope*abs(dx)
            stripe=((left if dx<0 else right) and abs(y-mother)<1.5)
            if cap and abs(dx)<cap_radius:
                stripe=abs(y-(260+shift_y+cap_height*(1-(dx/cap_radius)**2)))<2
            values.append(225 if stripe else 14)
    return b'P5\n640 480\n255\n'+values


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=pathlib.Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('output exists; preserve previous evidence')
    args.output.mkdir(parents=True)
    visible=raster()
    cap=raster(cap=True,shift_y=3)
    small=raster(cap=True,cap_radius=12,cap_height=1)
    blank=b'P5\n640 480\n255\n'+bytes([14])*640*480
    cases={
        'normal':([visible,visible,visible],[[360,260]]*3,{}),
        'bilateral-cap':([visible,visible,cap],[[360,260],[360,260],[360,263]],{}),
        'single-side':([visible,visible,raster(right=False,shift_y=3)],[[360,260],[360,260],[360,263]],{}),
        'no-support':([visible,visible,blank],[[360,260]]*3,{}),
        'weak-normals':([raster(slope=.18),raster(slope=.18),raster(cap=True,slope=.18,shift_y=3)],
                        [[360,260],[360,260],[360,263]],{'mother_options':{'min_normal_determinant':.4}}),
        'accepted-extra-corner':([visible,visible,small],[[360,260]]*3,{}),
        'budget-expired':([visible,visible,cap,cap],[[360,260],[360,260],[360,263],[360,263]],{'max_prediction_ms':200})}
    evaluations=[]
    links=[]
    for name,(images,truth,changes)in cases.items():
        folder=args.output/name
        folder.mkdir()
        frames=[]
        for index,image in enumerate(images):
            path=folder/f'frame-{index}.pgm'
            path.write_bytes(image)
            frames.append(dict(image=path.name,kind='synthetic',frame_id=index+1,timestamp_ms=1000+100*index,
                projection={'fixture':'synthetic_identity_1mm_per_pixel'},reference_revision=1,
                reference_point_at_exposure=[100,100],reference_exposure_frame_id=index+1,
                weld_progress_mm=index,progress_reference='defined-workpiece/frozen-path-v1',
                progress_provenance='defined caller exposure-associated arc; not inferred from truth pixels',
                progress_exposure_frame_id=index+1))
        manifest=folder/'input.jsonl'
        manifest.write_text(''.join(json.dumps(frame)+'\n'for frame in frames),encoding='utf-8')
        config=dict(enable_mother_constraints=True,max_prediction_ms=1000,max_prediction_travel_mm=10)
        config.update(changes)
        configuration=folder/'configuration.json'
        configuration.write_text(json.dumps(config),encoding='utf-8')
        output=folder/'replay'
        subprocess.run([sys.executable,str(HERE/'replay.py'),str(manifest),'--evidence',
                        '--policy-config',str(configuration),'--output',str(output)],check=True)
        rows=[json.loads(line)for line in(output/'results.jsonl').read_text().splitlines()]
        (output/'mother_viewer.html').write_text(render_viewer(rows,output),encoding='utf-8')
        estimates=[dict(frame_id=row['frame_id'],source=row['source'],available=row['available'],
            rank=row['mother_observation'].get('support_rank'),center_supported=row['mother_observation'].get('center_supported'),
            error_px=math.dist(row['final_pixel'],target)if row['available']else None,
            anchor_frame=row['anchor']['frame_id']if row['anchor']else None,
            raw_detector_source=row['raw_detector']['source'])for row,target in zip(rows,truth)]
        evaluations.append(dict(case=name,scope='synthetic raster; truth evaluator only',
            total_frames=len(rows),available_frames=sum(row['available']for row in rows),rows=estimates))
        links.append(f'<li><a href="{name}/replay/mother_viewer.html">{html.escape(name)}</a></li>')
    # Sparse archived real pixels: observation only, no invented exposure time or progress.
    real=args.output/'real-sparse'
    real.mkdir()
    calibration={'focal_length_m':.0122258,'sx_m':3.75202e-6,'sy_m':3.75e-6,'cx_pixel':672.386,
                 'cy_pixel':513.657,'kappa':-2503.95,'laser_plane_a':.334870,'laser_plane_b':10.819546,'laser_plane_c':-4.521574}
    identity=[1,0,0,0,1,0,0,0,1,0,0,0]
    projection=dict(calibration=calibration,camera_in_tool=identity,tool_in_robot_base=identity,
                    robot_base_to_positioner_tcp=identity,provenance='diagnostic nominal context; NOT archived exposure poses')
    frames=[]
    for fid in[14591,15248]:
        image=ROOT/'tests/fixtures/online_image_geometry'/f'frame_{fid}.png'
        (real/image.name).write_bytes(image.read_bytes())
        frames.append(dict(image=image.name,kind='real',frame_id=fid,timestamp_ms=None,reference_revision=None,projection=projection))
    (real/'input.jsonl').write_text(''.join(json.dumps(frame)+'\n'for frame in frames),encoding='utf-8')
    (real/'configuration.json').write_text(json.dumps(dict(enable_mother_constraints=True,anchor_confirmation_frames=1,
        max_prediction_ms=1000,max_prediction_travel_mm=10)),encoding='utf-8')
    subprocess.run([sys.executable,str(HERE/'replay.py'),str(real/'input.jsonl'),'--evidence',
        '--policy-config',str(real/'configuration.json'),'--output',str(real/'replay')],check=True)
    real_rows=[json.loads(line)for line in(real/'replay/results.jsonl').read_text().splitlines()]
    (real/'replay/mother_viewer.html').write_text(render_viewer(real_rows,real/'replay'),encoding='utf-8')
    links.append('<li><a href="real-sparse/replay/mother_viewer.html">真实稀疏帧：仅像素证据，不是连续遮挡验证</a></li>')
    evaluations.append(dict(case='real-sparse',scope='archived pixels; nominal poses; no physical ground truth or continuous exposure provenance',
        available=[row['available']for row in real_rows],rank=[row['mother_observation'].get('support_rank')for row in real_rows]))
    (args.output/'evaluation.json').write_text(json.dumps(evaluations,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (args.output/'index.html').write_text('''<!doctype html><meta charset="utf-8"><h1>母材部分约束回放</h1>
<p>合成图和真实归档分开报告。母材线来自当前原图；真值仅在评估器。双侧外推和单侧法向约束均为估计，不刷新真实锚、预测预算或空间执行资格。</p><ul>'''+''.join(links)+'''</ul>
<p><a href="evaluation.json">误差与可用率</a>含初次确认期间不可用帧。真实两张图存在较长且未知时间间隔，不是连续序列，不能用于真实预焊遮挡精度宣称。观测器不输出预焊类别判断，也不能从任意亮线确认母材身份。</p>''',encoding='utf-8')

if __name__=='__main__':
    main()
