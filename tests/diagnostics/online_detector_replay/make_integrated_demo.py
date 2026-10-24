"""生成一条相连的组合合成序列和独立真实单帧会话；真值仅评分，TCP只作执行记录。"""
import argparse
import json
import pathlib
import subprocess
import sys

HERE=pathlib.Path(__file__).resolve().parent
ROOT=HERE.parents[2]


def raster(x0=360,y0=260,left=True,right=True,cap=False,reflection=False,blank=False):
    pixels=bytearray()
    for y in range(480):
        for x in range(640):
            dx=x-x0
            stripe=not blank and (left if dx<0 else right) and abs(y-(y0-.45*abs(dx)))<1.5
            if cap and abs(dx)<32:
                stripe=abs(y-(y0+13*(1-(dx/32)**2)))<2
            value=150 if stripe else 14
            if reflection and abs(y-(330-.45*abs(x-210)))<1.5:
                value=255
            pixels.append(value)
    return b'P5\n640 480\n255\n'+pixels


def write(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def invoke(folder,frames,truth,tcp,context,policy,cache,control=None):
    (folder/'input.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in frames),encoding='utf-8')
    (folder/'tcp.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in tcp),encoding='utf-8')
    for name,value in [('evaluation',truth),('context',context),('policy',policy),('control-config',control or {})]:
        write(folder/(name+'.json'),value)
    command=[sys.executable,str(HERE/'integrated_replay.py'),str(folder/'input.jsonl'),
        '--context',str(folder/'context.json'),'--evaluation',str(folder/'evaluation.json'),
        '--tcp',str(folder/'tcp.jsonl'),'--policy-config',str(folder/'policy.json'),
        '--control-config',str(folder/'control-config.json'),'--cache',str(cache),'--output',str(folder/'result')]
    subprocess.run(command,check=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=pathlib.Path,required=True)
    parser.add_argument('--cache',type=pathlib.Path,default=ROOT/'output/analysis/detector_replay_build')
    args=parser.parse_args()
    if args.output.exists():
        parser.error('output exists; preserve previous evidence')
    args.output.mkdir(parents=True)
    combined=args.output/'combined-input'
    combined.mkdir()
    # 一次相连会话，正常检测之后才进入遮挡/竞争/参考切换/恢复。
    scenes=[('normal',i*.5,0) for i in range(18)]
    scenes += [('tack_occlusion',8.5,3),('single_side',8.5,3),('reflection_competition',8.5,0),
               ('reference_update',8.5,0),('reference_missing',8.5,0),('reference_missing',8.5,0)]
    scenes += [('recovery',13.5,0)]*9
    scenes += [('normal',13.5+i*.5,0) for i in range(8)]
    frames=[]
    truths={}
    tcp=[]
    for index,(scene,dx,dy) in enumerate(scenes):
        fid=index+1
        image=f'frame-{fid:03}.pgm'
        blank=scene in ('reference_update','reference_missing')
        (combined/image).write_bytes(raster(360+dx,260+dy,right=scene!='single_side',
            cap=scene=='tack_occlusion',reflection=scene=='reflection_competition',blank=blank))
        material=dict(coordinate_frame='PositionerTcp',workpiece_frame='defined-fixture-A',
                      path_id='defined-frozen-seam-A',path_revision=1,s_mm=index*.5)
        frame=dict(image=image,kind='synthetic',frame_id=fid,observation_id='synthetic-'+str(fid),
            timestamp_ms=1000+index*100,projection={'fixture':'synthetic_identity_1mm_per_pixel'},
            material_position=material,weld_progress_mm=index*.5,progress_reference='defined-fixture-A/frozen-seam-A',
            progress_provenance='explicit synthetic exposure-associated material arc, not TCP-derived',
            progress_exposure_frame_id=fid,reference_revision=1 if index<21 else 2,
            reference_point_at_exposure=[360+min(dx,8.5),260],reference_exposure_frame_id=fid)
        if scene=='reference_update':
            frame.update(reference_point_at_exposure=[500,400],previous_reference_revision=1,
                         previous_reference_point_at_exposure=[368.5,260],previous_reference_exposure_frame_id=fid)
        elif index>21:
            frame['reference_point_at_exposure']=[500,400]
        if scene=='reference_missing':
            frame.update(reference_revision=None,reference_point_at_exposure=None,reference_exposure_frame_id=None)
        frames.append(frame)
        # 投影fixture的闭式定义：像素中心(320,240)，Z=1000mm。
        point=[40+dx,20+dy,1000]
        truths[frame['observation_id']]=dict(scenario=scene,pixel=[360+dx,260+dy],
            positioner_tcp_mm=point,material_position=material)
        tcp.append(dict(record_id='synthetic-tcp-'+str(fid),kind='synthetic',timestamp_ms=10000+index*100,
            material_position=material,positioner_tcp_mm=[point[0],point[1]+.25,point[2]],
            provenance='defined execution fixture; not produced by continuous controller and never seam truth'))
    context=dict(coordinate_frame='PositionerTcp',calibration_valid=True,tool_tcp_in_last_link_mm=[0,0,231],
        station_base_translation_mm=[0,0,0],robot_pose_mm_deg=[40,20,1231,0,0,0],axes_deg=[0,0],
        teach_last_link_mm=[[40,20,1231],[80,20,1231]],
        model_positioner_tcp_mm=[[40+i*.5,19.5,1000] for i in range(80)],
        candidate_point_by_frame={str(f['frame_id']):[40+f['material_position']['s_mm'],19.5,1000]
            for f in frames},
        spatial_provenance={'kind':'synthetic_fixture','frame_ids':[f['frame_id'] for f in frames]})
    policy=dict(max_prediction_ms=1600,max_prediction_travel_mm=10,enable_mother_constraints=True,
                recovery_confirmation_frames=2,max_recovery_step_pixel=1,max_reacquisition_distance_pixel=12)
    invoke(combined,frames,{'kind':'synthetic_defined_truth','observations':truths},tcp,context,policy,args.cache,
           {'max_reference_chord_deviation_mm':2.0})
    # 稀疏真实原图分开成会话；缺失的时序/姿态/路径保持null，禁止拼接为连续现场验证。
    links=['<li><a href="combined-input/result/index.html">单一组合合成序列</a></li>']
    for fid in (14591,15248):
        real=args.output/('real-'+str(fid))
        real.mkdir()
        name='frame_'+str(fid)+'.png'
        (real/name).write_bytes((ROOT/'tests/fixtures/online_image_geometry'/name).read_bytes())
        frame=dict(image=name,kind='real',frame_id=fid,observation_id=None,timestamp_ms=None,
                   reference_revision=None,projection=None,material_position=None)
        invoke(real,[frame],{},[],context,policy,args.cache)
        links.append('<li><a href="real-'+str(fid)+'/result/index.html">真实稀疏原图 '+str(fid)+'：空间/材料关联未知</a></li>')
    # 为外部入口保留简单稳定的报告指针，不复制或伪造结果。
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><h1>组合离线验收</h1><p>合成真值、合成TCP执行记录和真实稀疏原图分开。无现场毫米精度结论、无生产修改、无运动。</p><ul>'+''.join(links)+'</ul>',encoding='utf-8')
    write(args.output/'demo-manifest.json',dict(combined='combined-input/result',real=['real-14591/result','real-15248/result'],
        maximum_lateral_deviation_mm=None,continuous_real_sequence_available=False,real_tcp_association_available=False))


if __name__=='__main__':
    main()
