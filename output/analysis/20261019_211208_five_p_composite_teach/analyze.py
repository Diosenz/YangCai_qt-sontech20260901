"""Read-only inputs; derived trajectory metrics and reports. No device access."""
from pathlib import Path
import hashlib, importlib.util, json, sys, html
sys.dont_write_bytecode=True
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
ASSETS=ROOT/'output/analysis/20261020_evening_comparison/assets'
spec=importlib.util.spec_from_file_location('geometry',ASSETS/'geometry.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
HASHES={}
def tracked(p):
    p=Path(p);HASHES[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest();return p
def read(p):return pd.read_csv(tracked(p),sep='\t',low_memory=False)
def dump(name,v):
    (OUT/name).write_text(json.dumps(v,ensure_ascii=False,indent=2,default=g.serializable),encoding='utf-8')
def rot(a):return g.rz(a[2])@g.ry(a[1])@g.rx(a[0])
def fit(q):return g.circle(g.representatives(q,g.circle(q)['center']))
def stamp(t):return pd.to_datetime(t,unit='ms',utc=True).tz_convert('Asia/Shanghai').strftime('%H:%M:%S.%f')[:-3]
SESSION=next((ROOT/'output/online_correction').glob('20261019_211208_621*'))
s=dict(read(SESSION/'session_summary.txt').itertuples(index=False,name=None))
r=read(SESSION/'actual_robot_path.txt');runtime_events=read(SESSION/'realtime_correction.txt');initial=read(SESSION/'initial_correction.txt')
work=np.loadtxt(tracked(Path(s['work_path_file'])))[:,:3]
tracked(Path(s['teach_pose_file']))
cal=np.array([float(x) for line in tracked(Path(s['rbcalib_file'])).read_text(encoding='utf-8-sig').splitlines() if line.startswith('////SRANG ') for x in line.split(' ',1)[1].split(',')])
sr=g.rx(np.deg2rad(-cal[3]))@g.ry(np.deg2rad(-cal[4]))@g.rz(np.deg2rad(-cal[5]));st=-sr@cal[:3]
tm=r.phase.eq('online_running')&r.corflag.eq(1)&r.detflag.eq(1)&r.s_start.eq(1)&r.s_hold.eq(0)&r.alarm.eq(0)
tcp=r[tm].copy().reset_index(drop=True)
tq=g.external_rotation(tcp[['x_mm','y_mm','z_mm']].to_numpy()@sr.T+st,tcp.s1_deg.to_numpy(),tcp.s2_deg.to_numpy(),inverse=True)
vis=runtime_events[runtime_events.event.eq('vision_sample')&runtime_events.phase.eq('online_running')&runtime_events.vision_valid.eq(1)].copy().reset_index(drop=True)
# These are logged result coordinates, not before-fusion KCF observations.
rawq=vis[['vision_x_mm','vision_y_mm','vision_z_mm']].to_numpy()+[0.,0.,231.]
assert np.isfinite(tq).all() and np.isfinite(rawq).all()
assert len(r)==int(s['robot_state_count']) and runtime_events.event.eq('vision_sample').sum()==int(s['tracking_result_count'])
roundtrip=g.external_rotation(tq,tcp.s1_deg.to_numpy(),tcp.s2_deg.to_numpy())
tcp_roundtrip=float(np.max(np.linalg.norm(roundtrip-(tcp[['x_mm','y_mm','z_mm']].to_numpy()@sr.T+st),axis=1)))
assert tcp_roundtrip<1e-9
corrected=initial[initial.event.eq('corrected_path_point')].drop_duplicates('point_index')
offsets=corrected[['tcp_offset_x_mm','tcp_offset_y_mm','tcp_offset_z_mm']].to_numpy()
assert np.allclose(offsets,[0,0,231],atol=1e-9)
actual_teach=initial[initial.event.eq('teach_point')].copy();actual_teach.to_csv(OUT/'five_p_initial_commands.csv',index=False,encoding='utf-8-sig')
assert len(actual_teach)==int(s['initial_command_count'])==5
startup=corrected[['aligned_first_x_mm','aligned_first_y_mm','aligned_first_z_mm']].to_numpy()
assert startup.shape==work.shape
startup_delta=np.median(startup-work,axis=0);startup_residual=float(np.max(np.linalg.norm(startup-work-startup_delta,axis=1)))
assert startup_residual<2e-6
# Read-only cross-check of coordinates against the previous independently saved analysis.
archive=OUT.parent/'20261019_211208_621_station_base'
previous_tcp=pd.read_csv(tracked(archive/'tcp_coordinates.csv'));previous_vision=pd.read_csv(tracked(archive/'kcf_coordinates.csv'))
assert len(previous_tcp)==len(tq) and len(previous_vision)==len(rawq)
tcp_saved_error=float(np.max(abs(previous_tcp[[a+'_material_frame_mm' for a in 'xyz']].to_numpy()-tq)))
vision_saved_error=float(np.max(abs(previous_vision[[a+'_material_frame_mm' for a in 'xyz']].to_numpy()-rawq)))
assert tcp_saved_error<1e-9 and vision_saved_error<1e-9
PREV=OUT.parent/'20261021_positive20_teach_synthesis'
origin=json.loads(tracked(PREV/'synthesized_original_circle.json').read_text(encoding='utf-8'))
origin={k:np.array(v) if isinstance(v,list) else v for k,v in origin.items()}
historical=json.loads(tracked(PREV/'results.json').read_text(encoding='utf-8'))
assert abs(origin['radius_mm']-142.03533963525268)<1e-12
kc=fit(rawq);align=g.circle_pose_alignment(origin,kc)
current=dict(center=kc['center'],normal=kc['normal'],radius_mm=origin['radius_mm'])
M=np.eye(4);M[:3,:3]=align['rotation'];M[:3,3]=align['translation']
np.savetxt(OUT/'teach_to_current_transform.csv',M,delimiter=',',fmt='%.12f')
synth=np.loadtxt(tracked(PREV/'20261021CHUFENGKOU20_SYNTH_ANALYSIS_W.txt'))[:,:3]
moved=synth@align['rotation'].T+align['translation']
for name,q in [('original_composite_teach',synth),('moved_composite_teach',moved)]:
    pd.DataFrame(q,columns=['x_mm','y_mm','z_mm']).to_csv(OUT/(name+'.csv'),index=False,encoding='utf-8-sig')
dump('synthesized_original_circle.json',origin);dump('moved_synthesized_circle.json',current)
metrics={};components=[];sectors=[];events=[];datasets={};circles={}
def feet(q,c):
    delta=q-c['center'];h=delta@c['normal'];plane=delta-h[:,None]*c['normal'];rho=np.linalg.norm(plane,axis=1)
    assert (rho>0).all()
    return c['center']+c['radius_mm']*plane/rho[:,None]
u=np.array([1.,0,0])-kc['normal']*kc['normal'][0];u/=np.linalg.norm(u);v=np.cross(kc['normal'],u)
for label,q,times,ids in [('TCP',tq,tcp.timestamp_ms,tcp.index),('recorded_vision',rawq,vis.timestamp_ms,vis.index)]:
    times=np.asarray(times);foot=feet(q,current);delta=q-foot;e=np.linalg.norm(delta,axis=1)
    assert np.max(abs(g.circle_distance(q,current)-e))<1e-10
    h=(q-kc['center'])@kc['normal'];rho=np.linalg.norm(np.cross(q-kc['center'],kc['normal']),axis=1);radial=rho-origin['radius_mm']
    a=dict(g.metric(e),within_2mm_fraction=float(np.mean(e<=2)),within_3mm_fraction=float(np.mean(e<=3)),above_3mm_count=int((e>3).sum()),max_time=stamp(times[e.argmax()]))
    metrics[label]=a;circles[label]=fit(q)
    frame=pd.DataFrame(dict(timestamp_ms=times,time=[stamp(t) for t in times],sample_id=np.asarray(ids),distance_mm=e,signed_radial_mm=radial,signed_plane_height_mm=h))
    for j,axis in enumerate('xyz'):
        frame[axis+'_mm']=q[:,j];frame['foot_'+axis+'_mm']=foot[:,j];frame['delta_'+axis+'_mm']=delta[:,j]
    angle=np.mod(np.rad2deg(np.arctan2((q-kc['center'])@v,(q-kc['center'])@u)),360);frame['angle_deg']=angle
    frame.to_csv(OUT/(label+'_circle_error_components.csv'),index=False,encoding='utf-8-sig');datasets[label]=frame
    components.append(dict(trajectory=label,radial_mean_mm=float(radial.mean()),radial_rmse_mm=float(np.sqrt(np.mean(radial**2))),plane_mean_mm=float(h.mean()),plane_rmse_mm=float(np.sqrt(np.mean(h*h)))))
    for j,axis in enumerate('xyz'):
        components[-1]['delta_'+axis+'_rmse_mm']=float(np.sqrt(np.mean(delta[:,j]**2)))
    for i in range(12):
        mask=(angle>=30*i)&(angle<30*(i+1))
        if mask.any():sectors.append(dict(trajectory=label,sector_start_deg=30*i,**g.metric(e[mask])))
    indices=np.where(e>3)[0]
    for group in np.split(indices,np.where((np.diff(indices)>1)|(np.diff(times[indices])>200))[0]+1):
        if not len(group):continue
        peak=group[e[group].argmax()]
        events.append(dict(trajectory=label,start=stamp(times[group[0]]),end=stamp(times[group[-1]]),count=len(group),span_ms=int(times[group[-1]]-times[group[0]]),max_mm=float(e[peak]),max_time=stamp(times[peak]),sample_id=int(np.asarray(ids)[peak]),radial_at_peak_mm=float(radial[peak]),plane_at_peak_mm=float(h[peak])))
pd.DataFrame([dict(trajectory=k,**a) for k,a in metrics.items()]).to_csv(OUT/'metrics.csv',index=False,encoding='utf-8-sig')
pd.DataFrame(components).to_csv(OUT/'error_component_summary.csv',index=False,encoding='utf-8-sig')
pd.DataFrame(sectors).to_csv(OUT/'angular_sector_metrics.csv',index=False,encoding='utf-8-sig')
pd.DataFrame(events,columns=['trajectory','start','end','count','span_ms','max_mm','max_time','sample_id','radial_at_peak_mm','plane_at_peak_mm']).to_csv(OUT/'over_3mm_events.csv',index=False,encoding='utf-8-sig')
# Interleaved sector holdout tests reference placement sensitivity, not independent truth.
angle=np.mod(np.rad2deg(np.arctan2(rawq[:,1]-kc['center'][1],rawq[:,0]-kc['center'][0])),360)
fold=(np.floor(angle/20).astype(int)%2);cv=np.empty(len(rawq));cv_rows=[]
for held in (0,1):
    fitted=fit(rawq[fold!=held]);candidate=dict(center=fitted['center'],normal=fitted['normal'],radius_mm=origin['radius_mm'])
    errors=g.circle_distance(rawq[fold==held],candidate);cv[fold==held]=errors
    cv_rows.append(dict(held_out_fold=held,training_count=int((fold!=held).sum()),center=fitted['center'],normal=fitted['normal'],**g.metric(errors)))
pd.DataFrame(dict(frame_id=vis.index,fold=fold,distance_mm=cv)).to_csv(OUT/'kcf_sector_holdout.csv',index=False,encoding='utf-8-sig')
sensitivity=[]
for omitted in range(4):
    rad=np.mean([a['circle']['radius_mm'] for i,a in enumerate(historical['sources']) if i!=omitted])
    for key in ['TCP','recorded_vision']:
        q=datasets[key][['x_mm','y_mm','z_mm']].to_numpy();sensitivity.append(dict(omitted_source=historical['sources'][omitted]['name'],radius_mm=rad,trajectory=key,**g.metric(g.circle_distance(q,dict(current,radius_mm=rad)))))
pd.DataFrame(sensitivity).to_csv(OUT/'leave_one_teaching_out_sensitivity.csv',index=False,encoding='utf-8-sig')
pd.DataFrame([dict(job_line=int(line),start=stamp(tcp.timestamp_ms.iloc[ix[0]]),end=stamp(tcp.timestamp_ms.iloc[ix[-1]]),**g.metric(datasets['TCP'].distance_mm.iloc[ix])) for line,ix in tcp.groupby('job_line',sort=False).groups.items()]).to_csv(OUT/'tcp_job_line_metrics.csv',index=False,encoding='utf-8-sig')
gaps=np.diff(np.r_[np.sort(angle),np.sort(angle)[0]+360]);wait=r[r.phase.eq('waiting_robot_program_stop')]
result=dict(session=SESSION.name,summary=s,coordinate_frame='变位机末节随动坐标系；日志视觉已在工具TCP系，Z加回231 mm；TCP逐点消除S1/S2旋转',source_counts=dict(robot=len(r),vision_samples=int(runtime_events.event.eq('vision_sample').sum()),valid_tcp=len(tcp),valid_recorded_vision=len(vis),waiting_stop_tcp=len(wait),initial_teach_points=len(actual_teach)),recorded_vision_semantics='Recorded visual result, not proven raw KCF; before-fusion pixels, gains, and fusion flags unavailable',original_circle=origin,current_circle=current,fitted_circles=circles,alignment=align,metrics=metrics,components=components,holdout=dict(folds=cv_rows,pooled=g.metric(cv)),reference_sensitivity=sensitivity,coverage_deg=360-float(gaps.max()),visual_endpoint_gap_mm=float(np.linalg.norm(rawq[-1]-rawq[0])),valid_window=dict(tcp_start=stamp(tcp.timestamp_ms.iloc[0]),tcp_end=stamp(tcp.timestamp_ms.iloc[-1]),vision_start=stamp(vis.timestamp_ms.iloc[0]),vision_end=stamp(vis.timestamp_ms.iloc[-1])),verification=dict(tcp_frame_roundtrip_max_mm=tcp_roundtrip,prior_saved_tcp_max_difference_mm=tcp_saved_error,prior_saved_vision_max_difference_mm=vision_saved_error,initial_corrected_path_shape_max_difference_mm=startup_residual),measurement_smoothing=False,reference_status='同一历史综合圆；位姿由本组记录视觉估计；融合状态未知；不是独立真值',limitations=['No diagnostic images, before-fusion pixels, camera timestamps, weights or ICP batch records in this session.','Visual timestamps are recorder times, not verified capture times; no time pairing used.','Session ended with robot-stop handshake timeout. Active-data geometry does not establish successful workflow completion.','Five O command records confirm five initial points; this dataset cannot isolate the causal effect of five P points.'])
dump('results.json',result)
tracked(Path(__file__));tracked(ASSETS/'geometry.py');tracked(OUT.parent/'20261021_152328_050_retest/prior_notion_workflow_snapshot.json')
for code in ['UI/src/OnlineCorrectionRunRecorder.cpp','Vision/src/geometry/WeldProjectionContextBuilder.cpp','Vision/src/geometry/PositionerKinematics.cpp']:tracked(ROOT/code)
dump('input_hashes.json',HASHES)
print(json.dumps(dict(metrics=metrics,window=result['valid_window'],counts=result['source_counts'],coverage_deg=result['coverage_deg'],verification=result['verification']),ensure_ascii=False,indent=2))
