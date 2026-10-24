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
SESSION=ROOT/'output/online_correction/20261021_152328_050'
s=dict(read(SESSION/'session_summary.txt').itertuples(index=False,name=None))
r=read(SESSION/'actual_robot_path.txt');d=read(SESSION/'vision_diagnostics.tsv')
work=np.loadtxt(tracked(Path(s['work_path_file'])))[:,:3]
tracked(Path(s['teach_pose_file']))
cal=np.array([float(x) for line in tracked(Path(s['rbcalib_file'])).read_text(encoding='utf-8-sig').splitlines() if line.startswith('////SRANG ') for x in line.split(' ',1)[1].split(',')])
sr=g.rx(np.deg2rad(-cal[3]))@g.ry(np.deg2rad(-cal[4]))@g.rz(np.deg2rad(-cal[5]));st=-sr@cal[:3]
vals={line.split()[0]:float(line.split()[1]) for line in tracked(ROOT/'config/calibration/SENSORPARA.txt').read_text().splitlines() if len(line.split())==2 and not line.startswith(('More','2026'))}
cr=rot(np.deg2rad([vals[f'pose_CinT[{i}]/deg'] for i in (3,4,5)]));ct=np.array([vals[f'pose_CinT[{i}]/m']*1000 for i in (0,1,2)])
def project(x,y,states):
    uv=(np.column_stack([x,y])-[vals['cx'],vals['cy']])*[vals['sx/m'],vals['sy/m']]
    uv/=1+vals['Kappa']*(uv*uv).sum(1)[:,None]
    ray=np.c_[uv/vals['f/m'],np.ones(len(uv))]
    cam=-1000/(ray@np.array([vals['a'],vals['b'],vals['c']]))[:,None]*ray
    tool=cam@cr.T+ct
    rb=np.array([rot(np.deg2rad([row.robot_rx_deg,row.robot_ry_deg,row.robot_rz_deg]))@q+[row.robot_x_mm,row.robot_y_mm,row.robot_z_mm] for row,q in zip(states.itertuples(),tool)])
    return rb@sr.T+st
finalfixed=project(d.final_x_px,d.final_y_px,d)
projection_error=np.linalg.norm(finalfixed-d[['station_base_x_mm','station_base_y_mm','station_base_z_mm']].to_numpy(),axis=1)
assert projection_error.max()<2e-6
final=g.external_rotation(finalfixed,d.s1_deg.to_numpy(),d.s2_deg.to_numpy(),inverse=True)
logged=d[['positioner_tcp_x_mm','positioner_tcp_y_mm','positioner_tcp_z_mm']].to_numpy()+[0,0,231]
logged_error=np.linalg.norm(final-logged,axis=1)
assert logged_error.max()<3e-6
raw=g.external_rotation(project(d.before_fusion_x_px,d.before_fusion_y_px,d),d.s1_deg.to_numpy(),d.s2_deg.to_numpy(),inverse=True)
tm=r.phase.eq('online_running')&r.corflag.eq(1)&r.detflag.eq(1)&r.s_start.eq(1)&r.s_hold.eq(0)&r.alarm.eq(0)
tcp=r[tm].copy().reset_index(drop=True)
tq=g.external_rotation(tcp[['x_mm','y_mm','z_mm']].to_numpy()@sr.T+st,tcp.s1_deg.to_numpy(),tcp.s2_deg.to_numpy(),inverse=True)
vm=d.phase.eq('online_running')&d.result_valid.eq(1)
vis=d[vm].copy().reset_index(drop=True);rawq=raw[vm];fq=final[vm]
assert np.isfinite(tq).all() and np.isfinite(rawq).all() and not d.frame_id.duplicated().any()
wc=fit(work);kc=fit(rawq);tc=fit(tq)
align=g.circle_pose_alignment(wc,kc)
moved=work@align['rotation'].T+align['translation']
assert np.allclose(align['rotation'].T@align['rotation'],np.eye(3),atol=1e-10)
assert np.linalg.det(align['rotation'])>.999999999
assert np.max(abs(np.linalg.norm(np.diff(work,axis=0),axis=1)-np.linalg.norm(np.diff(moved,axis=0),axis=1)))<1e-10
initial=read(SESSION/'initial_correction.txt')
startup=initial[initial.event.eq('corrected_path_point')].drop_duplicates('point_index')[['aligned_first_x_mm','aligned_first_y_mm','aligned_first_z_mm']].to_numpy()
assert startup.shape==work.shape
metrics={};distances={}
def measure(label,q,reference,times):
    error,foot,ix,fraction=g.closest_polyline(q,reference)
    stats=dict(g.metric(error),max_time=stamp(np.asarray(times)[error.argmax()]))
    table=pd.DataFrame({'timestamp_ms':np.asarray(times),'time':[stamp(t) for t in times],'distance_mm':error,'reference_segment_0based':ix,'fraction':fraction})
    for j,a in enumerate('xyz'):
        table['source_'+a+'_mm']=q[:,j];table['foot_'+a+'_mm']=foot[:,j];table['residual_'+a+'_mm']=q[:,j]-foot[:,j]
    table.to_csv(OUT/(label+'.csv'),index=False,encoding='utf-8-sig')
    metrics[label]=stats;distances[label]=error
for args in [('TCP_to_estimated_moved_teach',tq,moved,tcp.timestamp_ms),('raw_KCF_to_estimated_moved_teach',rawq,moved,vis.camera_timestamp_ms),('final_vision_to_estimated_moved_teach',fq,moved,vis.camera_timestamp_ms),('TCP_to_original_teach',tq,work,tcp.timestamp_ms),('TCP_to_startup_teach',tq,startup,tcp.timestamp_ms),('TCP_to_raw_KCF_polyline',tq,rawq, tcp.timestamp_ms)]:measure(*args)
for name,q,times in [('tcp',tq,tcp.timestamp_ms),('raw_kcf',rawq,vis.camera_timestamp_ms),('final_vision',fq,vis.camera_timestamp_ms)]:
    out=pd.DataFrame(q,columns=['x_mm','y_mm','z_mm']);out.insert(0,'timestamp_ms',np.asarray(times));out.to_csv(OUT/(name+'_material_frame.csv'),index=False,encoding='utf-8-sig')
for name,q in [('original_teach',work),('estimated_moved_teach',moved)]:pd.DataFrame(q,columns=['x_mm','y_mm','z_mm']).to_csv(OUT/(name+'.csv'),index=False,encoding='utf-8-sig')
M=np.eye(4);M[:3,:3]=align['rotation'];M[:3,3]=align['translation']
np.savetxt(OUT/'teach_to_estimated_current_transform.csv',M,delimiter=',',fmt='%.12f')
# Separate reference sensitivity from evaluation error. No query points are removed.
alternatives=[]
for name,rr,tt in [('translation_only',np.eye(3),kc['center']-wc['center']),('center_normal_rigid',align['rotation'],align['translation'])]:
    ref=work@rr.T+tt
    for label,q in [('TCP',tq),('raw_KCF',rawq)]:alternatives.append(dict(reference=name,trajectory=label,**g.metric(g.closest_polyline(q,ref)[0])))
# Interleaved 20-degree sectors cover the whole circle in each fit set.
# Hold-out only reduces direct fitting reuse; shared sensor/systematic bias remains.
angle=np.mod(np.arctan2(rawq[:,1]-kc['center'][1],rawq[:,0]-kc['center'][0]),2*np.pi)
fold=(np.floor(np.rad2deg(angle)/20).astype(int)%2)
cv=[];cross_errors=np.empty(len(rawq));reference_displacements=[]
for held in (0,1):
    training=rawq[fold!=held];validation=rawq[fold==held]
    c=fit(training);a=g.circle_pose_alignment(wc,c);ref=work@a['rotation'].T+a['translation']
    error=g.closest_polyline(validation,ref)[0];cross_errors[fold==held]=error
    reference_displacements.append(np.linalg.norm(ref-moved,axis=1))
    cv.append(dict(held_out_fold=held,fit_count=len(training),evaluate_count=len(validation),center=c['center'],normal=c['normal'],**g.metric(error)))
pd.DataFrame({'frame_id':vis.frame_id,'fold':fold,'distance_mm':cross_errors}).to_csv(OUT/'raw_kcf_cross_validation.csv',index=False,encoding='utf-8-sig')
# A point-to-polyline result is cross-track shape error, not time-synchronous tracking lag.
segment_rows=[]
for line,ix in tcp.groupby('job_line',sort=False).groups.items():
    ix=np.array(list(ix));segment_rows.append(dict(job_line=int(line),start=stamp(tcp.timestamp_ms.iloc[ix[0]]),end=stamp(tcp.timestamp_ms.iloc[ix[-1]]),**g.metric(distances['TCP_to_estimated_moved_teach'][ix])))
pd.DataFrame(segment_rows).to_csv(OUT/'tcp_job_line_metrics.csv',index=False,encoding='utf-8-sig')
archive=ROOT/'output/online_local_registration/20261021_072328_003_session_1'
archive_rows=[];verified=[];rawframes={};states=[]
for p in sorted(archive.glob('*.json')):
    obj=json.loads(tracked(p).read_text(encoding='utf-8-sig'))
    if p.name.endswith('.verified.json'):
        if obj.get('succeeded') and obj.get('verified_by_independent_observation'):
            samples=obj.get('post_write_samples',[])
            if samples:
                verified.append(dict(file=p.name,time_ms=int(samples[-1]['capture_time_ms']),count=len(samples),mean_error_mm=float(np.mean([a['current_error_mm'] for a in samples]))))
        continue
    if p.name.endswith('.writeback.json'):
        states.extend([np.array(obj[k]).reshape(4,4)[:3,3] for k in ['before_parent_from_workpiece','pending_parent_from_workpiece']]);continue
    archive_rows.append(dict(file=p.name,accepted=obj['accepted_before_archive'],before_mm=obj['before_residual_mm'],after_mm=obj['after_residual_mm'],reason=obj['reason']))
    for frame in obj['raw_frames']:rawframes[int(frame['frame_id'])]=frame
lookup={int(row.frame_id):i for i,row in enumerate(d.itertuples())}
replay=[]
for fid,f in rawframes.items():
    if fid in lookup and f['independent']['valid']:
        i=lookup[fid];replay.append(np.linalg.norm(raw[i]-[0,0,231]-f['independent']['real_weld_parent_mm']))
assert max(replay)<2e-6
pd.DataFrame(archive_rows).to_csv(OUT/'local_registration_metrics.csv',index=False,encoding='utf-8-sig')
verified.sort(key=lambda x:x['time_ms'])
phase_rows=[]
if verified:
    split=verified[0]['time_ms']
    for name,mask in [('before_first_verified',tcp.timestamp_ms<split),('after_first_verified',tcp.timestamp_ms>=split)]:
        if mask.any():phase_rows.append(dict(phase=name,start=stamp(tcp.timestamp_ms[mask].iloc[0]),end=stamp(tcp.timestamp_ms[mask].iloc[-1]),**g.metric(distances['TCP_to_estimated_moved_teach'][mask])))
angle_sorted=np.sort(np.unique(angle));coverage_deg=360-np.rad2deg(np.diff(np.r_[angle_sorted,angle_sorted[0]+2*np.pi]).max())
result=dict(session=SESSION.name,summary=s,reference_status='Estimated reference, NOT independent ground truth',method='Original teach polyline rigidly translated and plane-normal aligned to angle-balanced raw before_fusion KCF. No scaling, radius change, smoothing, outlier deletion or axial yaw fit.',coordinate_frame='Positioner last-link/material frame. Undo each sample Ry(S1)Rz(S2). Vision parent-TCP coordinates restored by +[0,0,231] mm. Original W is already in this last-link frame.',units='mm',original_count=len(work),tcp_count=len(tq),raw_kcf_count=len(rawq),online_vision_total=int(d.phase.eq('online_running').sum()),raw_kcf_coverage_deg=float(coverage_deg),original_circle=wc,raw_kcf_circle=kc,tcp_circle=tc,alignment=align,radius_difference_kcf_minus_original_mm=kc['radius_mm']-wc['radius_mm'],metrics=metrics,reference_alternatives=alternatives,cross_validation_folds=cv,cross_validation_pooled=g.metric(cross_errors),cross_validation_reference_displacement=g.metric(np.concatenate(reference_displacements)),tcp_phase_metrics=phase_rows,local_registration=dict(batches=len(archive_rows),accepted_candidates=int(sum(a['accepted'] for a in archive_rows)),verified_writebacks=len(verified),verified=verified,model_translation_min=np.min(states,axis=0),model_translation_max=np.max(states,axis=0)),verification=dict(final_projection_max_mm=float(projection_error.max()),positioner_replay_max_mm=float(logged_error.max()),raw_kcf_archive_replay_max_mm=float(max(replay)),raw_kcf_archive_replay_count=len(replay)),limitations=['KCF reference alignment shares observations with KCF evaluation, so systematic pose bias can be absorbed. Cross-validation does not eliminate shared calibration/detector bias.','A rigid transform cannot remove radius or shape differences.','Circle yaw is not identifiable from center/normal; choose the minimal rotation, no yaw optimization.','All valid online samples retained; means are sample weighted, not arc-length weighted.','Nearest point on actual recorded finite polyline; no timestamps paired between TCP and vision; tangential lag is not evaluated.','Current calibration files checked by replay, but no complete historical calibration snapshot exists.','RobotSim pose is an evolving estimator output; no independent measured pose or displaced teaching trajectory was supplied.'])
dump('results.json',result)
pd.DataFrame([dict(metric=k,**v) for k,v in metrics.items()]).to_csv(OUT/'metrics.csv',index=False,encoding='utf-8-sig')
pd.DataFrame(alternatives).to_csv(OUT/'reference_sensitivity.csv',index=False,encoding='utf-8-sig')
# Static plots use actual coordinates; both spatial projections have equal scale.
colors=['#777777','#0072b2','#d55e00','#009e73'];labels=['原始示教','TCP 实测','KCF 融合前','移动后示教（估计参考）']
im=Image.new('RGB',(1800,1250),'white');draw=ImageDraw.Draw(im)
font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',24);small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',19)
draw.text((50,15),'2026-10-21 15:23 复测：四条轨迹与几何误差',fill='#203345',font=font)
draw.text((50,55),'移动后示教为 KCF 定位的估计参考；保留原始形状与半径；不是独立焊缝真值',fill='#526576',font=small)
for i,(color,label) in enumerate(zip(colors,labels)):draw.text((50+i*420,95),label,fill=color,font=small)
def chart(box,series,title,xlabel,ylabel,equal=False):
    x0,y0,x1,y1=box;allq=np.vstack([np.c_[x,y] for x,y,c in series]);lo=allq.min(0);hi=allq.max(0)
    span=np.maximum(hi-lo,1e-6);lo-=span*.04;hi+=span*.04;span=hi-lo
    if equal:
        scale=min((x1-x0)/span[0],(y1-y0)/span[1]);center=(lo+hi)/2
        span=np.array([(x1-x0)/scale,(y1-y0)/scale]);lo=center-span/2
    def xy(x,y):return x0+(np.asarray(x)-lo[0])/span[0]*(x1-x0),y1-(np.asarray(y)-lo[1])/span[1]*(y1-y0)
    draw.text((x0,y0-38),title,fill='#203345',font=font)
    for j in range(6):
        x=x0+j*(x1-x0)/5;y=y1-j*(y1-y0)/5
        draw.line((x,y0,x,y1),fill='#e4e9ed');draw.line((x0,y,x1,y),fill='#e4e9ed')
        draw.text((x-20,y1+6),f'{lo[0]+span[0]*j/5:.1f}',fill='#526576',font=small)
        draw.text((x0-78,y-10),f'{lo[1]+span[1]*j/5:.1f}',fill='#526576',font=small)
    for x,y,c in series:
        xx,yy=xy(x,y);draw.line(list(zip(xx,yy)),fill=c,width=2)
    draw.text(((x0+x1)/2-55,y1+35),xlabel,fill='#526576',font=small)
    draw.text((x0,y0-65),ylabel,fill='#526576',font=small)
for box,(i,j,title) in zip([(125,225,850,625),(1025,225,1750,625)],[(0,1,'XY 投影（等比例）'),(0,2,'XZ 投影（等比例）')]):
    chart(box,[(q[:,i],q[:,j],c) for q,c in zip([work,tq,rawq,moved],colors)],title,'XYZ'[i]+' (mm)','XYZ'[j]+' (mm)',True)
start=min(tcp.timestamp_ms.iloc[0],vis.camera_timestamp_ms.iloc[0])
chart((125,825,850,1165),[((tcp.timestamp_ms-start)/1000,distances['TCP_to_estimated_moved_teach'],colors[1]),((vis.camera_timestamp_ms-start)/1000,distances['raw_KCF_to_estimated_moved_teach'],colors[2])],'全部有效在线点误差','在线时间 (s)','到估计参考折线距离 (mm)')
cdf=[]
for key,label,color in [('TCP_to_estimated_moved_teach','TCP','#0072b2'),('raw_KCF_to_estimated_moved_teach','原始 KCF','#d55e00')]:
    e=np.sort(distances[key]);cdf.append((e,np.arange(1,len(e)+1)/len(e),color))
chart((1025,825,1750,1165),cdf,'误差累计分布','距离 (mm)','累计比例')
im.save(OUT/'preview.png')
# Offline 3D viewer retains every point and uses a common material frame.
traces=[]
for q,label,color in zip([work,tq,rawq,moved],labels,colors):
    traces.append(dict(type='scatter3d',mode='lines',name=label,x=q[:,0].tolist(),y=q[:,1].tolist(),z=q[:,2].tolist(),line=dict(color=color,width=3)))
lib=tracked(ASSETS/'plotly-2.35.2.min.js').read_text(encoding='utf-8')
viewer='<!doctype html><meta charset="utf-8"><title>四轨迹对比</title><style>body{font:16px Microsoft YaHei;margin:20px}#plot{height:85vh}</style><h2>四轨迹对比 · 15:23复测</h2><p>绿色为 KCF 定位的估计参考，未改示教形状/半径；不是独立真值。坐标为变位机末节材料系，单位 mm。等比例三维，可通过图例隐藏轨迹。</p><div id="plot"></div><script>'+lib+'</script><script>Plotly.newPlot("plot",'+json.dumps(traces,ensure_ascii=False)+', {scene:{aspectmode:"data",xaxis:{title:"X (mm)"},yaxis:{title:"Y (mm)"},zaxis:{title:"Z (mm)"}},margin:{t:0}}, {responsive:true});</script>'
(OUT/'index.html').write_text(viewer,encoding='utf-8')
dump('input_hashes.json',HASHES)
print(json.dumps({k:result[k] for k in ['session','tcp_count','raw_kcf_count','original_circle','raw_kcf_circle','tcp_circle','alignment','radius_difference_kcf_minus_original_mm','metrics','cross_validation_pooled','cross_validation_reference_displacement','tcp_phase_metrics','verification']},ensure_ascii=False,indent=2,default=g.serializable))
