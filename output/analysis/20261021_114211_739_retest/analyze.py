"""Replay saved measurements; create derived outputs without changing inputs.
Run with the bundled Python (numpy, pandas, matplotlib). Geometry and viewer
dependencies are existing local analysis assets, hashed with all input records.
"""
from pathlib import Path
import hashlib, importlib.util, json, sys
sys.dont_write_bytecode = True
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
ASSETS = ROOT / 'output/analysis/20261020_evening_comparison/assets'
spec = importlib.util.spec_from_file_location('geometry', ASSETS / 'geometry.py')
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)
HASHES = {}
def tracked(p):
    HASHES[str(p.relative_to(ROOT))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return p
def read(p): return pd.read_csv(tracked(p), sep='\t', low_memory=False)
def dump(p, obj): p.write_text(json.dumps(obj, ensure_ascii=False, indent=2, default=g.serializable), encoding='utf-8')
def rot(a): return g.rz(a[2]) @ g.ry(a[1]) @ g.rx(a[0])
def clock(ms): return pd.to_datetime(ms, unit='ms', utc=True).tz_convert('Asia/Shanghai').strftime('%H:%M:%S.%f')[:-3]
def stats(values,unit):
    v=np.asarray(values,float)
    return dict(n=len(v),unit=unit,mean=float(v.mean()),median=float(np.median(v)),p95=float(np.quantile(v,.95)),maximum=float(v.max()))
def fit(q): return g.circle(g.representatives(q, g.circle(q)['center']))
def circlepoints(c):
    n = c['normal']; u = np.array([1.,0,0]) - n*n[0]; u /= np.linalg.norm(u)
    v = np.cross(n,u); a = np.linspace(0,2*np.pi,1441)
    return c['center'] + c['radius_mm']*(np.cos(a)[:,None]*u+np.sin(a)[:,None]*v)

SESSION = ROOT / 'output/online_correction/20261021_114211_739'
s = dict(read(SESSION/'session_summary.txt').itertuples(index=False,name=None))
r = read(SESSION/'actual_robot_path.txt'); d = read(SESSION/'vision_diagnostics.tsv')
initial = read(SESSION/'initial_correction.txt')
work = np.loadtxt(tracked(Path(s['work_path_file'])))[:,:3]
tracked(Path(s['teach_pose_file']))
cal = np.array([float(x) for line in tracked(Path(s['rbcalib_file'])).read_text(encoding='utf-8-sig').splitlines() if line.startswith('////SRANG ') for x in line.split(' ',1)[1].split(',')])
sr = g.rx(np.deg2rad(-cal[3])) @ g.ry(np.deg2rad(-cal[4])) @ g.rz(np.deg2rad(-cal[5])); st = -sr @ cal[:3]
vals = {line.split()[0]:float(line.split()[1]) for line in tracked(ROOT/'config/calibration/SENSORPARA.txt').read_text().splitlines() if len(line.split())==2 and not line.startswith(('More','2026'))}
cr = rot(np.deg2rad([vals[f'pose_CinT[{i}]/deg'] for i in (3,4,5)]))
ct = np.array([vals[f'pose_CinT[{i}]/m']*1000 for i in (0,1,2)])
def project(x,y,states):
    uv = (np.column_stack([x,y])-[vals['cx'],vals['cy']])*[vals['sx/m'],vals['sy/m']]
    uv /= 1+vals['Kappa']*(uv*uv).sum(1)[:,None]
    ray = np.c_[uv/vals['f/m'],np.ones(len(uv))]
    cam = -1000/(ray@np.array([vals['a'],vals['b'],vals['c']]))[:,None]*ray
    tool = cam@cr.T+ct
    rb = np.array([rot(np.deg2rad([row.robot_rx_deg,row.robot_ry_deg,row.robot_rz_deg]))@q+[row.robot_x_mm,row.robot_y_mm,row.robot_z_mm] for row,q in zip(states.itertuples(),tool)])
    return rb@sr.T+st
reproject = project(d.final_x_px,d.final_y_px,d)
projection_error = np.linalg.norm(reproject-d[['station_base_x_mm','station_base_y_mm','station_base_z_mm']].to_numpy(),axis=1)
assert projection_error.max()<2e-6
final_all = g.external_rotation(reproject,d.s1_deg.to_numpy(),d.s2_deg.to_numpy(),inverse=True)
logged = d[['positioner_tcp_x_mm','positioner_tcp_y_mm','positioner_tcp_z_mm']].to_numpy()+[0,0,231]
assert np.max(np.linalg.norm(final_all-logged,axis=1))<3e-6
raw_all = g.external_rotation(project(d.before_fusion_x_px,d.before_fusion_y_px,d),d.s1_deg.to_numpy(),d.s2_deg.to_numpy(),inverse=True)
tcp = r[r.phase.eq('online_running')&r.corflag.eq(1)&r.detflag.eq(1)&r.s_start.eq(1)&r.s_hold.eq(0)&r.alarm.eq(0)].copy()
tq = g.external_rotation(tcp[['x_mm','y_mm','z_mm']].to_numpy()@sr.T+st,tcp.s1_deg.to_numpy(),tcp.s2_deg.to_numpy(),inverse=True)
vm = d.phase.eq('online_running')&d.result_valid.eq(1)
vis = d[vm].copy(); rawq = raw_all[vm]; fq = final_all[vm]
wc = fit(work); kc = fit(rawq)
alignment = g.circle_pose_alignment(wc,kc)
moved = work@alignment['rotation'].T+alignment['translation']
corrected = initial[initial.event.eq('corrected_path_point')].drop_duplicates('point_index')
startup = corrected[['aligned_first_x_mm','aligned_first_y_mm','aligned_first_z_mm']].to_numpy()
assert startup.shape==work.shape
metrics = {}; distances = {}
for label,q,reference,times in [('TCP_to_relocated_teach',tq,moved,tcp.timestamp_ms),('TCP_to_raw_KCF_polyline',tq,rawq,tcp.timestamp_ms),('raw_KCF_to_relocated_teach',rawq,moved,vis.camera_timestamp_ms),('final_vision_to_relocated_teach',fq,moved,vis.camera_timestamp_ms),('TCP_to_original_teach',tq,work,tcp.timestamp_ms),('TCP_to_startup_aligned_teach',tq,startup,tcp.timestamp_ms)]:
    dist,foot,ix,fraction = g.closest_polyline(q,reference)
    metrics[label] = dict(g.metric(dist),max_time=clock(np.asarray(times)[dist.argmax()]))
    distances[label] = dist
    frame = pd.DataFrame({'timestamp_ms':np.asarray(times),'distance_mm':dist,'segment_index_0based':ix,'fraction':fraction})
    for j,axis in enumerate('xyz'): frame[axis+'_mm']=q[:,j]; frame['foot_'+axis+'_mm']=foot[:,j]
    frame.to_csv(OUT/(label+'.csv'),index=False,encoding='utf-8-sig')
for name,q,frame in [('tcp',tq,tcp),('raw_kcf',rawq,vis),('final_vision',fq,vis)]:
    v = pd.DataFrame(q,columns=['x_mm','y_mm','z_mm']); v.insert(0,'timestamp_ms',np.asarray(frame.timestamp_ms if name=='tcp' else frame.camera_timestamp_ms))
    v.to_csv(OUT/(name+'_material_frame.csv'),index=False)
pd.DataFrame(moved,columns=['x_mm','y_mm','z_mm']).to_csv(OUT/'relocated_teach.csv',index=False)
phase_rows = []
verify_times = []
local_dir = ROOT/'output/online_local_registration/20261021_034211_693_session_1'
local_counts = {}
native_rows = []
di = {int(row.frame_id):i for i,row in enumerate(d.itertuples())}
sim_all = g.external_rotation(project(d.sim_x_px,d.sim_y_px,d),d.s1_deg.to_numpy(),d.s2_deg.to_numpy(),inverse=True)-[0,0,231]
for p in sorted(local_dir.glob('*.json')):
    if p.name.endswith(('.verified.json','.writeback.json')): continue
    obj=json.loads(tracked(p).read_text(encoding='utf-8-sig'))
    reason=obj['reason']; local_counts[reason]=local_counts.get(reason,0)+1
    for f in obj['raw_frames']:
        fid=int(f['frame_id']); ind=f['independent']
        if fid not in di or not ind['valid']: continue
        i=di[fid]; real=np.asarray(ind['real_weld_parent_mm']); virtual=np.asarray(ind['virtual_weld_parent_mm'])
        native_rows.append(dict(frame_id=fid,timestamp_ms=int(d.camera_timestamp_ms.iloc[i]),native_real_virtual_mm=float(np.linalg.norm(real-virtual)),real_replay_error_mm=float(np.linalg.norm(real-(raw_all[i]-[0,0,231]))),sim_pixel_backproject_vs_native_3d_mm=float(np.linalg.norm(sim_all[i]-virtual)),pixel_pair_backproject_distance_mm=float(np.linalg.norm(sim_all[i]-(raw_all[i]-[0,0,231])))))
native_table=pd.DataFrame(native_rows).drop_duplicates('frame_id').sort_values('frame_id')
assert native_table.real_replay_error_mm.max()<2e-6
native_table.to_csv(OUT/'native_2d_3d_consistency.csv',index=False,encoding='utf-8-sig')
for p in sorted(local_dir.glob('*.verified.json')):
    obj=json.loads(tracked(p).read_text(encoding='utf-8-sig'))
    if obj.get('succeeded') and obj.get('verified_by_independent_observation'):
        verify_times.append(int(obj['post_write_samples'][-1]['capture_time_ms']))
assert verify_times
first_verified = min(verify_times)
post_native=native_table[native_table.timestamp_ms>=first_verified]
consistency=dict(sample_count=len(native_table),real_replay_error_max_mm=float(native_table.real_replay_error_mm.max()),sim_pixel_backproject_vs_native_3d=g.metric(native_table.sim_pixel_backproject_vs_native_3d_mm),post_write_native_real_virtual=g.metric(post_native.native_real_virtual_mm),last_reasons=local_counts,warning='Last reason is not an exclusive underlying-failure category; later gates may overwrite earlier reasons.')
dump(OUT/'local_consistency.json',consistency)
for name,mask in [('before_first_verified',tcp.timestamp_ms<first_verified),('after_first_verified',tcp.timestamp_ms>=first_verified)]:
    phase_rows.append(dict(phase=name,start=clock(tcp.timestamp_ms[mask].iloc[0]),end=clock(tcp.timestamp_ms[mask].iloc[-1]),**g.metric(distances['TCP_to_relocated_teach'][mask])))
segments = [dict(job_line=int(line),start=clock(gr.timestamp_ms.iloc[0]),end=clock(gr.timestamp_ms.iloc[-1]),**g.metric(distances['TCP_to_relocated_teach'][tcp.index.get_indexer(gr.index)])) for line,gr in tcp.groupby('job_line',sort=False)]
b = read(SESSION/'online_icp_batches.txt')
follow=read(SESSION/'online_follow_targets.txt')
cur=follow[[f'current_{a}_mm' for a in 'xyz']].to_numpy()
first=follow[[f'first_{a}_mm' for a in 'xyz']].to_numpy()
third=follow[[f'direction_{a}_mm' for a in 'xyz']].to_numpy()
vec=third-first; denom=np.einsum('ij,ij->i',vec,vec)
line_t=np.einsum('ij,ij->i',cur-first,vec)/denom
line_foot=first+line_t[:,None]*vec
line_dist=np.linalg.norm(cur-line_foot,axis=1)
follow_out=pd.DataFrame(dict(timestamp_ms=follow.timestamp_ms,projection_t=line_t,lead_mm=np.linalg.norm(cur-first,axis=1),line_length_mm=np.sqrt(denom),current_to_target_line_mm=line_dist))
follow_out.to_csv(OUT/'follow_geometry.csv',index=False)
mat=b[[f'm{i}{j}' for i in range(4) for j in range(4)]].to_numpy().reshape(-1,4,4)
angles=np.rad2deg(np.arccos(np.clip((np.trace(mat[:,:3,:3],axis1=1,axis2=2)-1)/2,-1,1)))
relative=mat[1:,:3,:3]@mat[:-1,:3,:3].transpose(0,2,1)
angle_step=np.rad2deg(np.arccos(np.clip((np.trace(relative,axis1=1,axis2=2)-1)/2,-1,1)))
e=read(SESSION/'realtime_correction.txt')
receipts=e[e.event.eq('command_receipt')&e.request.fillna('').str.startswith('P ')]
generated=e[e.event.eq('command_generated')]
follow_stats=dict(batches=len(b),attempted=int(b.attempted.sum()),applied=int(b.applied.sum()),rms=g.metric(b.rms_mm),rotation_deg=stats(angles,'deg'),successive_rotation_change_deg=stats(angle_step,'deg'),applied_rotations_above_90_deg=int(((angles>90)&b.applied.eq(1)).sum()),follow_rows=len(follow),all_projection_t_negative=bool((line_t<0).all()),projection_t_minimum=float(line_t.min()),lead_mm=g.metric(np.linalg.norm(cur-first,axis=1)),target_line_length_mm=g.metric(np.sqrt(denom)),current_to_target_line_mm=g.metric(line_dist),generated_commands=len(generated),p_receipts=len(receipts),p_receipt_gap_ms=stats(np.diff(receipts.timestamp_ms),'ms'),generation_gap_ms=stats(np.diff(generated.timestamp_ms),'ms'),p_received_all_success=bool(receipts.sent.eq(1).all()&receipts.response_received.eq(1).all()&receipts.error_code.eq(0).all()),warning='Transform magnitudes are not physical robot/workpiece displacements. Line residuals are internal control geometry, not weld truth. P receiver reuse semantics not established.')
dump(OUT/'follow_diagnostics.json',follow_stats)
res = dict(session=SESSION.name,summary=s,coordinate_frame='positioner last-link/material frame, inverse Ry(S1)Rz(S2); +231mm restores TCP-relative origin for vision',reference='Post-hoc raw KCF circle center/normal alignment of original teach polyline; rigid, no scaling, no axial yaw. Diagnostic reference, not independent truth or online controller target.',metrics=metrics,tcp_n=len(tq),raw_kcf_n=len(rawq),first_verified_time=clock(first_verified),first_verified_timestamp_ms=first_verified,phase_metrics=phase_rows,job_line_metrics=segments,projection_max_error_mm=float(projection_error.max()),fusion_displacement=g.metric(np.linalg.norm(fq-rawq,axis=1)),original_teach_circle=wc,raw_kcf_circle=kc,reference_alignment=alignment,local_consistency=consistency,startup_translation_mm=np.median(startup-work,axis=0),online_icp_attempted=int(b.attempted.sum()),online_icp_applied=int(b.applied.sum()),online_icp_batches=len(b),valid_online_vision_fraction=float(vm.sum()/d.phase.eq('online_running').sum()))
dump(OUT/'results.json',res)
pd.DataFrame([dict(metric=k,**v) for k,v in metrics.items()]).to_csv(OUT/'metrics.csv',index=False,encoding='utf-8-sig')
pd.DataFrame(segments).to_csv(OUT/'job_line_metrics.csv',index=False,encoding='utf-8-sig')

# Preserve every selected original point; fitted geometry is an optional overlay.
pose=np.deg2rad([tcp.s1_deg.iloc[0],tcp.s2_deg.iloc[0]])
rr=g.ry(pose[0])@g.rz(pose[1]); traces=[]
def trace(q,name,color,mode='lines',visible=True,dash='solid'):
    q=q@rr.T
    traces.append(dict(type='scatter3d',mode=mode,name=name,x=q[:,0].tolist(),y=q[:,1].tolist(),z=q[:,2].tolist(),visible=visible,line=dict(color=color,width=3,dash=dash),marker=dict(color=color,size=2),hovertemplate=name+'<br>X=%{x:.5f} mm<br>Y=%{y:.5f} mm<br>Z=%{z:.5f} mm<extra></extra>'))
trace(tq,'TCP 实际轨迹（原始）','#0072b2'); trace(rawq,'KCF 融合前检测（原始）','#d55e00')
cq=circlepoints(wc); trace(cq,'原始示教拟合圆','#888888',visible=False,dash='dash')
trace(cq@alignment['rotation'].T+alignment['translation'],'移动后示教拟合圆','#009e73',visible=False)
trace(work,'原始示教实际点与连线','#888888','lines+markers',dash='dash')
trace(moved,'移动后示教实际点与连线','#009e73','lines+markers')
trace(fq,'融合后视觉点（控制输入）','#9c51b6',visible='legendonly')
allq=np.vstack([np.c_[t['x'],t['y'],t['z']] for t in traces]); low=allq.min(0); high=allq.max(0); span=high-low
template=tracked(ASSETS/'viewer_template.html').read_text(encoding='utf-8')
template=template.replace('/* PLOTLY_LIBRARY */',tracked(ASSETS/'plotly-2.35.2.min.js').read_text(encoding='utf-8')).replace('/* TRACE_DATA */',json.dumps(traces,ensure_ascii=False)).replace('/* AXIS_RANGES */',json.dumps(np.c_[low-span*.06,high+span*.06].tolist())).replace('/* AXIS_SPANS */',json.dumps(span.tolist()))
template=template.replace('<option value="actual">','<option value="actual" selected>').replace('<option value="fit" selected>','<option value="fit">').replace('20261019_110509_938',SESSION.name).replace('COMMON_POSE',f'{tcp.s1_deg.iloc[0]:.4f}° / {tcp.s2_deg.iloc[0]:.4f}°')
a=template.index('<footer>'); z=template.index('</footer>',a)
template=template[:a]+f'<footer>TCP {len(tq)} 点；融合前 KCF {len(rawq)} 点；两组示教各 {len(work)} 点。未平滑、未剔除异常点。绿色为融合前 KCF 事后整圈圆心/法线定位的诊断参考，保持原始半径；不是独立焊缝真值，也不是控制器逐时目标。Z 倍率仅改变显示。禁止 Computer Use，本轮未执行浏览器交互验收。</footer>'+template[z+len('</footer>'):]
(OUT/'index.html').write_text(template,encoding='utf-8')
dump(OUT/'viewer_metadata.json',dict(trace_counts=[len(t['x']) for t in traces],default_visible=[t['visible'] for t in traces],coordinate_values_change_with_zscale=False,reference=res['reference'],ui_verification='not performed: user prohibits Computer Use'))

image=Image.new('RGB',(1600,1150),'white'); draw=ImageDraw.Draw(image)
font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',22)
small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',17)
draw.text((50,20),'2026-10-21 11:42 复测：原始轨迹与纠偏诊断',fill='#203345',font=font)
draw.text((50,55),'绿色参考由融合前 KCF 事后定位；不是独立真值，也不是在线控制器逐时目标',fill='#526576',font=small)
def chart(box,series,title,xlabel,ylabel,equal=False,vertical=None):
    x0,y0,x1,y1=box
    allq=np.vstack([np.c_[x,y] for x,y,color,label in series]); lo=allq.min(0); hi=allq.max(0)
    span=np.maximum(hi-lo,1e-6); lo-=span*.04; hi+=span*.04; span=hi-lo
    if equal:
        scale=min((x1-x0)/span[0],(y1-y0)/span[1]); center=(lo+hi)/2
        span=np.array([(x1-x0)/scale,(y1-y0)/scale]); lo=center-span/2
    def xy(x,y): return (x0+(np.asarray(x)-lo[0])/span[0]*(x1-x0),y1-(np.asarray(y)-lo[1])/span[1]*(y1-y0))
    draw.text((x0,y0-38),title,fill='#203345',font=font)
    for j in range(6):
        x=x0+j*(x1-x0)/5; y=y1-j*(y1-y0)/5
        draw.line((x,y0,x,y1),fill='#e4e9ed'); draw.line((x0,y,x1,y),fill='#e4e9ed')
        draw.text((x-20,y1+6),f'{lo[0]+span[0]*j/5:.0f}',fill='#526576',font=small)
        draw.text((x0-70,y-10),f'{lo[1]+span[1]*j/5:.1f}',fill='#526576',font=small)
    for x,y,color,label in series:
        xx,yy=xy(x,y); draw.line(list(zip(xx,yy)),fill=color,width=2)
    if vertical is not None:
        xx,_=xy(vertical,0); draw.line((xx,y0,xx,y1),fill='#009e73',width=3)
    draw.rectangle(box,outline='#aab8c2'); draw.text((x0,y1+32),xlabel,fill='#526576',font=small)
    draw.text((x0,y0-63),ylabel,fill='#526576',font=small)
t=(tcp.timestamp_ms.to_numpy()-int(s['start_timestamp_ms']))/1000
chart((120,155,1510,350),[(t,distances['TCP_to_relocated_teach'],'#0072b2','TCP')],'TCP 到移动后示教折线（绿色竖线为首次回写通过新帧验证）','会话开始后 / s','最短三维距离 / mm',vertical=(first_verified-int(s['start_timestamp_ms']))/1000)
tv=(vis.camera_timestamp_ms.to_numpy()-int(s['start_timestamp_ms']))/1000
pixel=np.hypot(vis.before_fusion_x_px-vis.sim_x_px,vis.before_fusion_y_px-vis.sim_y_px)
chart((120,490,1510,685),[(tv,pixel,'#d55e00','pixels')],'融合前 KCF 与仿真焊点的图像偏差','会话开始后 / s','像素欧氏距离 / px')
xy=tq@rr.T; kxy=rawq@rr.T; wxy=work@rr.T; mxy=moved@rr.T
chart((120,825,1510,1070),[(xy[:,0],xy[:,1],'#0072b2','TCP'),(kxy[:,0],kxy[:,1],'#d55e00','KCF'),(wxy[:,0],wxy[:,1],'#888888','teach'),(mxy[:,0],mxy[:,1],'#009e73','moved')],'蓝：TCP　橙：KCF　灰：原始示教　绿：事后定位示教','X / mm（统一外轴姿态）','Y / mm',equal=True)
image.save(OUT/'preview.png')
tracked(ASSETS/'geometry.py'); tracked(Path(__file__))
for p in ['UI/src/RuntimeLocalRegistration.cpp','UI/src/OnlineLocalRegistrationWorker.cpp','UI/src/RuntimeWorkerAdapters.cpp','UI/include/OnlineLocalRegistrationVerification.h','DigitalTwin/src/RobotSimDigitalTwinService.cpp','Vision/src/tracking/LegacyOnlineKcfDetector.cpp','Vision/src/geometry/WeldImageToSpaceProjector.cpp','Control/src/OnlineCorrectionPathFollower.cpp','Control/src/OnlineCorrectionExceptionProcessor.cpp','Control/src/OnlineCorrectionTrajectoryPlanner.cpp','config/digital_twin/online_local_registration.ini','docs/online_correction_state_machine.md']:
    if (ROOT/p).is_file(): tracked(ROOT/p)
dump(OUT/'input_hashes.json',HASHES)
assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in HASHES.items())
print(json.dumps({k:res[k] for k in ('tcp_n','raw_kcf_n','metrics','phase_metrics','fusion_displacement','first_verified_time')},ensure_ascii=False,default=g.serializable),flush=True)
print('All tracked input hashes unchanged.',flush=True)
