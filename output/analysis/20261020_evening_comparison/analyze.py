"""Read-only evening-run analysis. All outputs are derived; source files are hashed.
Uses the project's previous geometry routines and preserves every selected raw point.
Run with bundled Python (numpy, pandas, Pillow, matplotlib).
"""
from pathlib import Path
import hashlib, html, importlib.util, json, shutil, sys
import numpy as np
import pandas as pd
sys.dont_write_bytecode = True
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
spec=importlib.util.spec_from_file_location('geometry',OUT/'assets/geometry.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
INPUTS={}
def hashed(p):
    INPUTS[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest(); return p

def read(p):return pd.read_csv(hashed(p),sep='\t',low_memory=False)
def dump(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2,default=g.serializable),encoding='utf-8')
def resolve(s):
    s=s.replace('\\','/')
    for key in ['data/','config/']:
        if key in s:return ROOT/key/s.split(key,1)[1]
    raise ValueError(s)
def rot(a):return g.rz(a[2])@g.ry(a[1])@g.rx(a[0])
def stamp(ms):return pd.to_datetime(ms,unit='ms',utc=True).tz_convert('Asia/Shanghai').strftime('%H:%M:%S.%f')[:-3]
def circlefit(q):return g.circle(g.representatives(q,g.circle(q)['center']))
def circlepoints(c):
    n=np.array(c['normal']);u=np.array([1.,0,0])-n*n[0];u/=np.linalg.norm(u);v=np.cross(n,u);a=np.linspace(0,2*np.pi,1441)
    return c['center']+c['radius_mm']*(np.cos(a)[:,None]*u+np.sin(a)[:,None]*v)
def metric(q,ref,dest,times):
    d,p,j,f=g.closest_polyline(q,ref)
    table=pd.DataFrame({'time':times,'distance_mm':d,'reference_segment_index_0based':j,'fraction':f})
    for i,a in enumerate('xyz'):table['source_'+a+'_mm']=q[:,i];table['projected_'+a+'_mm']=p[:,i];table['residual_'+a+'_mm']=q[:,i]-p[:,i]
    table.to_csv(dest,index=False,encoding='utf-8-sig')
    return dict(g.metric(d),max_time=str(np.asarray(times)[d.argmax()])),d

def analyze(session):
    print('Analyzing',session.name,flush=True)
    target=OUT/session.name[:19];target.mkdir(exist_ok=True)
    summary=dict(read(session/'session_summary.txt').itertuples(index=False,name=None))
    r=read(session/'actual_robot_path.txt');d=read(session/'vision_diagnostics.tsv');e=read(session/'realtime_correction.txt');initial=read(session/'initial_correction.txt');b=read(session/'online_icp_batches.txt');follow=read(session/'online_follow_targets.txt')
    workpath=resolve(summary['work_path_file']);calpath=resolve(summary['rbcalib_file']);sensorpath=ROOT/'config/calibration/SENSORPARA.txt'
    work=np.loadtxt(hashed(workpath))[:,:3];hashed(resolve(summary['teach_pose_file']))
    cal=np.array([float(x) for l in hashed(calpath).read_text(encoding='utf-8-sig').splitlines() if l.startswith('////SRANG ') for x in l.split(' ',1)[1].split(',')])
    sr=g.rx(np.deg2rad(-cal[3]))@g.ry(np.deg2rad(-cal[4]))@g.rz(np.deg2rad(-cal[5]));st=-sr@cal[:3]
    vals={l.split()[0]:float(l.split()[1]) for l in hashed(sensorpath).read_text().splitlines() if len(l.split())==2 and not l.startswith(('More','2026'))}
    cr=rot(np.deg2rad([vals[f'pose_CinT[{i}]/deg'] for i in (3,4,5)]));ct=np.array([vals[f'pose_CinT[{i}]/m']*1000 for i in (0,1,2)])
    def project(x,y,states):
        uv=(np.column_stack([x,y])-[vals['cx'],vals['cy']])*[vals['sx/m'],vals['sy/m']]
        uv/=1+vals['Kappa']*(uv*uv).sum(1)[:,None]
        ray=np.c_[uv/vals['f/m'],np.ones(len(uv))];cam=-1000/(ray@np.array([vals['a'],vals['b'],vals['c']]))[:,None]*ray
        tool=cam@cr.T+ct
        xyz=np.array([rot(np.deg2rad([row.robot_rx_deg,row.robot_ry_deg,row.robot_rz_deg]))@q+[row.robot_x_mm,row.robot_y_mm,row.robot_z_mm] for row,q in zip(states.itertuples(),tool)])
        return xyz@sr.T+st
    finalfixed=project(d.final_x_px,d.final_y_px,d)
    projection_error=np.linalg.norm(finalfixed-d[['station_base_x_mm','station_base_y_mm','station_base_z_mm']].to_numpy(),axis=1)
    assert projection_error.max()<2e-6,projection_error.max()
    final=g.external_rotation(finalfixed,d.s1_deg.to_numpy(),d.s2_deg.to_numpy(),inverse=True)
    logged=d[['positioner_tcp_x_mm','positioner_tcp_y_mm','positioner_tcp_z_mm']].to_numpy()+[0,0,231]
    assert np.max(np.linalg.norm(final-logged,axis=1))<3e-6
    rawfixed=project(d.before_fusion_x_px,d.before_fusion_y_px,d)
    raw=g.external_rotation(rawfixed,d.s1_deg.to_numpy(),d.s2_deg.to_numpy(),inverse=True)
    # Existing analysis contract: +231 restores the last-link origin from TCP-relative vision.
    corrected=initial[initial.event.eq('corrected_path_point')].drop_duplicates('point_index')
    aligned_teach=corrected[['aligned_first_x_mm','aligned_first_y_mm','aligned_first_z_mm']].to_numpy()
    startup_delta=np.median(aligned_teach-work,axis=0)
    assert aligned_teach.shape==work.shape and np.max(abs(aligned_teach-work-startup_delta))<2e-6
    mask=r.phase.eq('online_running')&r.corflag.eq(1)&r.detflag.eq(1)&r.s_start.eq(1)&r.s_hold.eq(0)&r.alarm.eq(0)
    tcp=r[mask].copy().reset_index(drop=True)
    tq=g.external_rotation(tcp[['x_mm','y_mm','z_mm']].to_numpy()@sr.T+st,tcp.s1_deg.to_numpy(),tcp.s2_deg.to_numpy(),inverse=True)
    vm=d.phase.eq('online_running')&d.result_valid.eq(1)
    vision=d[vm].copy().reset_index(drop=True);rawq=raw[vm];fq=final[vm]
    assert len(vision)>0 and not d.frame_id.duplicated().any() and d.camera_timestamp_ms.is_monotonic_increasing
    for name,frame,q in [('tcp',tcp,tq),('kcf_raw',vision,rawq),('vision_final',vision,fq)]:
        frame=frame.copy()
        for i,a in enumerate('xyz'):frame[a+'_material_frame_mm']=q[:,i]
        frame.to_csv(target/(name+'_coordinates.csv'),index=False,encoding='utf-8-sig')
    wc=circlefit(work);rc=circlefit(rawq);fc=circlefit(fq)
    align=g.circle_pose_alignment(wc,rc);moved=work@align['rotation'].T+align['translation']
    af=g.circle_pose_alignment(wc,fc);movedfinal=work@af['rotation'].T+af['translation']
    np.savetxt(target/'teach_to_raw_kcf_transform.csv',np.block([[align['rotation'],align['translation'][:,None]],[np.zeros((1,3)),np.ones((1,1))]]),delimiter=',',fmt='%.12f')
    pd.DataFrame(moved,columns=['x_mm','y_mm','z_mm']).to_csv(target/'relocated_teach_material_frame.csv',index=False)
    metrics={};distances={}
    for key,q,ref,times in [('TCP_to_raw_relocated_teach',tq,moved,tcp.time),('raw_KCF_to_relocated_teach',rawq,moved,vision.camera_timestamp_ms.map(stamp)),('final_vision_to_raw_relocated_teach',fq,moved,vision.camera_timestamp_ms.map(stamp)),('TCP_to_final_relocated_teach',tq,movedfinal,tcp.time),('TCP_to_raw_KCF',tq,rawq,tcp.time),('TCP_to_final_vision',tq,fq,tcp.time),('TCP_to_original_teach',tq,work,tcp.time)]:
        metrics[key],distances[key]=metric(q,ref,target/(key+'_distances.csv'),times)
    segments=[]
    for line,ix in tcp.groupby('job_line',sort=False).groups.items():
        ix=np.asarray(list(ix));segments.append(dict(job_line=int(line),n=len(ix),start=str(tcp.time.iloc[ix[0]]),end=str(tcp.time.iloc[ix[-1]]),metric=g.metric(distances['TCP_to_raw_relocated_teach'][ix])))
    vision['elapsed_s']=(vision.camera_timestamp_ms-vision.camera_timestamp_ms.iloc[0])/1000
    vision['dt_ms']=vision.camera_timestamp_ms.diff();vision['state_age_ms']=vision.camera_timestamp_ms-vision.robot_state_received_ms
    for axis in 'xy':
        vision['d_raw_'+axis]=vision['before_fusion_'+axis+'_px'].diff();vision['d_final_'+axis]=vision['final_'+axis+'_px'].diff()
    vision['step_raw_mm']=np.r_[np.nan,np.linalg.norm(np.diff(rawq,axis=0),axis=1)]
    vision['step_final_mm']=np.r_[np.nan,np.linalg.norm(np.diff(fq,axis=0),axis=1)]
    vision['fusion_displacement_mm']=np.linalg.norm(rawq-fq,axis=1)
    vision.to_csv(target/'diagnostics_enriched.csv',index=False,encoding='utf-8-sig')
    jumps=(vision.d_final_x.abs()>5)|(vision.d_final_y.abs()>5)
    vision[jumps].to_csv(target/'jump_events.csv',index=False,encoding='utf-8-sig')
    # Point-to-polyline departure of released ICP targets from their own raw pending batch.
    pts=read(session/'online_icp_points.txt');effects=[]
    for bi,grp in pts.groupby('batch_index',sort=False):
        q=grp[grp.kind.eq('released_target')][['x_mm','y_mm','z_mm']].to_numpy();ref=grp[grp.kind.eq('raw_pending')][['x_mm','y_mm','z_mm']].to_numpy()
        if len(q) and len(ref)>1:
            dd,*_=g.closest_polyline(q,ref);effects.append({'batch_index':bi,'released_n':len(q),**g.metric(dd)})
    effects=pd.DataFrame(effects).merge(b,on='batch_index');effects.to_csv(target/'icp_release_effects.csv',index=False,encoding='utf-8-sig')
    applied=effects[effects.applied.eq(1)]
    # Verify expected counters against physical rows, summary is append-only with last state winning.
    counts={'robot_state_count':len(r),'tracking_result_count':int(e.event.eq('vision_sample').sum()),'vision_diagnostic_count':len(d),'corrected_path_point_count':len(corrected),'icp_batch_count':len(b),'realtime_command_count':int(e.event.eq('command_generated').sum())}
    assert all(int(summary[k])==v for k,v in counts.items())
    manifest=read(session/'vision_images/image_manifest.tsv');from PIL import Image
    for row in manifest.itertuples():
        assert row.status=='saved'
        for kind in ['source','enhanced']:
            p=session/'vision_images'/f'{row.stem}_{kind}.png';hashed(p)
            with Image.open(p) as im:im.verify()
    times=vision.camera_timestamp_ms.to_numpy(); camera_duration=(times[-1]-times[0])/1000
    def coverage(q,c):return float(np.rad2deg(np.ptp(np.unwrap(np.arctan2(q[:,1]-c[1],q[:,0]-c[0])))))
    wait=r[r.phase.eq('waiting_robot_program_stop')]
    waits={'count':len(wait),'s_start_last':int(wait.s_start.iloc[-1]),'alarm_max':int(wait.alarm.max()),'last_job_line':int(wait.job_line.iloc[-1])}
    result=dict(session=session.name,summary=summary,counts=counts,tcp_count=len(tcp),raw_kcf_count=len(rawq),tcp_start=str(tcp.time.iloc[0]),tcp_end=str(tcp.time.iloc[-1]),online_duration_s=camera_duration,online_fps=(len(vision)-1)/camera_duration,metrics=metrics,segments=segments,projection_max_residual_mm=float(projection_error.max()),raw_circle=rc,final_circle=fc,teach_circle=wc,alignment=align,tcp_coverage_deg=coverage(tq,rc['center']),raw_kcf_coverage_deg=coverage(rawq,rc['center']),common_pose_deg=[float(tcp.s1_deg.iloc[0]),float(tcp.s2_deg.iloc[0])],icp_attempts=int(b.attempted.sum()),icp_applied=int(b.applied.sum()),icp_fallbacks=int(b.applied.eq(0).sum()),icp_release_to_raw_max_mm=float(applied.max_mm.max()),icp_release_to_raw_batch_mean_median_mm=float(applied.mean_mm.median()),fusion_frames=int(vision.fusion_applied.sum()),fusion_displacement_mm=g.metric(np.linalg.norm(rawq-fq,axis=1)),raw_vs_final_reference_center_shift_mm=float(np.linalg.norm(rc['center']-fc['center'])),jump_events=int(jumps.sum()),jump_reinitialized=int((jumps&vision.template_reinitialized.eq(1)).sum()),template_reinitialized=int(vision.template_reinitialized.sum()),raw_x_jumps=int(vision.d_raw_x.abs().gt(5).sum()),raw_y_jumps=int(vision.d_raw_y.abs().gt(5).sum()),final_x_jumps=int(vision.d_final_x.abs().gt(5).sum()),final_y_jumps=int(vision.d_final_y.abs().gt(5).sum()),dt_ms={'median':float(vision.dt_ms.median()),'p95':float(vision.dt_ms.quantile(.95)),'max':float(vision.dt_ms.max())},state_age_ms={'median':float(vision.state_age_ms.median()),'p95':float(vision.state_age_ms.quantile(.95)),'max':float(vision.state_age_ms.max()),'negative_count':int(vision.state_age_ms.lt(0).sum())},waiting_stop=waits,work_path=str(workpath.relative_to(ROOT)),image_pairs=len(manifest),image_first_time=stamp(d[d.image_request_status.eq('queued')].camera_timestamp_ms.iloc[0]),image_last_time=stamp(d[d.image_request_status.eq('queued')].camera_timestamp_ms.iloc[-1]))
    dump(target/'results.json',result)
    pd.DataFrame([dict(metric=k,**v) for k,v in metrics.items()]).to_csv(target/'metrics.csv',index=False,encoding='utf-8-sig')
    viewer(target,result,tq,rawq,fq,work,moved)
    print(json.dumps({k:result[k] for k in ['session','tcp_count','raw_kcf_count','online_fps','metrics','icp_applied','icp_release_to_raw_max_mm','jump_events','raw_vs_final_reference_center_shift_mm']},ensure_ascii=False,default=g.serializable),flush=True)
    return result

def viewer(dest,res,tq,rawq,fq,work,moved):
    pose=np.deg2rad(res['common_pose_deg']);rr=g.ry(pose[0])@g.rz(pose[1]);traces=[]
    def trace(q,name,color,mode='lines',visible=True,dash='solid'):
        q=q@rr.T
        traces.append(dict(type='scatter3d',mode=mode,name=name,x=q[:,0].tolist(),y=q[:,1].tolist(),z=q[:,2].tolist(),visible=visible,line=dict(color=color,width=3,dash=dash),marker=dict(color=color,size=2),hovertemplate=name+'<br>X=%{x:.5f} mm<br>Y=%{y:.5f} mm<br>Z=%{z:.5f} mm<extra></extra>'))
    trace(tq,'TCP 实际轨迹（未平滑）','#0072b2');trace(rawq,'KCF 融合前检测点（像素重投影）','#d55e00')
    fit=circlepoints(res['teach_circle']);trace(fit,'原始示教拟合圆','#888888',visible=False,dash='dash');trace(fit@res['alignment']['rotation'].T+res['alignment']['translation'],'移动后示教拟合圆（融合前 KCF 定位）','#009e73',visible=False)
    trace(work,'原始示教实际点与连线','#888888','lines+markers',dash='dash');trace(moved,'移动后示教实际点与连线','#009e73','lines+markers');trace(fq,'控制使用的最终视觉点（融合后）','#9c51b6',visible='legendonly')
    allq=np.vstack([np.c_[t['x'],t['y'],t['z']] for t in traces]);low=allq.min(0);high=allq.max(0);span=high-low;ranges=np.c_[low-span*.06,high+span*.06]
    template=(OUT/'assets/viewer_template.html').read_text()
    template=template.replace('/* PLOTLY_LIBRARY */',(OUT/'assets/plotly-2.35.2.min.js').read_text()).replace('/* TRACE_DATA */',json.dumps(traces,ensure_ascii=False)).replace('/* AXIS_RANGES */',json.dumps(ranges.tolist())).replace('/* AXIS_SPANS */',json.dumps(span.tolist()))
    template=template.replace('<option value="actual">','<option value="actual" selected>').replace('<option value="fit" selected>','<option value="fit">')
    template=template.replace('20261019_110509_938',html.escape(res['session'])).replace('COMMON_POSE',', '.join(f'{x:.4f}°' for x in res['common_pose_deg'])).replace('在线纠偏：四曲线三维观察','在线纠偏：原始检测、融合与 TCP').replace('KCF 原始检测点','KCF 融合前检测点')
    a=template.index('<footer>');z=template.index('</footer>',a)
    template=template[:a]+'<footer><a href="../report.html">返回三组对照报告</a> · TCP '+str(len(tq))+' 点，融合前 KCF '+str(len(rawq))+' 点，最终视觉 '+str(len(fq))+' 点；全部有效在线点保留，无平滑、无异常点剔除。两组示教各 '+str(len(work))+' 点，默认实际点与连线。绿色参考由融合前 KCF 事后整圈圆心/法线刚体定位，保持原半径，不是实时目标或独立真值。橙色由 before_fusion 像素及同帧机器人姿态重投影；紫色为最终视觉点，可由图例开启。图中统一外轴姿态，动态轨迹已回到材料系后再统一显示。</footer>'+template[z+len('</footer>'):]
    (dest/'index.html').write_text(template,encoding='utf-8')
    dump(dest/'viewer_metadata.json',{'trace_counts':[len(t['x']) for t in traces],'common_pose_deg':res['common_pose_deg'],'default_visible':[t['visible'] for t in traces],'coordinate_values_change_with_zscale':False,'reference':'raw before_fusion KCF, rigid circle center/normal alignment, original radius, no fitted yaw','source_session':res['session']})

if __name__=='__main__':
    sessions=sorted((ROOT/'output/online_correction').glob('20261020_2*'))
    results=[analyze(s) for s in sessions]
    dump(OUT/'comparison.json',results)
    rows=[]
    for r in results:
        rows.append(dict(session=r['session'],tcp_n=r['tcp_count'],vision_n=r['raw_kcf_count'],online_fps=r['online_fps'],icp_applied=r['icp_applied'],icp_attempts=r['icp_attempts'],fusion_frames=r['fusion_frames'],jumps=r['jump_events'],**r['metrics']['TCP_to_raw_relocated_teach']))
    pd.DataFrame(rows).to_csv(OUT/'comparison.csv',index=False,encoding='utf-8-sig')
    dump(OUT/'input_hashes.json',INPUTS)
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in INPUTS.items())
    print('All source hashes unchanged.',flush=True)
