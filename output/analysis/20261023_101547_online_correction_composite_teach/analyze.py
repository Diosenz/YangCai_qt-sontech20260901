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
def stamp(t):return pd.to_datetime(t,unit='ms',utc=True).tz_convert('Asia/Shanghai').strftime('%m-%d %H:%M:%S.%f')[:-3]
SESSION=next((ROOT/'output/online_correction').glob('20261023_101547_678*'))
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
geometry=g.external_rotation(project(d.geometry_x_px,d.geometry_y_px,d),d.s1_deg.to_numpy(),d.s2_deg.to_numpy(),inverse=True)
coarse=g.external_rotation(project(d.coarse_final_x_px,d.coarse_final_y_px,d),d.s1_deg.to_numpy(),d.s2_deg.to_numpy(),inverse=True)
raw=g.external_rotation(project(d.before_fusion_x_px,d.before_fusion_y_px,d),d.s1_deg.to_numpy(),d.s2_deg.to_numpy(),inverse=True)
tm=r.phase.eq('online_running')&r.corflag.eq(1)&r.detflag.eq(1)&r.s_start.eq(1)&r.s_hold.eq(0)&r.alarm.eq(0)
tcp=r[tm].copy().reset_index(drop=True)
tq=g.external_rotation(tcp[['x_mm','y_mm','z_mm']].to_numpy()@sr.T+st,tcp.s1_deg.to_numpy(),tcp.s2_deg.to_numpy(),inverse=True)
vm=d.phase.eq('online_running')&d.result_valid.eq(1)
vis=d[vm].copy().reset_index(drop=True);rawq=raw[vm];fq=final[vm]
assert np.isfinite(tq).all() and np.isfinite(rawq).all() and not d.frame_id.duplicated().any()
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
for label,q,times,ids in [('TCP',tq,tcp.timestamp_ms,tcp.index),('raw_KCF',rawq,vis.camera_timestamp_ms,vis.frame_id),('final_vision',fq,vis.camera_timestamp_ms,vis.frame_id)]:
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
# Automatic fusion is described from actual recorded gains, not assumed fixed ratios.
fusion_delta=np.linalg.norm(fq-rawq,axis=1)
pixel_error=float(np.max(abs(vis[['before_fusion_x_px','before_fusion_y_px']].to_numpy()-vis[['final_x_px','final_y_px']].to_numpy())))
weight_stats={}
for key in ['kcf_weight_x','kcf_weight_y','prior_weight_x','prior_weight_y','normalized_peak','observation_noise']:
    values=vis[key].dropna()
    weight_stats[key]=dict(n=len(values),min=float(values.min()),median=float(values.median()),max=float(values.max())) if len(values) else dict(n=0)
raw_errors=datasets['raw_KCF'].distance_mm.to_numpy();final_errors=datasets['final_vision'].distance_mm.to_numpy()
improved=final_errors<raw_errors-1e-10;worse=final_errors>raw_errors+1e-10
fusion=dict(applied_count=int(vis.fusion_applied.sum()),valid_online_count=len(vis),weights_available_count=int(vis.kcf_weight_x.notna().sum()),raw_final_pixel_max_px=pixel_error,movement_mm=g.metric(fusion_delta),geometry_rmse_change_mm=metrics['final_vision']['rmse_mm']-metrics['raw_KCF']['rmse_mm'],correlation_jump_rejected=int(vis.correlation_jump_rejected.sum()),fusion_jump_rejected=int(vis.fusion_jump_rejected.sum()),weight_stats=weight_stats,improved_count=int(improved.sum()),worsened_count=int(worse.sum()),unchanged_count=int((~improved&~worse).sum()),template_reinitialized_count=int(vis.template_reinitialized.sum()),kalman_seeded_count=int(vis.kalman_seeded.sum()))
assert fusion['applied_count']>0
pd.DataFrame(dict(frame_id=vis.frame_id,time=[stamp(t) for t in vis.camera_timestamp_ms],fusion_applied=vis.fusion_applied,kcf_weight_x=vis.kcf_weight_x,kcf_weight_y=vis.kcf_weight_y,fusion_shift_mm=fusion_delta,raw_error_mm=raw_errors,final_error_mm=final_errors,error_change_mm=final_errors-raw_errors)).to_csv(OUT/'fusion_effects.csv',index=False,encoding='utf-8-sig')
# Interleaved sector holdout tests reference placement sensitivity, not independent truth.
angle=np.mod(np.rad2deg(np.arctan2(rawq[:,1]-kc['center'][1],rawq[:,0]-kc['center'][0])),360)
fold=(np.floor(angle/20).astype(int)%2);cv=np.empty(len(rawq));cv_rows=[]
for held in (0,1):
    fitted=fit(rawq[fold!=held]);candidate=dict(center=fitted['center'],normal=fitted['normal'],radius_mm=origin['radius_mm'])
    errors=g.circle_distance(rawq[fold==held],candidate);cv[fold==held]=errors
    cv_rows.append(dict(held_out_fold=held,training_count=int((fold!=held).sum()),center=fitted['center'],normal=fitted['normal'],**g.metric(errors)))
pd.DataFrame(dict(frame_id=vis.frame_id,fold=fold,distance_mm=cv)).to_csv(OUT/'kcf_sector_holdout.csv',index=False,encoding='utf-8-sig')
sensitivity=[]
for omitted in range(4):
    rad=np.mean([a['circle']['radius_mm'] for i,a in enumerate(historical['sources']) if i!=omitted])
    for key in ['TCP','raw_KCF']:
        q=datasets[key][['x_mm','y_mm','z_mm']].to_numpy();sensitivity.append(dict(omitted_source=historical['sources'][omitted]['name'],radius_mm=rad,trajectory=key,**g.metric(g.circle_distance(q,dict(current,radius_mm=rad)))))
pd.DataFrame(sensitivity).to_csv(OUT/'leave_one_teaching_out_sensitivity.csv',index=False,encoding='utf-8-sig')
prior=pd.read_csv(tracked(OUT.parent/'20261021_235225_seam_refinement_composite_teach/metrics.csv'));comparison=[]
for key in metrics:
    before=prior[prior.trajectory.eq(key)].iloc[0]
    comparison.append(dict(trajectory=key,previous_mean_mm=before.mean_mm,current_mean_mm=metrics[key]['mean_mm'],mean_change_mm=metrics[key]['mean_mm']-before.mean_mm,previous_rmse_mm=before.rmse_mm,current_rmse_mm=metrics[key]['rmse_mm'],rmse_change_mm=metrics[key]['rmse_mm']-before.rmse_mm,previous_max_mm=before.max_mm,current_max_mm=metrics[key]['max_mm']))
pd.DataFrame(comparison).to_csv(OUT/'previous_run_comparison.csv',index=False,encoding='utf-8-sig')
# These are trajectory prediction batches, not local surface registration/writeback.
batches=read(SESSION/'online_icp_batches.txt');points=read(SESSION/'online_icp_points.txt');effects=[]
for bi,grp in points.groupby('batch_index',sort=False):
    released=grp[grp.kind.eq('released_target')][['x_mm','y_mm','z_mm']].to_numpy()
    pending=grp[grp.kind.eq('raw_pending')][['x_mm','y_mm','z_mm']].to_numpy()
    if len(released) and len(pending)>1:
        effects.append(dict(batch_index=int(bi),released_count=len(released),**g.metric(g.closest_polyline(released,pending)[0])))
effects=pd.DataFrame(effects).merge(batches,on='batch_index')
effects.to_csv(OUT/'trajectory_icp_effects.csv',index=False,encoding='utf-8-sig')
applied=effects[effects.applied.eq(1)]
archive=dict(batch_count=len(batches),attempted=int(batches.attempted.sum()),applied=int(batches.applied.sum()),not_applied=int(batches.applied.eq(0).sum()),enabled=int(batches.enabled.sum()),released_departure_max_mm=float(applied.max_mm.max()),batch_mean_departure_median_mm=float(applied.mean_mm.median()),semantic='online trajectory prediction ICP, not surface-localization SET_POSE evidence')
assert len(batches)==int(s['icp_batch_count'])
pd.DataFrame([dict(job_line=int(line),start=stamp(tcp.timestamp_ms.iloc[ix[0]]),end=stamp(tcp.timestamp_ms.iloc[ix[-1]]),**g.metric(datasets['TCP'].distance_mm.iloc[ix])) for line,ix in tcp.groupby('job_line',sort=False).groups.items()]).to_csv(OUT/'tcp_job_line_metrics.csv',index=False,encoding='utf-8-sig')
# Local surface registration is separate from the trajectory ICP batches above.
config=json.loads((OUT/'run_config.json').read_text(encoding='utf-8'))
folder=ROOT/'output/online_local_registration'/config['archive'];candidate_rows=[];verified_rows=[];writeback_rows=[];rawframes={}
for path in sorted(folder.glob('*.json')):
    obj=json.loads(tracked(path).read_text(encoding='utf-8-sig'))
    if path.name.endswith('.retained.json'):
        continue
    if path.name.endswith('.verified.json'):
        samples=obj.get('post_write_samples',[]);good=obj.get('verified_good_observations',[])
        times=[int(x['capture_time_ms']) for x in samples if 'capture_time_ms' in x]
        verified_rows.append(dict(file=path.name,succeeded=bool(obj.get('succeeded')),independently_verified=bool(obj.get('verified_by_independent_observation')),phase=obj.get('phase'),rollback_attempted=bool(obj.get('rollback_attempted')),samples=len(samples),good_samples=len(good),time_ms=max(times) if times else None,time=stamp(max(times)) if times else '',cumulative_translation_mm=obj.get('cumulative_translation_mm'),cumulative_rotation_deg=obj.get('cumulative_rotation_deg'),maximum_keypoint_displacement_mm=obj.get('maximum_keypoint_displacement_mm'),message=obj.get('message')))
        continue
    if path.name.endswith('.writeback.json'):
        writeback_rows.append(dict(file=path.name,**{key:value for key,value in obj.items() if not isinstance(value,(dict,list))}));continue
    st=obj.get('stability_protection',{});evaluated=bool(st.get('evaluated'))
    if 'base_safety_accepted' in obj:base='recorded_pass' if obj['base_safety_accepted'] else 'recorded_reject'
    else:base='inferred_pass_from_stability_evaluated' if evaluated else 'not_recorded_not_inferred'
    row=dict(file=path.name,schema=obj.get('schema_version'),history_policy_version=obj.get('history_policy_version'),frame_id=int(obj['frame_id']),accepted=bool(obj.get('accepted_before_archive')),base_status=base,before_mm=obj.get('before_residual_mm'),after_mm=obj.get('after_residual_mm'),before_independent_mm=obj.get('before_independent_error_mm'),after_independent_mm=obj.get('after_independent_error_mm'),solve_elapsed_ms=int(obj.get('solve_elapsed_ms',0)),stability_enabled=bool(st.get('enabled')),stability_evaluated=evaluated,stability_accepted=bool(st.get('accepted')) if evaluated else None,history_enforced=st.get('history_enforced') if evaluated else None,before_mean_weld_error_mm=st.get('before_mean_weld_error_mm') if evaluated else None,after_mean_weld_error_mm=st.get('after_mean_weld_error_mm') if evaluated else None,required_gain_mm=st.get('required_gain_mm') if evaluated else None,history_max_predicted_mm=st.get('maximum_historical_error_mm') if evaluated else None,region_count=st.get('region_count',len(st.get('anchors',[]))) if evaluated else None,conflicting_region_count=st.get('conflicting_region_count') if evaluated else None,evidence_status=st.get('evidence_status') if evaluated else None,reason=obj.get('reason'),stability_reason=st.get('reason'))
    candidate_rows.append(row)
    for frame in obj.get('raw_frames',[]):rawframes[int(frame['frame_id'])]=frame
pd.DataFrame(candidate_rows).to_csv(OUT/'local_registration_metrics.csv',index=False,encoding='utf-8-sig')
pd.DataFrame(verified_rows).to_csv(OUT/'local_verification_records.csv',index=False,encoding='utf-8-sig')
dump('local_writeback_records.json',writeback_rows)
lookup={int(row.frame_id):i for i,row in enumerate(d.itertuples())};replay=[]
for fid,frame in rawframes.items():
    if fid in lookup and frame.get('independent',{}).get('valid'):
        replay.append(float(np.linalg.norm(geometry[lookup[fid]]-[0,0,231]-frame['independent']['real_weld_parent_mm'])))
assert not replay or max(replay)<2e-6
local=dict(archive=config['archive'],candidate_count=len(candidate_rows),schema_versions=sorted(set(x['schema'] for x in candidate_rows)),accepted_count=sum(x['accepted'] for x in candidate_rows),rejected_count=sum(not x['accepted'] for x in candidate_rows),writeback_record_count=len(writeback_rows),verification_record_count=len(verified_rows),verified_success_count=sum(x['succeeded'] and x['independently_verified'] for x in verified_rows),rollback_attempted_count=sum(x['rollback_attempted'] for x in verified_rows),stability_enabled_count=sum(x['stability_enabled'] for x in candidate_rows),stability_evaluated_count=sum(x['stability_evaluated'] for x in candidate_rows),stability_reject_count=sum(x['stability_evaluated'] and not x['stability_accepted'] for x in candidate_rows),base_status_counts=pd.Series([x['base_status'] for x in candidate_rows]).value_counts().to_dict(),evaluated_history_not_enforced_count=sum(x['stability_evaluated'] and x['history_enforced'] is False for x in candidate_rows),evaluated_conflict_count=sum(x['stability_evaluated'] and (x.get('conflicting_region_count') or 0)>0 for x in candidate_rows),raw_archive_replay_count=len(replay),raw_archive_replay_max_mm=max(replay) if replay else None)
dump('local_registration_summary.json',local)
phase_rows=[];successful=[x['time_ms'] for x in verified_rows if x['succeeded'] and x['independently_verified'] and x['time_ms'] is not None]
if successful:
    boundary=min(successful)
    for key in ['TCP','raw_KCF','final_vision']:
        frame=datasets[key]
        for label,mask in [('before_first_verified_update',frame.timestamp_ms<boundary),('after_first_verified_update',frame.timestamp_ms>=boundary)]:
            if mask.any():phase_rows.append(dict(trajectory=key,phase=label,boundary_ms=boundary,boundary_time=stamp(boundary),**g.metric(frame.distance_mm[mask])))
pd.DataFrame(phase_rows).to_csv(OUT/'first_verified_update_windows.csv',index=False,encoding='utf-8-sig')
# Verify image inventory and select actual saved images nearest trajectory peaks.
manifest=read(SESSION/'vision_images/image_manifest.tsv');image_rows=[];image_candidates=[]
from PIL import Image
for row in manifest.itertuples():
    assert row.status=='saved'
    fid=int(str(row.stem).rsplit('_frame_',1)[1]);matches=d[d.frame_id.eq(fid)];assert len(matches)==1
    source_row=matches.iloc[0]
    for kind in ['source','enhanced']:
        path=tracked(SESSION/'vision_images'/(str(row.stem)+'_'+kind+'.png'))
        with Image.open(path) as im:im.verify()
    image_candidates.append(dict(frame_id=fid,stem=str(row.stem),camera_timestamp_ms=int(source_row.camera_timestamp_ms),phase=source_row.phase,bucket=int(source_row.image_sample_bucket) if 'image_sample_bucket' in d else None))
for key in ['TCP','raw_KCF','final_vision']:
    frame=datasets[key];peak=frame.loc[frame.distance_mm.idxmax()];chosen=min(image_candidates,key=lambda item:abs(item['camera_timestamp_ms']-peak.timestamp_ms))
    image_rows.append(dict(trajectory=key,peak_time=stamp(peak.timestamp_ms),peak_timestamp_ms=int(peak.timestamp_ms),peak_error_mm=float(peak.distance_mm),image_time=stamp(chosen['camera_timestamp_ms']),time_offset_ms=int(chosen['camera_timestamp_ms']-peak.timestamp_ms),**chosen))
pd.DataFrame(image_rows).to_csv(OUT/'nearest_saved_peak_images.csv',index=False,encoding='utf-8-sig')
sampling=dict(pairs=len(manifest),image_diagnostic_schema=s.get('vision_diagnostics_schema'),buckets=pd.Series([x['bucket'] for x in image_candidates]).value_counts().to_dict() if 'image_sample_bucket' in d else {},first_saved_time=stamp(min(x['camera_timestamp_ms'] for x in image_candidates)),last_saved_time=stamp(max(x['camera_timestamp_ms'] for x in image_candidates)))
dump('image_sampling_summary.json',sampling)

exec(compile(tracked(OUT/'stage_analysis.py').read_text(encoding='utf-8-sig'),str(OUT/'stage_analysis.py'),'exec'))
gaps=np.diff(np.r_[np.sort(angle),np.sort(angle)[0]+360])
result=dict(session=SESSION.name,summary=s,coordinate_frame='变位机末节随动坐标系（材料系），逐点消除S1/S2旋转；与变位机工具TCP系相差Z=231 mm',source_counts=dict(robot=len(r),diagnostics=len(d),valid_tcp=len(tcp),valid_raw_kcf=len(vis)),original_circle=origin,current_circle=current,fitted_circles=circles,alignment=align,metrics=metrics,components=components,fusion=fusion,comparison=comparison,holdout=dict(folds=cv_rows,pooled=g.metric(cv)),reference_sensitivity=sensitivity,coverage_deg=360-float(gaps.max()),archive=archive,local_registration=local,image_sampling=sampling,stages=extra_summary,verification=dict(final_projection_max_mm=float(projection_error.max()),positioner_replay_max_mm=float(logged_error.max()),raw_final_pixel_max_px=pixel_error,raw_final_spatial_max_mm=float(fusion_delta.max())),workflow_source='Prior saved Notion page snapshots dated 2026-10-19 and 2026-10-18; saved workflow reused; live Notion not refreshed this run',measurement_smoothing=False,reference_status='固定历史综合圆；位姿由本轮融合前KCF事后估计；不是独立真值')
dump('results.json',result)
tracked(OUT/'run_config.json');tracked(Path(__file__));tracked(ASSETS/'geometry.py');tracked(PREV/'prior_notion_workflow_snapshot.json') if (PREV/'prior_notion_workflow_snapshot.json').exists() else tracked(OUT.parent/'20261021_152328_050_retest/prior_notion_workflow_snapshot.json')
for code in ['Vision/src/track/OnlineSeamContinuity.cpp','Vision/src/track/LegacyOnlineKcfDetector.cpp','UI/src/RuntimeWorkerAdapters.cpp','Vision/src/geometry/WeldProjectionContextBuilder.cpp','Vision/src/geometry/PositionerKinematics.cpp','Vision/src/track/KcfTracker.cpp','Control/src/OnlineCorrectionExceptionProcessor.cpp','docs/online_local_registration.md']:
    tracked(ROOT/code)
dump('input_hashes.json',HASHES)
print(json.dumps(dict(metrics=metrics,fusion=fusion,archive=result['archive'],verification=result['verification']),ensure_ascii=False,indent=2))
