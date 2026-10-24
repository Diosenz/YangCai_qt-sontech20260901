"""Adapt the established geometry pipeline to the early, reduced recording schema."""
from pathlib import Path
OUT=Path(__file__).resolve().parent
TEMPLATE=OUT.parent/'20261020_202202_auto_kalman_composite_teach'
old=(TEMPLATE/'analyze.py').read_text(encoding='utf-8')
head=old[:old.index('SESSION=')]
head+='''SESSION=next((ROOT/'output/online_correction').glob('20261019_211208_621*'))
s=dict(read(SESSION/'session_summary.txt').itertuples(index=False,name=None))
r=read(SESSION/'actual_robot_path.txt');events=read(SESSION/'realtime_correction.txt');initial=read(SESSION/'initial_correction.txt')
work=np.loadtxt(tracked(Path(s['work_path_file'])))[:,:3]
tracked(Path(s['teach_pose_file']))
cal=np.array([float(x) for line in tracked(Path(s['rbcalib_file'])).read_text(encoding='utf-8-sig').splitlines() if line.startswith('////SRANG ') for x in line.split(' ',1)[1].split(',')])
sr=g.rx(np.deg2rad(-cal[3]))@g.ry(np.deg2rad(-cal[4]))@g.rz(np.deg2rad(-cal[5]));st=-sr@cal[:3]
tm=r.phase.eq('online_running')&r.corflag.eq(1)&r.detflag.eq(1)&r.s_start.eq(1)&r.s_hold.eq(0)&r.alarm.eq(0)
tcp=r[tm].copy().reset_index(drop=True)
tq=g.external_rotation(tcp[['x_mm','y_mm','z_mm']].to_numpy()@sr.T+st,tcp.s1_deg.to_numpy(),tcp.s2_deg.to_numpy(),inverse=True)
vis=events[events.event.eq('vision_sample')&events.phase.eq('online_running')&events.vision_valid.eq(1)].copy().reset_index(drop=True)
# These are logged result coordinates, not before-fusion KCF observations.
rawq=vis[['vision_x_mm','vision_y_mm','vision_z_mm']].to_numpy()+[0.,0.,231.]
assert np.isfinite(tq).all() and np.isfinite(rawq).all()
assert len(r)==int(s['robot_state_count']) and events.event.eq('vision_sample').sum()==int(s['tracking_result_count'])
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
'''
core=old[old.index('PREV='):old.index('# Automatic fusion')]
core=core.replace("('raw_KCF',rawq,vis.camera_timestamp_ms,vis.frame_id),('final_vision',fq,vis.camera_timestamp_ms,vis.frame_id)", "('recorded_vision',rawq,vis.timestamp_ms,vis.index)")
core=core.replace('raw_KCF','recorded_vision')
tail=old[old.index('# Interleaved sector holdout'):old.index('prior=pd.read_csv')].replace('raw_KCF','recorded_vision').replace("vis.frame_id",'vis.index')
tail+='''pd.DataFrame([dict(job_line=int(line),start=stamp(tcp.timestamp_ms.iloc[ix[0]]),end=stamp(tcp.timestamp_ms.iloc[ix[-1]]),**g.metric(datasets['TCP'].distance_mm.iloc[ix])) for line,ix in tcp.groupby('job_line',sort=False).groups.items()]).to_csv(OUT/'tcp_job_line_metrics.csv',index=False,encoding='utf-8-sig')
gaps=np.diff(np.r_[np.sort(angle),np.sort(angle)[0]+360]);wait=r[r.phase.eq('waiting_robot_program_stop')]
result=dict(session=SESSION.name,summary=s,coordinate_frame='变位机末节随动坐标系；日志视觉已在工具TCP系，Z加回231 mm；TCP逐点消除S1/S2旋转',source_counts=dict(robot=len(r),vision_samples=int(events.event.eq('vision_sample').sum()),valid_tcp=len(tcp),valid_recorded_vision=len(vis),waiting_stop_tcp=len(wait),initial_teach_points=len(actual_teach)),recorded_vision_semantics='Recorded visual result, not proven raw KCF; before-fusion pixels, gains, and fusion flags unavailable',original_circle=origin,current_circle=current,fitted_circles=circles,alignment=align,metrics=metrics,components=components,holdout=dict(folds=cv_rows,pooled=g.metric(cv)),reference_sensitivity=sensitivity,coverage_deg=360-float(gaps.max()),visual_endpoint_gap_mm=float(np.linalg.norm(rawq[-1]-rawq[0])),valid_window=dict(tcp_start=stamp(tcp.timestamp_ms.iloc[0]),tcp_end=stamp(tcp.timestamp_ms.iloc[-1]),vision_start=stamp(vis.timestamp_ms.iloc[0]),vision_end=stamp(vis.timestamp_ms.iloc[-1])),verification=dict(tcp_frame_roundtrip_max_mm=tcp_roundtrip,prior_saved_tcp_max_difference_mm=tcp_saved_error,prior_saved_vision_max_difference_mm=vision_saved_error,initial_corrected_path_shape_max_difference_mm=startup_residual),measurement_smoothing=False,reference_status='同一历史综合圆；位姿由本组记录视觉估计；融合状态未知；不是独立真值',limitations=['No diagnostic images, before-fusion pixels, camera timestamps, weights or ICP batch records in this session.','Visual timestamps are recorder times, not verified capture times; no time pairing used.','Session ended with robot-stop handshake timeout. Active-data geometry does not establish successful workflow completion.','Five O command records confirm five initial points; this dataset cannot isolate the causal effect of five P points.'])
dump('results.json',result)
tracked(Path(__file__));tracked(ASSETS/'geometry.py');tracked(OUT.parent/'20261021_152328_050_retest/prior_notion_workflow_snapshot.json')
for code in ['UI/src/OnlineCorrectionRunRecorder.cpp','Vision/src/geometry/WeldProjectionContextBuilder.cpp','Vision/src/geometry/PositionerKinematics.cpp']:tracked(ROOT/code)
dump('input_hashes.json',HASHES)
print(json.dumps(dict(metrics=metrics,window=result['valid_window'],counts=result['source_counts'],coverage_deg=result['coverage_deg'],verification=result['verification']),ensure_ascii=False,indent=2))
'''
head=head.replace('events','runtime_events')
tail=tail.replace("events.event.eq('vision_sample')", "runtime_events.event.eq('vision_sample')")
(OUT/'analyze.py').write_text(head+core+tail,encoding='utf-8')
verify=(TEMPLATE/'verify.py').read_text(encoding='utf-8')
verify=verify[:verify.index("report_hashes=json.loads")]
verify=verify.replace("[r['metrics']['TCP']['n'],r['metrics']['raw_KCF']['n'],r['metrics']['final_vision']['n'],1441,1441,len(a),len(b)]", "[r['metrics']['TCP']['n'],r['metrics']['recorded_vision']['n'],1441,1441,len(a),len(b)]")
verify=verify.replace("['TCP','raw_KCF','final_vision']", "['TCP','recorded_vision']")
verify+='''result=dict(status='passed',input_hashes_verified=len(hashes),independent_scalar_geometry_checks=checks,metrics_and_component_identities='passed',event_counts='passed',viewer_coordinates='passed',rigid_transform='passed',local_links_checked=links,reference_radius_fixed_mm=radius,mouse_used=False,production_code_modified=False)
(OUT/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
'''
(OUT/'verify.py').write_text(verify,encoding='utf-8')
browser=(TEMPLATE/'verify_browser.cjs').read_text(encoding='utf-8').replace("['original-mode',3,5],['relocated-mode',4,6]", "['original-mode',2,4],['relocated-mode',3,5]").replace('axes.traces,9','axes.traces,6')
(OUT/'verify_browser.cjs').write_text(browser,encoding='utf-8')
print('Prepared early-session analysis; no raw/final visual split invented.')
