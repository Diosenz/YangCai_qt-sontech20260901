# Executed by analyze.py in its existing data context.
stage_rows=[]
for key,q,mask in [('coarse_fused',coarse[vm],np.ones(len(vis),dtype=bool)),('geometry_valid',geometry[vm],vis.geometry_valid.eq(1).to_numpy())]:
    q=q[mask];err=g.circle_distance(q,current);ft=feet(q,current)
    stage_rows.append(dict(stage=key,**g.metric(err),above_3mm_count=int((err>3).sum())))
    frame=pd.DataFrame(dict(frame_id=vis.frame_id.to_numpy()[mask],timestamp_ms=vis.camera_timestamp_ms.to_numpy()[mask],x_mm=q[:,0],y_mm=q[:,1],z_mm=q[:,2],distance_mm=err))
    for j,axis in enumerate('xyz'):frame['delta_'+axis+'_mm']=q[:,j]-ft[:,j]
    frame.to_csv(OUT/(key+'_components.csv'),index=False,encoding='utf-8-sig')
pd.DataFrame(stage_rows).to_csv(OUT/'additional_stage_metrics.csv',index=False,encoding='utf-8-sig')
source_rows=[]
for source,group in vis.groupby('point_source'):
    ix=group.index.to_numpy();source_rows.append(dict(source=int(source),name={0:'Unavailable',1:'Geometry',2:'CorrectedKcf',3:'SimulationPrediction',4:'GuidedEstimate'}.get(int(source),'unknown'),**g.metric(final_errors[ix]),raw_kcf_rmse_mm=float(np.sqrt(np.mean(raw_errors[ix]**2))),above_3mm_count=int((final_errors[ix]>3).sum())))
pd.DataFrame(source_rows).to_csv(OUT/'point_source_metrics.csv',index=False,encoding='utf-8-sig')
segments=[]
change=np.r_[0,np.where(np.diff(vis.point_source.to_numpy())!=0)[0]+1,len(vis)]
for a,b in zip(change[:-1],change[1:]):
    segments.append(dict(source=int(vis.point_source.iloc[a]),start=stamp(vis.camera_timestamp_ms.iloc[a]),end=stamp(vis.camera_timestamp_ms.iloc[b-1]),count=b-a,span_ms=int(vis.camera_timestamp_ms.iloc[b-1]-vis.camera_timestamp_ms.iloc[a]),max_mm=float(final_errors[a:b].max())))
pd.DataFrame(segments).to_csv(OUT/'point_source_segments.csv',index=False,encoding='utf-8-sig')
stage_effects=[]
for title,before,after,mask in [('raw_to_coarse',rawq,coarse[vm],np.ones(len(vis),bool)),('raw_to_geometry',rawq,geometry[vm],vis.geometry_valid.eq(1).to_numpy()),('geometry_to_final',geometry[vm],fq,vis.geometry_valid.eq(1).to_numpy()),('raw_to_final',rawq,fq,np.ones(len(vis),bool))]:
    be=g.circle_distance(before[mask],current);ae=g.circle_distance(after[mask],current)
    stage_effects.append(dict(transition=title,n=int(mask.sum()),before_rmse_mm=float(np.sqrt(np.mean(be*be))),after_rmse_mm=float(np.sqrt(np.mean(ae*ae))),movement_mean_mm=float(np.linalg.norm(after[mask]-before[mask],axis=1).mean()),improved=int((ae<be-1e-10).sum()),worsened=int((ae>be+1e-10).sum())))
pd.DataFrame(stage_effects).to_csv(OUT/'stage_effects.csv',index=False,encoding='utf-8-sig')
phase=r.groupby('phase',sort=False).agg(n=('timestamp_ms','size'),start_ms=('timestamp_ms','min'),end_ms=('timestamp_ms','max')).reset_index()
phase['start']=phase.start_ms.map(stamp);phase['end']=phase.end_ms.map(stamp);phase.to_csv(OUT/'robot_phase_timeline.csv',index=False,encoding='utf-8-sig')
extra_summary=dict(stage_metrics=stage_rows,point_sources=source_rows,estimated_count=int(vis.estimated.sum()),transition_limited_count=int(vis.transition_limited.sum()),geometry_valid_count=int(vis.geometry_valid.sum()),stage_effects=stage_effects,raw_stage_semantics='before_fusion coarse KCF; final output includes seam refinement/continuity and fusion',archive_independent_semantics='geometry_x/y original-image refinement, not raw KCF',robot_phases=phase.to_dict('records'))
dump('stage_summary.json',extra_summary)
# Schema7 runtime fields and schema4 registration transaction outcomes.
new_fields=['guided_valid','guided_reacquisition_pending','guided_reacquired','guided_reacquisition_blocked_previous_branch','guided_image_continuity_active','guided_model_translation_suppressed','kcf_training_deferred','kcf_training_committed','kcf_real_search_fallback_used','coarse_observation_used','raw_geometry_valid','online_observation_fusion_enabled']
extra_summary['runtime_flags']={key:int(vis[key].fillna(0).sum()) for key in new_fields}
extra_summary['pose_revisions']=sorted(int(x) for x in vis.workpiece_pose_revision.dropna().unique())
extra_summary['source_labels']={1:'几何精定位',2:'校正KCF估计',3:'预测来源（沿用枚举SimulationPrediction）',4:'原图肩部/时序辅助估计'}
transactions=[]
for path in sorted(folder.glob('*.json')):
    if not path.name.endswith(('.writeback.json','.verified.json','.retained.json')):continue
    obj=json.loads(tracked(path).read_text(encoding='utf-8-sig'));times=[int(x['capture_time_ms']) for x in obj.get('post_write_samples',[]) if 'capture_time_ms' in x]
    transactions.append(dict(file=path.name,record_type=path.name.split('.')[-2],time_ms=max(times) if times else None,phase=obj.get('phase'),succeeded=obj.get('succeeded'),verification_kind=obj.get('verification_kind'),current_pose_geometrically_verified=obj.get('current_pose_geometrically_verified'),retained_without_geometric_verification=obj.get('retained_without_geometric_verification'),verified_by_independent_observation=obj.get('verified_by_independent_observation'),auto_rollback_enabled=obj.get('auto_rollback_enabled'),rollback_attempted=obj.get('rollback_attempted'),workpiece_pose_revision=obj.get('workpiece_pose_revision'),message=obj.get('message')))
pd.DataFrame(transactions).to_csv(OUT/'local_transaction_outcomes.csv',index=False,encoding='utf-8-sig')
extra_summary['transaction_counts']=pd.Series([x['record_type'] for x in transactions]).value_counts().to_dict()
extra_summary['verification_kinds']=pd.Series([x['verification_kind'] for x in transactions]).value_counts().to_dict()
extra_summary['retained_unverified_count']=sum(x['record_type']=='retained' and x['retained_without_geometric_verification'] is True for x in transactions)
extra_summary['surface_only_candidate_count']=0
for path in folder.glob('*.json'):
    if path.name.endswith(('.writeback.json','.verified.json','.retained.json')):continue
    obj=json.loads(path.read_text(encoding='utf-8-sig'));extra_summary['surface_only_candidate_count']+=bool(obj.get('surface_only'))
# Newest run: source3 enum alone does not prove dependence on current simulation.
extra_summary['raw_stage_semantics']='before_fusion粗KCF；最终输出经过几何、图像连续性及辅助估计，不以fusion_applied直接归因'
dump('stage_summary.json',extra_summary)
