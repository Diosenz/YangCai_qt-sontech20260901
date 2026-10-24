"""Prepare two isolated run analyses; all original recordings remain read-only."""
from pathlib import Path
import json
OUT=Path(__file__).resolve().parent
TEMPLATE=OUT.parent/'20261020_202202_auto_kalman_composite_teach'
configs=[dict(prefix='20261021_185548_548',directory='20261021_185548_stability_auto_kalman_composite_teach',label='18:55 稳定保护',archive='20261021_105548_492_session_1'),dict(prefix='20261021_201400_950',directory='20261021_201400_stability10_auto_kalman_composite_teach',label='20:14 稳定保护1.0',archive='20261021_121400_904_session_1')]
extra='''# Local surface registration is separate from the trajectory ICP batches above.
config=json.loads((OUT/'run_config.json').read_text(encoding='utf-8'))
folder=ROOT/'output/online_local_registration'/config['archive'];candidate_rows=[];verified_rows=[];writeback_rows=[];rawframes={}
for path in sorted(folder.glob('*.json')):
    obj=json.loads(tracked(path).read_text(encoding='utf-8-sig'))
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
        replay.append(float(np.linalg.norm(raw[lookup[fid]]-[0,0,231]-frame['independent']['real_weld_parent_mm'])))
assert replay and max(replay)<2e-6
local=dict(archive=config['archive'],candidate_count=len(candidate_rows),schema_versions=sorted(set(x['schema'] for x in candidate_rows)),accepted_count=sum(x['accepted'] for x in candidate_rows),rejected_count=sum(not x['accepted'] for x in candidate_rows),writeback_record_count=len(writeback_rows),verification_record_count=len(verified_rows),verified_success_count=sum(x['succeeded'] and x['independently_verified'] for x in verified_rows),rollback_attempted_count=sum(x['rollback_attempted'] for x in verified_rows),stability_enabled_count=sum(x['stability_enabled'] for x in candidate_rows),stability_evaluated_count=sum(x['stability_evaluated'] for x in candidate_rows),stability_reject_count=sum(x['stability_evaluated'] and not x['stability_accepted'] for x in candidate_rows),base_status_counts=pd.Series([x['base_status'] for x in candidate_rows]).value_counts().to_dict(),evaluated_history_not_enforced_count=sum(x['stability_evaluated'] and x['history_enforced'] is False for x in candidate_rows),evaluated_conflict_count=sum(x['stability_evaluated'] and (x.get('conflicting_region_count') or 0)>0 for x in candidate_rows),raw_archive_replay_count=len(replay),raw_archive_replay_max_mm=max(replay))
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
'''
for config in configs:
    destination=OUT.parent/config['directory'];destination.mkdir(parents=True,exist_ok=True)
    (destination/'run_config.json').write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding='utf-8')
    text=(TEMPLATE/'analyze.py').read_text(encoding='utf-8').replace("glob('20261020_202202_369*')", "glob('"+config['prefix']+"*')")
    text=text.replace("gaps=np.diff(np.r_[np.sort(angle),np.sort(angle)[0]+360])",extra+"\ngaps=np.diff(np.r_[np.sort(angle),np.sort(angle)[0]+360])")
    text=text.replace('archive=archive,verification=', 'archive=archive,local_registration=local,image_sampling=sampling,verification=')
    text=text.replace('tracked(Path(__file__));', "tracked(OUT/'run_config.json');tracked(Path(__file__));")
    (destination/'analyze.py').write_text(text,encoding='utf-8')
    for name in ['verify.py','verify_browser.cjs']:(destination/name).write_text((TEMPLATE/name).read_text(encoding='utf-8'),encoding='utf-8')
(OUT/'run_configs.json').write_text(json.dumps(configs,ensure_ascii=False,indent=2),encoding='utf-8')
print('Prepared separate analyses for both stability recordings.')
