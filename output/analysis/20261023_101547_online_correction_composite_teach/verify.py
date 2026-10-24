"""Independent scalar geometry, saved-data, provenance and report-link checks."""
from pathlib import Path
import hashlib,json,re
import numpy as np
import pandas as pd
OUT=Path(__file__).resolve().parent
r=json.loads((OUT/'results.json').read_text(encoding='utf-8'))
hashes=json.loads((OUT/'input_hashes.json').read_text(encoding='utf-8'))
for name,digest in hashes.items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name
origin=r['original_circle'];c=r['current_circle'];center=np.array(c['center']);normal=np.array(c['normal']);radius=c['radius_mm']
assert radius==origin['radius_mm']==142.03533963525268
assert abs(normal@normal-1)<1e-12
M=np.loadtxt(OUT/'teach_to_current_transform.csv',delimiter=',');a=pd.read_csv(OUT/'original_composite_teach.csv').to_numpy();b=pd.read_csv(OUT/'moved_composite_teach.csv').to_numpy()
assert np.max(abs(a@M[:3,:3].T+M[:3,3]-b))<2e-10
assert np.allclose(M[:3,:3].T@M[:3,:3],np.eye(3),atol=1e-12)
checks=0
for key,stats in r['metrics'].items():
    d=pd.read_csv(OUT/(key+'_circle_error_components.csv'));q=d[['x_mm','y_mm','z_mm']].to_numpy();foot=d[['foot_x_mm','foot_y_mm','foot_z_mm']].to_numpy();e=d.distance_mm.to_numpy();delta=d[['delta_x_mm','delta_y_mm','delta_z_mm']].to_numpy()
    assert len(q)==stats['n']
    assert abs(e.mean()-stats['mean_mm'])<1e-12 and abs(np.sqrt(np.mean(e*e))-stats['rmse_mm'])<1e-12 and abs(e.max()-stats['max_mm'])<1e-12
    assert np.max(abs(q-foot-delta))<1e-12
    assert np.max(abs(np.linalg.norm(delta,axis=1)-e))<1e-12
    for ix in np.unique(np.r_[np.linspace(0,len(q)-1,31,dtype=int),e.argmax()]):
        v=q[ix]-center;h=float(sum(v[j]*normal[j] for j in range(3)));plane=[v[j]-h*normal[j] for j in range(3)];rho=sum(x*x for x in plane)**.5
        expected=((rho-radius)**2+h*h)**.5
        assert abs(expected-e[ix])<1e-11
        reconstructed=center+radius*np.array(plane)/rho
        assert np.linalg.norm(reconstructed-foot[ix])<1e-10
        checks+=1
    assert int((e>3).sum())==stats['above_3mm_count']
    events=pd.read_csv(OUT/'over_3mm_events.csv');assert events[events.trajectory.eq(key)]['count'].sum()==stats['above_3mm_count']
    assert np.all(np.diff(d.timestamp_ms)>=0)
component=pd.read_csv(OUT/'error_component_summary.csv')
for row in component.itertuples():
    m=r['metrics'][row.trajectory]
    assert abs(row.radial_rmse_mm**2+row.plane_rmse_mm**2-m['rmse_mm']**2)<1e-12
    assert abs(row.delta_x_rmse_mm**2+row.delta_y_rmse_mm**2+row.delta_z_rmse_mm**2-m['rmse_mm']**2)<1e-12
v=json.loads((OUT/'viewer_data.json').read_text(encoding='utf-8'))
assert [len(t['x']) for t in v['traces']]==[r['metrics']['TCP']['n'],r['metrics']['raw_KCF']['n'],r['metrics']['final_vision']['n'],1441,1441,len(a),len(b)]
for i,key in enumerate(['TCP','raw_KCF','final_vision']):
    d=pd.read_csv(OUT/(key+'_circle_error_components.csv'));assert np.max(abs(np.c_[v['traces'][i]['x'],v['traces'][i]['y'],v['traces'][i]['z']]-d[['x_mm','y_mm','z_mm']].to_numpy()))<1e-11
links=0
for name in ['report.html','index.html','axis_fluctuations.html']:
    source=(OUT/name).read_text(encoding='utf-8')
    for target in re.findall(r'(?:src|href)="([^"]+)"',source):
        if target.startswith(('http:','https:','#')):continue
        if target=='verification.json':continue
        assert (OUT/target).is_file(),target
        links+=1
    assert '\ufffd' not in source
report_hashes=json.loads((OUT/'report_input_hashes.json').read_text(encoding='utf-8'))
for name,digest in report_hashes.items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name
raw=pd.read_csv(OUT/'raw_KCF_circle_error_components.csv');final=pd.read_csv(OUT/'final_vision_circle_error_components.csv');fusion=pd.read_csv(OUT/'fusion_effects.csv')
assert np.array_equal(raw.sample_id,final.sample_id) and np.array_equal(raw.sample_id,fusion.frame_id)
assert np.array_equal(raw.timestamp_ms,final.timestamp_ms)
movement=np.linalg.norm(raw[['x_mm','y_mm','z_mm']].to_numpy()-final[['x_mm','y_mm','z_mm']].to_numpy(),axis=1)
assert np.max(abs(movement-fusion.fusion_shift_mm))<1e-10
changes=final.distance_mm.to_numpy()-raw.distance_mm.to_numpy()
assert np.max(abs(changes-fusion.error_change_mm))<1e-12
assert int((changes< -1e-10).sum())==r['fusion']['improved_count']
assert int((changes>1e-10).sum())==r['fusion']['worsened_count']
assert int(fusion.fusion_applied.sum())==r['fusion']['applied_count']
assert abs(movement.mean()-r['fusion']['movement_mm']['mean_mm'])<1e-12
local=r['local_registration'];local_table=pd.read_csv(OUT/'local_registration_metrics.csv');verified_table=pd.read_csv(OUT/'local_verification_records.csv')
assert len(local_table)==local['candidate_count']
assert int(local_table.accepted.sum())==local['accepted_count']
assert int(local_table.stability_evaluated.sum())==local['stability_evaluated_count']
assert int((local_table.stability_evaluated&local_table.stability_accepted.eq(False)).sum())==local['stability_reject_count']
assert int((verified_table.succeeded&verified_table.independently_verified).sum())==local['verified_success_count']
assert len(verified_table)==local['verification_record_count']
cfg=json.loads((OUT/'run_config.json').read_text(encoding='utf-8'));archive=OUT.parents[1]/'online_local_registration'/cfg['archive']
raw_table=pd.read_csv(OUT/'geometry_valid_components.csv');by_id={int(row.frame_id):row for row in raw_table.itertuples()}
local_source_count=local_source_accepted=local_source_verified=sampled_archive_coordinates=0
for path in archive.glob('*.json'):
    obj=json.loads(path.read_text(encoding='utf-8-sig'))
    if path.name.endswith('.verified.json'):
        local_source_verified+=bool(obj.get('succeeded')) and bool(obj.get('verified_by_independent_observation'));continue
    if path.name.endswith(('.writeback.json','.retained.json')):continue
    local_source_count+=1;local_source_accepted+=bool(obj.get('accepted_before_archive'))
    st=obj.get('stability_protection',{})
    if not st.get('evaluated'):
        row=local_table[local_table.file.eq(path.name)].iloc[0];assert pd.isna(row.history_max_predicted_mm)
    if obj.get('schema_version')==3:
        assert 'base_safety_accepted' in obj
        if st.get('evaluated'):assert obj['base_safety_accepted']
    frames=obj.get('raw_frames',[])
    for item in frames[::max(1,len(frames)//3)]:
        fid=int(item['frame_id'])
        if fid in by_id and item.get('independent',{}).get('valid'):
            row=by_id[fid];point=np.array([row.x_mm,row.y_mm,row.z_mm])-[0,0,231]
            assert np.linalg.norm(point-item['independent']['real_weld_parent_mm'])<2e-6
            sampled_archive_coordinates+=1
assert local_source_count==local['candidate_count'] and local_source_accepted==local['accepted_count'] and local_source_verified==local['verified_success_count']
image_table=pd.read_csv(OUT/'nearest_saved_peak_images.csv');session=OUT.parents[1]/'online_correction'/r['session']
for row in image_table.itertuples():
    assert row.time_offset_ms==row.camera_timestamp_ms-row.peak_timestamp_ms
    for kind in ['source','enhanced']:
        name=row.stem+'_'+kind+'.png'
        assert (OUT/'evidence'/name).read_bytes()==(session/'vision_images'/name).read_bytes()

stages=json.loads((OUT/'stage_summary.json').read_text(encoding='utf-8'))
for item in stages['stage_metrics']:
    frame=pd.read_csv(OUT/(item['stage']+'_components.csv'));delta=frame[['x_mm','y_mm','z_mm']].to_numpy()-center
    h=delta@normal;rho=np.linalg.norm(delta-h[:,None]*normal,axis=1);expected=np.sqrt((rho-radius)**2+h*h)
    assert len(frame)==item['n'] and np.max(abs(expected-frame.distance_mm.to_numpy()))<1e-10
    assert abs(np.sqrt(np.mean(expected**2))-item['rmse_mm'])<1e-10
    assert int((expected>3).sum())==item['above_3mm_count']
diag=pd.read_csv(session/'vision_diagnostics.tsv',sep='	');valid=diag[diag.phase.eq('online_running')&diag.result_valid.eq(1)].reset_index(drop=True)
for item in stages['point_sources']:
    mask=valid.point_source.eq(item['source']).to_numpy();errors=final.distance_mm.to_numpy()[mask]
    assert len(errors)==item['n'] and abs(np.sqrt(np.mean(errors**2))-item['rmse_mm'])<1e-10
    assert int((errors>3).sum())==item['above_3mm_count']
assert int(valid.estimated.sum())==stages['estimated_count']
assert np.array_equal(valid.estimated.eq(1),valid.point_source.isin([2,3,4]))
assert r['summary']['session_state']=='reset'

transactions=pd.read_csv(OUT/'local_transaction_outcomes.csv')
assert transactions.record_type.value_counts().to_dict()=={'writeback':27,'retained':18,'verified':9}
assert int(transactions[transactions.record_type.eq('retained')].retained_without_geometric_verification.sum())==18
assert transactions[transactions.record_type.eq('verified')].current_pose_geometrically_verified.all()
assert not transactions[transactions.record_type.eq('retained')].current_pose_geometrically_verified.any()
browser=json.loads((OUT/'browser_verification.json').read_text(encoding='utf-8'));assert browser['status']=='passed' and not browser['mouse_used']

result=dict(status='passed',input_hashes_verified=len(hashes),report_source_hashes_verified=len(report_hashes),independent_scalar_geometry_checks=checks,metrics_and_component_identities='passed',fusion_same_frame_pairing_and_effects='passed',local_source_candidates_verified=local_source_count,local_independent_coordinate_checks=sampled_archive_coordinates,local_postverification_successes=local_source_verified,saved_image_copy_and_timing_checks='passed',browser_checks='passed',event_counts='passed',viewer_coordinates='passed',rigid_transform='passed',local_links_checked=links,reference_radius_fixed_mm=radius,mouse_used=False,production_code_modified=False,new_stage_and_point_source_checks='passed',archive_observation='original image geometry, not coarse KCF')
(OUT/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
