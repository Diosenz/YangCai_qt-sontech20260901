"""Extend independent saved-artifact checks for local registration and image evidence."""
from pathlib import Path
import json
OUT=Path(__file__).resolve().parent
TEMPLATE=OUT.parent/'20261020_202202_auto_kalman_composite_teach'
addition='''local=r['local_registration'];local_table=pd.read_csv(OUT/'local_registration_metrics.csv');verified_table=pd.read_csv(OUT/'local_verification_records.csv')
assert len(local_table)==local['candidate_count']
assert int(local_table.accepted.sum())==local['accepted_count']
assert int(local_table.stability_evaluated.sum())==local['stability_evaluated_count']
assert int((local_table.stability_evaluated&local_table.stability_accepted.eq(False)).sum())==local['stability_reject_count']
assert int((verified_table.succeeded&verified_table.independently_verified).sum())==local['verified_success_count']
assert len(verified_table)==local['verification_record_count']
cfg=json.loads((OUT/'run_config.json').read_text(encoding='utf-8'));archive=OUT.parents[1]/'online_local_registration'/cfg['archive']
raw_table=pd.read_csv(OUT/'raw_KCF_circle_error_components.csv');by_id={int(row.sample_id):row for row in raw_table.itertuples()}
local_source_count=local_source_accepted=local_source_verified=sampled_archive_coordinates=0
for path in archive.glob('*.json'):
    obj=json.loads(path.read_text(encoding='utf-8-sig'))
    if path.name.endswith('.verified.json'):
        local_source_verified+=bool(obj.get('succeeded')) and bool(obj.get('verified_by_independent_observation'));continue
    if path.name.endswith('.writeback.json'):continue
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
browser=json.loads((OUT/'browser_verification.json').read_text(encoding='utf-8'));assert browser['status']=='passed' and not browser['mouse_used']
'''
for cfg in json.loads((OUT/'run_configs.json').read_text(encoding='utf-8')):
    directory=OUT.parent/cfg['directory'];text=(TEMPLATE/'verify.py').read_text(encoding='utf-8')
    text=text.replace('result=dict(status=',addition+'\nresult=dict(status=')
    text=text.replace("fusion_same_frame_pairing_and_effects='passed',", "fusion_same_frame_pairing_and_effects='passed',local_source_candidates_verified=local_source_count,local_independent_coordinate_checks=sampled_archive_coordinates,local_postverification_successes=local_source_verified,saved_image_copy_and_timing_checks='passed',browser_checks='passed',")
    (directory/'verify.py').write_text(text,encoding='utf-8')
print('Prepared local-source, image, and schema-aware verification for both reports.')
