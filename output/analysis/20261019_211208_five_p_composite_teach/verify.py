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
assert [len(t['x']) for t in v['traces']]==[r['metrics']['TCP']['n'],r['metrics']['recorded_vision']['n'],1441,1441,len(a),len(b)]
for i,key in enumerate(['TCP','recorded_vision']):
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
session=OUT.parents[1]/'online_correction'/r['session']
runtime=pd.read_csv(session/'realtime_correction.txt',sep='\t');robot=pd.read_csv(session/'actual_robot_path.txt',sep='\t');initial=pd.read_csv(session/'initial_correction.txt',sep='\t')
vis=runtime[runtime.event.eq('vision_sample')&runtime.phase.eq('online_running')&runtime.vision_valid.eq(1)].reset_index(drop=True)
saved=pd.read_csv(OUT/'recorded_vision_circle_error_components.csv')
assert np.max(abs(saved[['x_mm','y_mm','z_mm']].to_numpy()-vis[['vision_x_mm','vision_y_mm','vision_z_mm']].to_numpy()-[0,0,231]))<1e-10
assert np.array_equal(saved.timestamp_ms,vis.timestamp_ms)
assert initial.event.eq('teach_point').sum()==5
assert robot.phase.eq('waiting_robot_program_stop').sum()==r['source_counts']['waiting_stop_tcp']==1097
assert r['summary']['session_state']=='error' and '120秒' in r['summary']['message']
assert not (session/'vision_diagnostics.tsv').exists()
result=dict(status='passed',input_hashes_verified=len(hashes),report_source_hashes_verified=len(report_hashes),independent_scalar_geometry_checks=checks,metrics_and_component_identities='passed',source_logged_visual_coordinates='passed',five_initial_points_and_stop_timeout='passed',event_counts='passed',viewer_coordinates='passed',rigid_transform='passed',local_links_checked=links,reference_radius_fixed_mm=radius,mouse_used=False,production_code_modified=False)
(OUT/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
