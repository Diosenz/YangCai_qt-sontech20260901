"""Independent artifact checks without running devices, building C++, or using UI."""
from pathlib import Path
import hashlib, json
import numpy as np
import pandas as pd
OUT=Path(__file__).resolve().parent
r=json.loads((OUT/'results.json').read_text(encoding='utf-8'))
hashes=json.loads((OUT/'input_hashes.json').read_text(encoding='utf-8'))
for p,h in hashes.items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
work=pd.read_csv(OUT/'original_teach.csv').to_numpy();ref=pd.read_csv(OUT/'estimated_moved_teach.csv').to_numpy()
M=np.loadtxt(OUT/'teach_to_estimated_current_transform.csv',delimiter=',')
assert np.max(abs(ref-work@M[:3,:3].T-M[:3,3]))<1e-8
assert np.allclose(M[:3,:3].T@M[:3,:3],np.eye(3),atol=2e-12)
checks=0
for label,stats in r['metrics'].items():
    d=pd.read_csv(OUT/(label+'.csv'));e=d.distance_mm.to_numpy()
    assert len(e)==stats['n']
    for k,v in [('mean_mm',e.mean()),('max_mm',e.max()),('rmse_mm',np.sqrt(np.mean(e*e)))]:assert abs(v-stats[k])<1e-11,(label,k)
    a=d[[f'source_{x}_mm' for x in 'xyz']].to_numpy();b=d[[f'foot_{x}_mm' for x in 'xyz']].to_numpy()
    assert np.max(abs(np.linalg.norm(a-b,axis=1)-e))<1e-10
    assert stats['max_mm']>=stats['rmse_mm']>=stats['mean_mm']>=0
    if label.endswith('estimated_moved_teach'):
        for i in np.unique(np.r_[np.linspace(0,len(e)-1,19,dtype=int),e.argmax()]):
            p=a[i];best=float('inf')
            for aa,bb in zip(ref[:-1],ref[1:]):
                v=bb-aa;den=float(v@v)
                f=0 if den==0 else max(0.,min(1.,float((p-aa)@v)/den))
                best=min(best,float(np.linalg.norm(p-aa-f*v)))
            assert abs(best-e[i])<1e-10,(label,i,best,e[i]);checks+=1
cv=pd.read_csv(OUT/'raw_kcf_cross_validation.csv')
assert len(cv)==r['raw_kcf_count'] and abs(np.sqrt(np.mean(cv.distance_mm**2))-r['cross_validation_pooled']['rmse_mm'])<1e-12
v=json.loads((OUT/'viewer_data.json').read_text(encoding='utf-8'))
assert [len(t['x']) for t in v['traces']]==[5848,7522,1441,1441,1368,1368]
for t,q in [(v['traces'][0],pd.read_csv(OUT/'tcp_material_frame.csv').iloc[:,1:].to_numpy()),(v['traces'][1],pd.read_csv(OUT/'raw_kcf_material_frame.csv').iloc[:,1:].to_numpy()),(v['traces'][4],work),(v['traces'][5],ref)]:
    assert np.max(abs(np.c_[t['x'],t['y'],t['z']]-q))<1e-10
report=(OUT/'report.html').read_text(encoding='utf-8');viewer=(OUT/'index.html').read_text(encoding='utf-8')
assert 'COMMON_POSE' not in viewer and '20261019_110509_938' not in viewer and 'RADIUS_VALUE' not in viewer
assert '<option value="actual" selected>' in viewer and '不能直接当独立真值' in viewer
assert '真实浏览器' in report and '估计参考' in report
result=dict(input_hashes_verified=len(hashes),metric_tables='passed',independent_scalar_point_segment_checks=checks,rigid_transform_and_viewer_coordinate_checks='passed',cross_validation='passed',numeric_projection_replay=r['verification'],static_preview='inspected',browser_ui='not performed; no mouse or Computer Use',production_files='unchanged; only new analysis artifacts')
(OUT/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
