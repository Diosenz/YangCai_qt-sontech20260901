from pathlib import Path
import hashlib,json
import numpy as np
import pandas as pd
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]
r=json.loads((OUT/'results.json').read_text(encoding='utf-8'))
h=json.loads((OUT/'input_hashes.json').read_text(encoding='utf-8'))
for p,digest in h.items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==digest,p
assert len({s['geometry_sha256'] for s in r['sources']})==4
assert abs(r['new_reference_radius_mm']-sum(s['circle']['radius_mm'] for s in r['sources'])/4)<1e-12
c=np.array(r['current_circle']['center']);n=np.array(r['current_circle']['normal']);n/=np.linalg.norm(n);rad=r['new_reference_radius_mm']
for label,m in r['metrics'].items():
    df=pd.read_csv(OUT/(label+'_to_synthesized_circle.csv'));q=df[['x_mm','y_mm','z_mm']].to_numpy();e=df.distance_mm.to_numpy()
    delta=q-c
    # Cross-product radius and scalar height: independent of the projector code.
    radial=np.linalg.norm(np.cross(delta,n),axis=1);height=delta@n
    independent=np.sqrt((radial-rad)**2+height**2)
    assert np.max(abs(independent-e))<1e-10
    assert len(e)==m['n'] and abs(np.mean(e)-m['mean_mm'])<1e-12 and abs(np.sqrt(np.mean(e**2))-m['rmse_mm'])<1e-12
    assert abs(e.max()-m['max_mm'])<1e-12 and int(np.sum(e>3))==m['above_3mm_count']
    original=ROOT/'output/analysis/20261021_152328_050_retest'/({'TCP':'tcp','raw_KCF':'raw_kcf','final_vision':'final_vision'}[label]+'_material_frame.csv')
    assert np.max(abs(q-pd.read_csv(original).iloc[:,1:4].to_numpy()))<1e-10
base='20261021CHUFENGKOU20_SYNTH_ANALYSIS'
for suffix in ['W','T']:
    a=np.loadtxt(OUT/(base+'_'+suffix+'.txt'));orig=np.loadtxt(ROOT/('data/WorkPiece/20260404CHUFENGKOU1_'+suffix+'.txt'))
    assert a.shape==orig.shape and np.max(abs(a[:,3:]-orig[:,3:]))==0
    center=np.array(r['new_original_circle']['center']);normal=np.array(r['new_original_circle']['normal']);d=a[:,:3]-center
    assert np.max(abs(d@normal))<1e-8 and np.max(abs(np.linalg.norm(d,axis=1)-rad))<1e-8
for s in r['sources']:
    for key in ['W_file','T_file']:
        source=Path(s[key]);copy=OUT/'sources'/Path(s['W_file']).stem/source.name
        assert source.read_bytes()==copy.read_bytes()
v=json.loads((OUT/'viewer_data.json').read_text(encoding='utf-8'))
assert [len(a['x']) for a in v['traces']]==[5848,7522,1441,1441,1368,1368]
assert [a['visible'] for a in v['traces']]==[True,True,True,True,False,False]
verification=json.loads((OUT/'verification.json').read_text(encoding='utf-8'))
verification.update(independent_cross_product_circle_formula='passed',metric_csv_readback='passed',same_TCP_KCF_measurements='passed',generated_W_T_readback='passed',viewer_defaults_and_trace_counts='passed')
(OUT/'verification.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(verification,ensure_ascii=False,indent=2))
