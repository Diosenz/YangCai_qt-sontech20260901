"""Add the fourth most recent distinct run using the same geometric definition.
The two 16:28 directories contain identical primary records and count as one run.
"""
from pathlib import Path
import hashlib, importlib.util, json, sys
import pandas as pd
sys.dont_write_bytecode=True
OUT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('analysis',OUT/'analyze.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
result=a.analyze(a.ROOT/'output/online_correction/20261020_162854_226_P1+4固定结果')
results=[result]+json.loads((OUT/'comparison.json').read_text())
rows=[]
for r in results:
 details=pd.read_csv(OUT/r['session'][:19]/'TCP_to_raw_relocated_teach_distances.csv')
 distances=details.distance_mm
 metrics={'mean_mm':float(distances.mean()),'max_mm':float(distances.max()),'rmse_mm':float((distances.pow(2).mean())**.5)}
 assert all(abs(metrics[k]-r['metrics']['TCP_to_raw_relocated_teach'][k])<1e-9 for k in metrics)
 rows.append({'session':r['session'],'tcp_count':len(distances),**metrics})
pd.DataFrame(rows).to_csv(OUT/'latest_four_errors.csv',index=False,encoding='utf-8-sig')
(OUT/'latest_four_metadata.json').write_text(json.dumps({'definition':'TCP point to all segments of the original taught polyline after rigid relocation derived from before-fusion KCF; sample-weighted, mm','new_input_hashes':a.INPUTS,'duplicate_run':'20261020_162854_226 and 20261020_162854_226_P1+4固定结果 share identical summary, robot, and vision diagnostic hashes','rows':rows},ensure_ascii=False,indent=2))
assert all(hashlib.sha256((a.ROOT/k).read_bytes()).hexdigest()==v for k,v in a.INPUTS.items())
print('LATEST FOUR:',json.dumps(rows,ensure_ascii=False,indent=2))
