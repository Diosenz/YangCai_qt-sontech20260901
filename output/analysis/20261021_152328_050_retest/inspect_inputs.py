from pathlib import Path
import importlib.util, sys, json
sys.dont_write_bytecode=True
import numpy as np
import pandas as pd
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
spec=importlib.util.spec_from_file_location('geometry',ROOT/'output/analysis/20261020_evening_comparison/assets/geometry.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
session=ROOT/'output/online_correction/20261021_152328_050'
r=pd.read_csv(session/'actual_robot_path.txt',sep='\t');d=pd.read_csv(session/'vision_diagnostics.tsv',sep='\t')
work=np.loadtxt(ROOT/'data/WorkPiece/20260404CHUFENGKOU1_W.txt')[:,:3]
archive=ROOT/'output/online_local_registration/20261021_072328_003_session_1'
rows=[];rawframes={}; virtual=[];surface=[];writebacks=[]
for p in sorted(archive.glob('*.json')):
    a=json.loads(p.read_text(encoding='utf-8-sig'))
    if p.name.endswith('.writeback.json'):
        writebacks.append(dict(file=p.name,before=a['before_parent_from_workpiece'],pending=a['pending_parent_from_workpiece'],succeeded=a['succeeded']))
        continue
    if p.name.endswith('.verified.json'):continue
    rows.append(dict(file=p.name,reason=a['reason'],accepted=a['accepted_before_archive'],before=a['before_residual_mm'],after=a['after_residual_mm']))
    for f in a['raw_frames']:rawframes[int(f['frame_id'])]=f
for fid,f in rawframes.items():
    ind=f['independent']
    if ind['valid']:
        virtual.append(ind['virtual_weld_parent_mm'])
first=json.loads(next(p for p in sorted(archive.glob('*.json')) if not p.name.endswith(('.verified.json','.writeback.json'))).read_text(encoding='utf-8-sig'))
print('robot phases',r.groupby('phase').size().to_dict(),'vision phases',d.groupby('phase').size().to_dict())
print('original circle',g.circle(work))
print('archive keys',list(first),'frame keys',list(next(iter(rawframes.values()))))
print('frame sample metadata',json.dumps({k:v for k,v in next(iter(rawframes.values())).items() if k not in ['real_parent_mm','virtual_parent_mm','real_surface_parent_mm','virtual_surface_parent_mm']},ensure_ascii=False)[:2500])
print('archives',len(rows),'writebacks',len(writebacks),'frames',len(rawframes))
print('virtual circle in moving model coordinates',g.circle(np.array(virtual)))
pd.DataFrame(rows).to_csv(OUT/'archive_inventory.csv',index=False,encoding='utf-8-sig')
(OUT/'writeback_inventory.json').write_text(json.dumps(writebacks,indent=2),encoding='utf-8')
print('first/last model before',writebacks[0]['before'],writebacks[-1]['before'])
