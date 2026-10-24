from pathlib import Path
import hashlib,importlib.util,json,re,subprocess,sys
sys.dont_write_bytecode=True
import numpy as np
import pandas as pd
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]
s=importlib.util.spec_from_file_location('g',ROOT/'output/analysis/20261020_evening_comparison/assets/geometry.py');g=importlib.util.module_from_spec(s);s.loader.exec_module(g)
roots=[p for p in Path('E:/').iterdir() if p.is_dir() and ('sontechworks' in p.name.lower() or p.name=='SW-20230214backup')]
paths=[];mappings=[];sources={}
for root in roots:
    proc=subprocess.run(['rg','--files','--hidden','--no-ignore','-g','*_W.txt','-g','*_T.txt','-g','!**/.git/**','-g','!**/.vs/**',str(root)],capture_output=True,encoding='utf-8',errors='replace')
    found=[Path(line) for line in proc.stdout.splitlines() if re.search('chufeng|cfk',Path(line).name,re.I)]
    paths.extend(found)
    cpp=subprocess.run(['rg','--files','--hidden','--no-ignore','-g','robotleadingDlg.cpp','-g','!**/.git/**',str(root)],capture_output=True,encoding='utf-8',errors='replace')
    for p in map(Path,cpp.stdout.splitlines()):
        raw=p.read_bytes()
        try:code=raw.decode('utf-8-sig')
        except UnicodeDecodeError:code=raw.decode('gb18030',errors='replace')
        sources[str(p)]=hashlib.sha256(raw).hexdigest()
        lines=code.splitlines()
        for i,line in enumerate(lines):
            if re.search(r'workpieceTeachTraj\s*=.*CHUFENG',line,re.I) and not line.lstrip().startswith('//'):
                mappings.append(dict(root=str(root),file=str(p),line=i+1,context='\n'.join(lines[max(0,i-3):i+3])))
paths.extend([ROOT/'data/WorkPiece/20260404CHUFENGKOU1_W.txt',ROOT/'data/WorkPiece/20260404CHUFENGKOU1_T.txt'])
rows=[];errors=[]
for p in paths:
    data=p.read_bytes();h=hashlib.sha256(data).hexdigest()
    row=dict(path=str(p),name=p.name,kind=p.stem[-1],sha256=h,bytes=len(data),flip=bool(re.search('flip|fan|反',p.stem,re.I)))
    try:
        a=np.loadtxt(p,encoding='utf-8-sig',ndmin=2);q=a[:,:3]
        row.update(n=len(a),cols=a.shape[1],geometry_sha256=hashlib.sha256(np.round(q,6).astype('<f8').tobytes()).hexdigest())
        if len(q)>=8:
            c=g.circle(g.representatives(q,g.circle(q)['center']));e=g.circle_distance(q,c)
            angle=np.sort(np.mod(np.arctan2(q[:,1]-c['center'][1],q[:,0]-c['center'][0]),2*np.pi))
            row.update(radius_mm=c['radius_mm'],cx=c['center'][0],cy=c['center'][1],cz=c['center'][2],nx=c['normal'][0],ny=c['normal'][1],nz=c['normal'][2],fit_rmse_mm=float(np.sqrt(np.mean(e**2))),coverage_deg=float(360-np.rad2deg(np.diff(np.r_[angle,angle[0]+2*np.pi]).max())))
    except Exception as exc:row['parse_error']=str(exc);errors.append(str(p))
    rows.append(row)
df=pd.DataFrame(rows);df.to_csv(OUT/'all_chufeng_W_T_inventory.csv',index=False,encoding='utf-8-sig')
(OUT/'historical_mappings.json').write_text(json.dumps(mappings,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'code_evidence_hashes.json').write_text(json.dumps(sources,ensure_ascii=False,indent=2),encoding='utf-8')
summary=dict(roots=[str(p) for p in roots],files=len(rows),unique_file_hashes=df.sha256.nunique(),unique_W_geometry=df[df.kind.eq('W')].geometry_sha256.nunique(),parse_failures=errors,mappings=mappings)
(OUT/'inventory_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False,indent=2))
print(df[df.kind.eq('W')&~df.flip].drop_duplicates('geometry_sha256')[['name','n','radius_mm','cz','fit_rmse_mm','coverage_deg','path']].to_string(index=False))
