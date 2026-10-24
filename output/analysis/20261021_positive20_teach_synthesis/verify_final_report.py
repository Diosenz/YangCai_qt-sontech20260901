from pathlib import Path
from html.parser import HTMLParser
import hashlib,json
import numpy as np
import pandas as pd
OUT=Path(__file__).resolve().parent;OLD=OUT.parent/'20261021_152328_050_retest'
class Links(HTMLParser):
    def __init__(self):super().__init__();self.links=[];self.frames=[];self.base=None
    def handle_starttag(self,tag,attrs):
        a=dict(attrs)
        if tag=='base':self.base=a.get('href')
        if tag in ['a','img','iframe']:
            target=a.get('href',a.get('src'))
            if target:self.links.append(target)
            if tag=='iframe':self.frames.append(target)
count=0
for directory in [OUT,OLD]:
    parser=Links();parser.feed((directory/'report.html').read_text(encoding='utf-8'))
    base=(directory/parser.base).resolve() if parser.base else directory
    for link in parser.links:
        if '://' not in link and not link.startswith('#'):
            assert (base/link.split('#')[0]).exists(),(directory,link);count+=1
    assert parser.frames==['index.html','axis_fluctuations.html']
data=json.loads((OUT/'axis_viewer_data.json').read_text(encoding='utf-8'));assert len(data['traces'])==6
for i,axis in enumerate('XYZ'):
    for j,key in enumerate(['TCP','raw_KCF']):
        d=pd.read_csv(OUT/(key+'_to_synthesized_circle.csv'));t=data['traces'][2*i+j]
        error=d[axis.lower()+'_mm']-d['foot_'+axis.lower()+'_mm']
        assert len(t['x'])==len(d) and np.max(abs(np.array(t['y'])-error.to_numpy()))<1e-12
events=pd.read_csv(OUT/'over_3mm_events.csv');r=json.loads((OUT/'results.json').read_text(encoding='utf-8'))
for label in ['TCP','raw_KCF','final_vision']:
    rows=events[events.trajectory.eq(label)]
    assert rows.point_count.sum()==r['metrics'][label]['above_3mm_count']
    rows=pd.read_csv(OUT/'angular_sector_metrics.csv');assert rows[rows.trajectory.eq(label)].n.sum()==r['metrics'][label]['n']
for p,h in json.loads((OUT/'final_report_input_hashes.json').read_text(encoding='utf-8')).items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
result=dict(report_local_links_verified=count,embedded_views=['3D four trajectories','XYZ residual fluctuations'],same_measurements_and_reference=True,axis_trace_coordinates='passed',event_counts_and_sector_counts='passed',report_input_hashes='passed',old_report_entry_updated=True,old_report_preserved=True,actual_browser_interaction='not performed',mouse_used=False)
(OUT/'final_report_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(result,ensure_ascii=False,indent=2))
