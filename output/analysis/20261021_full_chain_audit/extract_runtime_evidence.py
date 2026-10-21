"""Extract a small line-numbered evidence snapshot from the long-running log.
Source is only read; no simulator requests or mutations are made.
"""
from pathlib import Path
import hashlib,json
OUT=Path(__file__).resolve().parent
source=Path('G:/RobotSimWork-Leningrad-win64/log/20261019/20261019-102140.log')
targets=[2367376,2367540,2367543,2367545,2367551,2367582,2367587,2367588,2380853]
selected=set(i+j for i in targets for j in range(-1,2))
before=source.stat(); rows=[]
with source.open('r',encoding='utf-8-sig',errors='replace') as stream:
    for i,line in enumerate(stream,1):
        if i in selected: rows.append(f'{i}: {line.rstrip()}')
        if i>max(selected): break
after=source.stat()
text='Source: '+str(source)+'\nLine numbers are one-based. Read-only extracted subset.\n\n'+'\n'.join(rows)+'\n'
(OUT/'runtime_log_evidence.txt').write_text(text,encoding='utf-8')
metadata=dict(source=str(source),source_size_before=before.st_size,source_size_after=after.st_size,source_mtime_ns_before=before.st_mtime_ns,source_mtime_ns_after=after.st_mtime_ns,selected_line_numbers=sorted(selected),subset_sha256=hashlib.sha256(text.encode('utf-8')).hexdigest(),note='Hash covers extracted evidence, not the entire potentially growing log.')
(OUT/'runtime_log_evidence_metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
print(text)
