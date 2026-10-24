"""Summarize the two supplied sessions and archive schemas without modifying inputs."""
from pathlib import Path
import json
import pandas as pd
ROOT=Path(__file__).resolve().parents[3]
for prefix,archive_name in [('20261021_185548_548','20261021_105548_492_session_1'),('20261021_201400_950','20261021_121400_904_session_1')]:
    session=next((ROOT/'output/online_correction').glob(prefix+'*'))
    d=pd.read_csv(session/'vision_diagnostics.tsv',sep='\t',low_memory=False);v=d[d.phase.eq('online_running')&d.result_valid.eq(1)]
    folder=ROOT/'output/online_local_registration'/archive_name
    objects=[(p,json.loads(p.read_text(encoding='utf-8-sig'))) for p in folder.glob('*.json')]
    candidate=[(p,o) for p,o in objects if not p.name.endswith(('.verified.json','.writeback.json'))]
    verified=[(p,o) for p,o in objects if p.name.endswith('.verified.json')]
    result=dict(session=session.name,valid_vision_count=len(v),invalid_online_count=int((d.phase.eq('online_running')&d.result_valid.ne(1)).sum()),fusion_applied=int(v.fusion_applied.sum()),jump_flags=dict(correlation=int(v.correlation_jump_rejected.sum()),fusion=int(v.fusion_jump_rejected.sum())),archives=len(objects),candidates=len(candidate),verified_records=len(verified))
    if candidate:
        obj=candidate[0][1]
        result['candidate_scalar_fields']={k:val for k,val in obj.items() if not isinstance(val,(dict,list))}
        result['candidate_nested_types']={k:(list(val)[:35] if isinstance(val,dict) else 'list '+str(len(val))) for k,val in obj.items() if isinstance(val,(dict,list))}
        result['first_stability']=obj.get('stability')
        result['reason_counts']=pd.Series([o.get('reason') for p,o in candidate]).value_counts().head(7).to_dict()
    if verified:
        obj=verified[0][1]
        result['verified_scalar_fields']={k:val for k,val in obj.items() if not isinstance(val,(dict,list))}
        result['verified_nested_types']={k:(list(val)[:20] if isinstance(val,dict) else 'list '+str(len(val))) for k,val in obj.items() if isinstance(val,(dict,list))}
    manifest=pd.read_csv(session/'vision_images/image_manifest.tsv',sep='\t')
    result['image_manifest_columns']=manifest.columns.tolist();result['image_manifest_first']=manifest.head(1).to_dict('records')
    print(json.dumps(result,ensure_ascii=False,indent=2,default=str))
