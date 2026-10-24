import json
from pathlib import Path
p=Path('C:/Users/admin/.codex/sessions/2026/10/21/rollout-2026-10-21T11-59-07-01a15c9c-e169-7d62-8e8a-e762b13bc61c.jsonl')
found={}
def walk(v):
    if isinstance(v,dict):
        if 'title' in v and 'text' in v and ('数据分析' in str(v['title']) or 'AI 协作工作流' in str(v['title'])):
            found[str(v['title'])]=v['text']
        for a in v.values():walk(a)
    elif isinstance(v,list):
        for a in v:walk(a)
    elif isinstance(v,str):
        try:walk(json.loads(v))
        except (ValueError,TypeError,RecursionError):pass
for l in p.open(encoding='utf-8'):
    d=json.loads(l)
    if d.get('type')=='event_msg' and d.get('payload',{}).get('type')=='item_completed':walk(d)
for k,v in found.items():print(k,'\n',v.replace('\\n','\n').replace('\\"','"'),'\n')
Path(__file__).with_name('prior_notion_workflow_snapshot.json').write_text(json.dumps(found,ensure_ascii=False,indent=2),encoding='utf-8')
