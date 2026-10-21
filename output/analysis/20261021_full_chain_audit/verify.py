"""Numerical/source/static-report checks only; no browser or device control."""
from pathlib import Path
import ast,hashlib,json
from html.parser import HTMLParser
import pandas as pd
OUT=Path(__file__).resolve().parent
r=json.loads((OUT/'geometry_results.json').read_text(encoding='utf-8'))
t=pd.read_csv(OUT/'geometry_scenarios.csv')
assert len(t)==384*6
assert t.groupby('scenario').frame_id.nunique().eq(384).all()
assert r['scenarios']['full_scene_chain']['backprojection_distance_mm']['mean']<.13
assert r['scenarios']['full_scene_chain']['forward_projection_error_px']['mean']<.8
assert r['scenarios']['full_scene_chain']['laser_plane_normal_error_mm']['maximum']<1e-10
assert r['base_current_vs_scene']['translation_mm']<1e-6
assert r['tool_ik_vs_scene']['rotation_deg']>9
hashes=json.loads((OUT/'input_hashes.json').read_text(encoding='utf-8'))
for p,h in hashes.items(): assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
for p in OUT.glob('*.py'): ast.parse(p.read_text(encoding='utf-8'))
class LinkParser(HTMLParser):
    def __init__(self): super().__init__(); self.links=[]; self.headings=0
    def handle_starttag(self,tag,attrs):
        if tag=='h2': self.headings+=1
        if tag=='a': self.links.extend(v for k,v in attrs if k=='href')
parser=LinkParser(); report=(OUT/'report.html').read_text(encoding='utf-8');parser.feed(report)
assert parser.headings==6
assert 'SCENARIOS' not in report
for link in parser.links:
    if not link.startswith(('https://','http://','#')): assert (OUT/link).is_file(),link
log=(OUT/'runtime_log_evidence.txt').read_text(encoding='utf-8')
meta=json.loads((OUT/'runtime_log_evidence_metadata.json').read_text(encoding='utf-8'))
assert hashlib.sha256(log.encode('utf-8')).hexdigest()==meta['subset_sha256']
assert 'Load scene from file: G:/RobotSimWork-Leningrad-win64/data/test31.xml' in log
assert '[-125.800000, 4.700000, 490.200000]' in log
assert '[9.461327, -13.443195, -4.379898]' in log
result=dict(samples=384,scenarios=6,source_hashes_checked=len(hashes),numerical_assertions='passed',script_syntax='passed',html_structure_and_local_links='passed',scene_load_tool_and_workpiece_log_crosscheck='passed',actual_browser_or_UI='not used; user prohibits Computer Use',device_or_scene_commands='none',production_changes='none',production_build_and_runtime_fix_verification='not performed; investigation only')
(OUT/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
