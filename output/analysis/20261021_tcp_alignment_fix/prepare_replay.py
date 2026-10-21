"""Export saved TCP inputs and verify the project profile against read-only RobotSim XML."""
from pathlib import Path
import hashlib,json,re,xml.etree.ElementTree as ET
import pandas as pd
OUT=Path(__file__).resolve().parent; ROOT=OUT.parents[2]
scene=Path('G:/RobotSimWork-Leningrad-win64/data/test31.xml')
scene_bytes=scene.read_bytes(); tree=ET.fromstring(scene_bytes)
base=next(x for x in tree.iter('base_object') if x.findtext('id')=='1011')
scene_pose=[float(base.findtext('rel_pose/'+k)) for k in ['x','y','z','rx','ry','rz']]
source=ROOT/'DigitalTwin/src/MotomanMh12InverseKinematics.cpp'
text=source.read_text(encoding='utf-8-sig')
values=[]
for key in ['X_M','Y_M','Z_M','RX_RAD','RY_RAD','RZ_RAD']:
    values.append(float(re.search(r'TOOL_TO_FLANGE_'+key+r'\s*=\s*([-\d.]+)',text)[1]))
profile=[v*1000 if i<3 else v for i,v in enumerate(values)]
assert max(abs(a-b) for a,b in zip(profile,scene_pose))<1e-10
session=ROOT/'output/online_correction/20261021_114211_739'
data_path=session/'vision_diagnostics.tsv'; data=pd.read_csv(data_path,sep='\t',low_memory=False)
data[['robot_'+k for k in ['x_mm','y_mm','z_mm','rx_deg','ry_deg','rz_deg']]].to_csv(OUT/'recorded_tcp_inputs.txt',sep=' ',header=False,index=False,float_format='%.10f')
result=dict(scene_file=str(scene),scene_sha256=hashlib.sha256(scene_bytes).hexdigest(),scene_tcp_pose_mm_deg=scene_pose,project_tool_pose_mm_deg=profile,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),saved_input_file=str(data_path),saved_input_sha256=hashlib.sha256(data_path.read_bytes()).hexdigest(),replay_samples=len(data),robot_commands_sent=False,ui_or_computer_use=False)
(OUT/'replay_metadata.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
