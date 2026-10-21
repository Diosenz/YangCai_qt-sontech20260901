"""Read-only geometric hypothesis replay. Writes derived results in this folder.
No simulator connection, device commands, C++ build, UI or source mutation.
Requires bundled Python, numpy, pandas. Input scene is a saved current snapshot,
not a scene file archived at the experiment timestamp; this limits attribution.
"""
from pathlib import Path
import hashlib, json, sys, xml.etree.ElementTree as ET
sys.dont_write_bytecode=True
import numpy as np
import pandas as pd
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
INPUTS={}
def read(p):
    p=Path(p); data=p.read_bytes(); INPUTS[str(p)]=hashlib.sha256(data).hexdigest()
    return data.decode('utf-8-sig')
def rx(a):
    c,s=np.cos(a),np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def ry(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rz(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def pose(v):
    t=np.eye(4); a=np.deg2rad(v[3:]); t[:3,:3]=rz(a[2])@ry(a[1])@rx(a[0]); t[:3,3]=v[:3]; return t
def statistics(x,unit):
    a=np.asarray(x); return dict(n=len(a),unit=unit,minimum=float(a.min()),mean=float(a.mean()),median=float(np.median(a)),p95=float(np.quantile(a,.95)),maximum=float(a.max()))
def point(t,p): return t[:3,:3]@p+t[:3,3]
scene_path=Path('G:/RobotSimWork-Leningrad-win64/data/test31.xml')
scene=ET.fromstring(read(scene_path))
def scene_pose(object_id):
    base=next(x for x in scene.iter('base_object') if x.findtext('id')==str(object_id))
    return [float(base.findtext('rel_pose/'+key,'0')) for key in ['x','y','z','rx','ry','rz']]
Bscene=pose(scene_pose(1001)); TCPscene=pose(scene_pose(1011)); Cscene=pose(scene_pose(1013))
sensor={line.split()[0]:float(line.split()[1]) for line in read(ROOT/'config/calibration/SENSORPARA.txt').splitlines() if len(line.split())==2 and not line.startswith(('More','2026'))}
Cactual=pose([sensor[f'pose_CinT[{i}]/m']*1000 for i in range(3)]+[sensor[f'pose_CinT[{i}]/deg'] for i in range(3,6)])
cal=[float(v) for line in read(ROOT/'config/calibration/RBCALIB.DAT').splitlines() if line.startswith('////SRANG ') for v in line.split(' ',1)[1].split(',')]
Bactual=np.linalg.inv(pose(cal))
merge_cal=[float(v) for line in read('D:/qt-sontech-merge/config/calibration/RBCALIB.DAT').splitlines() if line.startswith('////SRANGREVERSE ') for v in line.split(' ',1)[1].split(',')]
Bmerge=pose(merge_cal)
IKtool=pose([-125.136,4.882,494.425,.5225,-34.8683,.1727])
read(ROOT/'DigitalTwin/src/MotomanMh12InverseKinematics.cpp')
read(ROOT/'Vision/src/geometry/WeldProjectionContextBuilder.cpp')
read(ROOT/'Vision/src/geometry/PositionerKinematics.cpp')
read(ROOT/'Vision/src/geometry/WeldImageToSpaceProjector.cpp')
read(ROOT/'DigitalTwin/src/RobotSimDigitalTwinService.cpp')
session=ROOT/'output/online_correction/20261021_114211_739'
from io import StringIO
d=pd.read_csv(StringIO(read(session/'vision_diagnostics.tsv')),sep='\t',low_memory=False).set_index('frame_id')
archive=ROOT/'output/online_local_registration/20261021_034211_693_session_1'
observations={}
for p in sorted(archive.glob('*.json')):
    if p.name.endswith(('.verified.json','.writeback.json')): continue
    obj=json.loads(read(p))
    for f in obj['raw_frames']:
        if f['independent']['valid']: observations[int(f['frame_id'])]=f['independent']
intr=np.array([sensor['sx/m'],sensor['sy/m']]); center=np.array([sensor['cx'],sensor['cy']]); focal=sensor['f/m']; kappa=sensor['Kappa']
plane=np.array([sensor[x] for x in ['a','b','c']])
def undistort(pixel):
    q=(pixel-center)*intr; return q/(1+kappa*(q@q))
def pixel_to_camera(pixel):
    ray=np.r_[undistort(pixel)/focal,1.]; return -1000/(plane@ray)*ray
def camera_to_pixel(p):
    u=focal*p[:2]/p[2]; r2=u@u
    # Inverse of HALCON division undistortion; continuous solution at kappa=0.
    dist=u*(2/(1+np.sqrt(1-4*kappa*r2)))
    return dist/intr+center
scenario_specs={
    'current_production':(Bactual,Cactual),
    'scene_base_only':(Bscene,Cactual),
    'merge_reverse_base_only':(Bmerge,Cactual),
    'scene_sensor_extrinsic_only':(Bactual,Cscene),
    'scene_tool_and_sensor_chain':(Bactual,np.linalg.inv(IKtool)@TCPscene@Cscene),
    'full_scene_chain':(Bscene,np.linalg.inv(IKtool)@TCPscene@Cscene),
}
rows=[]; roundtrip=[]; current_real_replay=[]
for fid,obs in sorted(observations.items()):
    row=d.loc[fid]; simpx=np.array([row.sim_x_px,row.sim_y_px]); rawpx=np.array([row.before_fusion_x_px,row.before_fusion_y_px])
    simcam=pixel_to_camera(simpx); rawcam=pixel_to_camera(rawpx)
    roundtrip.append(np.linalg.norm(camera_to_pixel(simcam)-simpx))
    T=pose([row.robot_x_mm,row.robot_y_mm,row.robot_z_mm,row.robot_rx_deg,row.robot_ry_deg,row.robot_rz_deg])
    E=np.eye(4); E[:3,:3]=ry(np.deg2rad(row.s1_deg))@rz(np.deg2rad(row.s2_deg)); E[:3,3]=E[:3,:3]@np.array([0,0,231.])
    native=np.array(obs['virtual_weld_parent_mm']); real=np.array(obs['real_weld_parent_mm'])
    for name,(B,Ceff) in scenario_specs.items():
        Pcam=np.linalg.inv(E)@B@T@Ceff
        nativecam=point(np.linalg.inv(Pcam),native)
        residual=np.linalg.norm(nativecam-simcam)
        pixel_delta=camera_to_pixel(nativecam)-simpx
        reprojection=np.linalg.norm(pixel_delta)
        plane_error=abs(plane@nativecam+1000)/np.linalg.norm(plane)
        raw_replay=point(Pcam,rawcam)
        if name=='current_production': current_real_replay.append(np.linalg.norm(raw_replay-real))
        rows.append(dict(frame_id=fid,camera_timestamp_ms=int(row.camera_timestamp_ms),scenario=name,backprojection_distance_mm=residual,forward_projection_error_px=reprojection,forward_delta_x_px=float(pixel_delta[0]),forward_delta_y_px=float(pixel_delta[1]),laser_plane_normal_error_mm=plane_error,raw_real_replay_difference_mm=float(np.linalg.norm(raw_replay-real))))
table=pd.DataFrame(rows); table.to_csv(OUT/'geometry_scenarios.csv',index=False,encoding='utf-8-sig')
summary={name:{column:statistics(gr[column].to_numpy(),unit) for column,unit in [('backprojection_distance_mm','mm'),('forward_projection_error_px','px'),('laser_plane_normal_error_mm','mm')]} for name,gr in table.groupby('scenario',sort=False)}
def transform_difference(a,b):
    q=np.linalg.inv(a)@b
    return dict(translation_mm=float(np.linalg.norm(a[:3,3]-b[:3,3])),rotation_deg=float(np.rad2deg(np.arccos(np.clip((np.trace(q[:3,:3])-1)/2,-1,1)))))
result=dict(samples=len(observations),roundtrip_max_error_px=float(max(roundtrip)),production_raw_replay_max_error_mm=float(max(current_real_replay)),scenarios=summary,base_current_vs_scene=transform_difference(Bactual,Bscene),base_current_vs_merge_reverse=transform_difference(Bactual,Bmerge),camera_current_vs_scene=transform_difference(Cactual,Cscene),tool_ik_vs_scene=transform_difference(IKtool,TCPscene),matrices={name:matrix.tolist() for name,matrix in [('base_current',Bactual),('base_scene',Bscene),('base_merge_reverse',Bmerge),('tool_ik',IKtool),('tool_scene',TCPscene),('sensor_actual',Cactual),('sensor_scene',Cscene)]},scene_path=str(scene_path),scene_timestamp_warning='Current saved scene was not archived at the retest timestamp. Numeric closure strongly supports a tool-chain mismatch, but does not alone prove the active in-memory scene state on that date.',interpretation='RobotSim 2D/native3D are nearly consistent with its own effective camera chain; the original 10.55mm comparison mixed actual TCP calibration and a simulator tool built from different calibration. This is not evidence of a broken 2D/3D parser.')
assert result['roundtrip_max_error_px']<1e-8
assert result['production_raw_replay_max_error_mm']<2e-6
assert abs(summary['current_production']['backprojection_distance_mm']['mean']-10.5502791184)<1e-6
assert summary['full_scene_chain']['backprojection_distance_mm']['maximum']<.3
read(Path(__file__))
for path,h in INPUTS.items(): assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==h,path
(OUT/'geometry_results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'input_hashes.json').write_text(json.dumps(INPUTS,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:result[k] for k in ['samples','roundtrip_max_error_px','production_raw_replay_max_error_mm','scenarios','base_current_vs_scene','base_current_vs_merge_reverse','camera_current_vs_scene','tool_ik_vs_scene']},ensure_ascii=False,indent=2))
