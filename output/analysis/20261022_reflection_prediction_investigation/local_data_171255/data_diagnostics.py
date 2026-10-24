"""Read-only log diagnosis. New output is exclusive; no source/log/hardware mutation."""
import csv, json, collections, itertools, math, re, hashlib, bisect
from pathlib import Path
from datetime import datetime, timezone, timedelta
import numpy as np
ROOT=Path.cwd()
RUN=ROOT/'output/online_correction/20261022_171255_974'
LOCAL=ROOT/'output/online_local_registration/20261022_091255_931_session_1'
OUT=Path(__file__).resolve().parent

def read(name):
    with (RUN/name).open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f,delimiter='\t'))
def number(row,key):
    value=row.get(key,'');return float(value) if value else None

def time_text(ms):return datetime.fromtimestamp(int(ms)/1000,timezone(timedelta(hours=8))).isoformat()
def interval(g):
    return dict(count=len(g),first_frame=g[0]['frame_id'],last_frame=g[-1]['frame_id'],seconds=(int(g[-1]['camera_timestamp_ms'])-int(g[0]['camera_timestamp_ms']))/1000,start_ms=int(g[0]['camera_timestamp_ms']),end_ms=int(g[-1]['camera_timestamp_ms']),start=time_text(g[0]['camera_timestamp_ms']),end=time_text(g[-1]['camera_timestamp_ms']),sources=dict(collections.Counter(x['point_source'] for x in g)),pose_revisions=list(dict.fromkeys(x['workpiece_pose_revision'] for x in g)))
def streaks(rows,fn):return [interval(list(g)) for key,g in itertools.groupby(rows,key=fn) if key]
def stats(values):
    a=np.array(values,float)
    return None if not len(a) else dict(min=float(a.min()),median=float(np.median(a)),max=float(a.max()))
def sample(x):
    keys=['frame_id','camera_timestamp_ms','point_source','estimated','sim_x_px','sim_y_px','before_fusion_x_px','before_fusion_y_px','final_x_px','final_y_px','raw_geometry_status','raw_geometry_valid','raw_geometry_x_px','raw_geometry_y_px','geometry_status','geometry_valid','geometry_support_ratio','geometry_fit_rms_px','geometry_ambiguous_column_ratio','coarse_observation_quality','coarse_observation_used','observation_weight_x','observation_weight_y','kcf_weight_x','guided_has_trusted_anchor','guided_prediction_budget_valid','guided_anchor_age_ms','guided_raw_jump_rejected','guided_reacquisition_pending','guided_reacquired','guided_reacquisition_blocked_previous_branch','workpiece_pose_revision','source_image','positioner_tcp_x_mm','positioner_tcp_y_mm','positioner_tcp_z_mm']
    return {k:x[k] for k in keys}

allv=read('vision_diagnostics.tsv');v=[x for x in allv if x['phase']=='online_running'];est=streaks(v,lambda x:x['estimated']=='1');pure=streaks(v,lambda x:x['point_source']=='3');tail=est[-1];tailrows=[x for x in v if int(x['camera_timestamp_ms'])>=tail['start_ms']]
source=collections.Counter(x['point_source'] for x in v)
summary=dict(run=str(RUN.relative_to(ROOT)),local=str(LOCAL.relative_to(ROOT)),total_diagnostics=len(allv),online_frames=len(v),source_counts=dict(source),source_percent={k:100*n/len(v) for k,n in source.items()},all_online_business_valid=all(x['result_valid']=='1' for x in v),algorithm_attempted=sum(x['algorithm_attempted']=='1' for x in v),estimated_streaks=est,longest_pure_prediction_streaks=sorted(pure,key=lambda x:x['seconds'],reverse=True)[:10],last_estimated_streak=tail)
revisions=[]
for key,g in itertools.groupby(v,key=lambda x:int(x['workpiece_pose_revision'])):
    a=list(g);p=[x for x in a if x['point_source']=='3']
    entry=interval(a);entry.update(revision=key,raw_geometry_status=dict(collections.Counter(x['raw_geometry_status'] for x in a)),trusted_anchor=sum(x['guided_has_trusted_anchor']=='1' for x in a),no_anchor=sum(x['guided_has_trusted_anchor']=='0' for x in a),prediction_budget_valid=sum(x['guided_prediction_budget_valid']=='1' for x in a),observation_gain=stats([number(x,'observation_weight_x') for x in a if x['observation_weight_x']]),tracker_gain=stats([number(x,'kcf_weight_x') for x in a if x['kcf_weight_x']]),coarse_final_distance_px=stats([math.hypot(number(x,'final_x_px')-number(x,'before_fusion_x_px'),number(x,'final_y_px')-number(x,'before_fusion_y_px')) for x in a]))
    if p:
        delta=np.array([[number(x,'final_x_px')-number(x,'sim_x_px'),number(x,'final_y_px')-number(x,'sim_y_px')] for x in p]);entry['pure_prediction_final_minus_sim_range_px']=(delta.max(0)-delta.min(0)).tolist();entry['pure_prediction_final_minus_sim_first_px']=delta[0].tolist()
    revisions.append(entry)
summary['revisions']=revisions
transactions=[]
for f in sorted(LOCAL.glob('*.writeback.json'),key=lambda x:int(re.search(r'local_icp_1_(\d+)_',x.name)[1])):
    write=json.loads(f.read_text(encoding='utf-8-sig'));base=Path(str(f)[:-len('.writeback.json')]);a=json.loads(base.read_text(encoding='utf-8-sig'));outcome=next((k for k in ['retained','verified','rollback'] if Path(str(base)+'.'+k+'.json').exists()),None);end=json.loads(Path(str(base)+'.'+outcome+'.json').read_text(encoding='utf-8-sig')) if outcome else {};s=end.get('post_write_samples',[]);m=write['pending_parent_from_workpiece'];rev=int(write['workpiece_pose_revision']);vr=next((x for x in revisions if x['revision']==rev),None)
    transactions.append(dict(attempt=base.name,source_frame=a['frame_id'],pose_revision=rev,outcome=outcome,outcome_succeeded=end.get('succeeded'),writeback_succeeded=write.get('succeeded'),first_vision_frame=vr['first_frame'] if vr else None,first_vision_capture_ms=vr['start_ms'] if vr else None,last_post_capture_ms=int(s[-1]['capture_time_ms']) if s else None,post_frames=len(s),post_surface_valid=sum(x.get('valid',False) for x in s),post_pair_valid=sum(x.get('comparison_valid',False) for x in s),post_accepted=sum(x.get('accepted_observation',False) for x in s),target_translation_mm=[m[i] for i in (3,7,11)],cumulative_translation_mm=write['cumulative_translation_mm'],retained_without_verification=end.get('retained_without_geometric_verification'),before_surface_residual_mm=a['before_residual_mm'],after_surface_residual_mm=a['after_residual_mm']))
summary['local_transactions']=transactions;summary['local_outcome_counts']=dict(collections.Counter(x['outcome'] for x in transactions))
rt=read('realtime_correction.txt');late=[x for x in rt if int(x['timestamp_ms'])>=tail['start_ms']];receipts=[x for x in late if x['event']=='command_receipt'];commands=[x for x in late if x['event']=='command_generated'];vision_samples=[x for x in late if x['event']=='vision_sample'];actual=[x for x in read('actual_robot_path.txt') if x['phase']=='online_running' and int(x['timestamp_ms'])>=tail['start_ms']];follow=[x for x in read('online_follow_targets.txt') if int(x['timestamp_ms'])>=tail['start_ms']]
summary['last_interval_control']=dict(events=dict(collections.Counter(x['event'] for x in late)),estimated_vision_samples=sum(x['estimated']=='1' for x in vision_samples),sent_receipts=sum(x['sent']=='1' for x in receipts),completed_receipts=sum(x['completed']=='1' for x in receipts),receipt_error_counts=dict(collections.Counter(x['error_code'] for x in receipts)),command_dx_um=stats([number(x,'dx_um') for x in commands]),command_dy_um=stats([number(x,'dy_um') for x in commands]),command_dz_um=stats([number(x,'dz_um') for x in commands]),speed_override=dict(collections.Counter(x['speed_override'] for x in commands)),first_command=commands[0],last_command=commands[-1],last_completed_receipt=next((x for x in reversed(receipts) if x['completed']=='1'),None),last_receipt=receipts[-1],actual_robot_feedback_count=len(actual),first_actual_feedback=actual[0],last_actual_feedback=actual[-1],follow_target_count=len(follow),first_follow_target=follow[0],last_follow_target=follow[-1])
summary['selected_frames']=[sample(x) for x in v if x['frame_id'] in ['13609','14676','14733','14858','14945','15248','15301','15306','15682','16000','16021','16194']]
requested=next(x for x in v if x['frame_id']=='13609');pngs=[]
for x in v:
    if x['source_image']:pngs.append(dict(frame=x['frame_id'],difference=int(x['frame_id'])-13609,path=x['source_image']))
for f in LOCAL.glob('*.json'):
    if '.json.' in f.name:continue
    j=json.loads(f.read_text(encoding='utf-8-sig'));src=j.get('source_image',{});frame=src.get('frame_id')
    if frame and src.get('saved'):pngs.append(dict(frame=frame,difference=int(frame)-13609,path=str((LOCAL/src['file']).relative_to(ROOT))))
summary['frame13609']=dict(diagnostic=sample(requested),exact_source_png_saved=bool(requested['source_image']),closest_saved_pngs=sorted(pngs,key=lambda x:abs(x['difference']))[:5])
summary['input_sha256']={str((RUN/n).relative_to(ROOT)):hashlib.sha256((RUN/n).read_bytes()).hexdigest() for n in ['vision_diagnostics.tsv','realtime_correction.txt','online_follow_targets.txt','actual_robot_path.txt','session_summary.txt']}
with (OUT/'diagnostic_summary.json').open('x',encoding='utf-8') as f:json.dump(summary,f,ensure_ascii=False,indent=2)
with (OUT/'selected_frames.tsv').open('x',encoding='utf-8',newline='') as f:
    rows=summary['selected_frames'];w=csv.DictWriter(f,fieldnames=list(rows[0]),delimiter='\t');w.writeheader();w.writerows(rows)
print(json.dumps(dict(online_frames=len(v),source_counts=dict(source),last_estimated_interval=tail,outcome_counts=summary['local_outcome_counts'],output=str(OUT.relative_to(ROOT))),ensure_ascii=False))
