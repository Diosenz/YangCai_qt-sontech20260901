"""组合离线回放；显式同材料位置比较后续TCP，从不发送运动。"""
import argparse
import hashlib
import html
import json
import math
import pathlib
import subprocess
import sys

from control_replay import ControlReplaySession
from evidence_session import finite_number, finite_point
from validation_replay import ValidationReplaySession

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BASELINE = '430aaa204fd5319dd7f697d3de2b4e1faddbb06f'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def jsonl(path):
    return [json.loads(v) for v in path.read_text(encoding='utf-8').splitlines() if v.strip()]


def material_key(value):
    """不从XYZ或TCP反推材料位置，不使用最近点、逐行或同时刻替代关联。"""
    keys = ('coordinate_frame', 'workpiece_frame', 'path_id')
    if (not isinstance(value, dict) or value.get('coordinate_frame') != 'PositionerTcp'
            or any(not isinstance(value.get(k), str) or not value[k].strip() for k in keys)
            or not finite_number(value.get('s_mm'))
            or not isinstance(value.get('path_revision'), int) or isinstance(value.get('path_revision'),bool)
            or value['path_revision'] < 0):
        return None
    return tuple(value[k] for k in keys)+(value['path_revision'],value['s_mm'])


def compare_tcp(frame, released, records):
    result = dict(tcp_record_id=None, association='unavailable', reference_to_tcp_distance_mm=None,
                  is_seam_accuracy=False, reason='no released measured reference')
    if not finite_point(released, 3):
        return result
    key = material_key(frame.get('material_position'))
    timestamp = frame.get('timestamp_ms')
    if key is None or not finite_number(timestamp):
        result['reason'] = 'unknown material position or exposure time; no guessed pairing'
        return result
    candidates = [r for r in records if material_key(r.get('material_position')) == key
                  and finite_number(r.get('timestamp_ms')) and r['timestamp_ms'] > timestamp
                  and finite_point(r.get('positioner_tcp_mm'), 3)
                  and r.get('kind') in ('synthetic', 'real') and r.get('kind')==frame.get('kind')
                  and isinstance(r.get('record_id'), str) and r['record_id']]
    if len(candidates) != 1:
        result['reason'] = 'no unique later TCP at exact material position; no nearest, row or simultaneous pairing'
        return result
    record = candidates[0]
    result.update(tcp_record_id=record['record_id'], tcp_kind=record['kind'],
        tcp_timestamp_ms=record['timestamp_ms'], tcp_positioner_tcp_mm=record['positioner_tcp_mm'],
        material_position=frame['material_position'],
        association='exact_explicit_material_position_and_later_time',
        reference_to_tcp_distance_mm=math.dist(released, record['positioner_tcp_mm']),
        reason='execution discrepancy only; TCP is never seam truth')
    return result


def stats(values):
    values = [v for v in values if finite_number(v)]
    return dict(count=len(values), maximum=max(values) if values else None,
                rms=math.sqrt(sum(v*v for v in values)/len(values)) if values else None)


def archived_execution_inventory():
    """读取固定基准真实归档清单；有TCP记录但不能据此猜造曝光/材料关联。"""
    base='output/online_correction/20261023_194602_142/'
    records=[]
    for name in ('actual_robot_path.txt','online_follow_targets.txt','vision_diagnostics.tsv','session_summary.txt'):
        path=base+name
        content=subprocess.check_output(['git','show',BASELINE+':'+path],cwd=ROOT)
        lines=content.decode('utf-8-sig').splitlines()
        records.append(dict(path=path,sha256=hashlib.sha256(content).hexdigest(),
                            records=max(0,len(lines)-1),header=lines[0] if lines else None,
                            observation_id=None,material_position=None,coordinate_frame=None))
    return dict(revision=BASELINE,files=records,actual_tcp_archive_available=True,
        same_material_association_available=False,used_as_truth=False,
        reason='real TCP, follow-target and vision archives exist; no unique exposure-to-material-location map or independent seam truth; no XYZ/time nearest pairing or guessed transform')


def score(trace, evaluation):
    """真值及场景只在评分端读取，不能送入检测、状态、控制或模型校验。"""
    kind = evaluation.get('kind')
    if kind not in (None, 'synthetic_defined_truth', 'independent_seam_truth'):
        raise ValueError('truth must be synthetic_defined_truth or independent_seam_truth; TCP/CAD is not truth')
    intervals=[]
    for index,row in enumerate(trace):
        truth=evaluation.get('observations',{}).get(row['identity']['observation_id'],{}) if kind else {}
        e=dict(scenario=truth.get('scenario'),truth_kind=kind,final_pixel_error=None,
               raw_pixel_error=None,recovery_measurement_pixel_error=None,
               measured_space_error_mm=None,released_space_error_mm=None)
        row['evaluation']=e
        if finite_point(truth.get('pixel')):
            for target,source in [('final_pixel_error','final_pixel'),('raw_pixel_error','raw_measurement_pixel'),
                                  ('recovery_measurement_pixel_error','recovery_measurement_pixel')]:
                if finite_point(row.get(source)):
                    e[target]=math.dist(row[source],truth['pixel'])
        if (finite_point(truth.get('positioner_tcp_mm'),3) and material_key(truth.get('material_position')) is not None
                and material_key(truth['material_position'])==material_key(row['material_position'])):
            for target,source in [('measured_space_error_mm','final_positioner_tcp_mm'),
                                  ('released_space_error_mm','released_positioner_tcp_mm')]:
                if finite_point(row.get(source),3):
                    e[target]=math.dist(row[source],truth['positioner_tcp_mm'])
        if not row['available']:
            if not intervals or intervals[-1]['last_record_index']!=index-1:
                intervals.append(dict(first_record_index=index,last_record_index=index,records=0,
                                      first_exposure_ms=row['identity']['timestamp_ms']))
            intervals[-1].update(last_record_index=index,records=intervals[-1]['records']+1,
                                 last_exposure_ms=row['identity']['timestamp_ms'])
    total=len(trace)
    unavailable=sum(not r['available'] for r in trace)
    measured=sum(r['available'] and r['mode']=='measured' for r in trace)
    estimated=total-unavailable-measured
    summary=dict(total_records=total,measured_records=measured,estimated_records=estimated,unavailable_records=unavailable,
        measured_ratio=measured/total if total else None,estimated_ratio=estimated/total if total else None,
        unavailable_ratio=unavailable/total if total else None,failure_intervals=intervals,
        denominator='all input records including initialization, duplicates and difficult frames')
    for key in ('final_pixel_error','measured_space_error_mm','released_space_error_mm'):
        summary[key]=stats([r['evaluation'][key] for r in trace])
    for source,name in [('bounded_prediction','prediction'),('recovery_transition','recovery_transition')]:
        summary[name+'_pixel_error']=stats([r['evaluation']['final_pixel_error'] for r in trace if r['source']==source])
    summary['recovery_measurement_pixel_error']=stats([r['evaluation']['recovery_measurement_pixel_error']
        for r in trace if r['source']=='recovery_transition'])
    summary['processing_displacement_mm']=stats([r['processing_displacement_mm'] for r in trace])
    summary['tcp_execution_discrepancy_mm']=stats([r['tcp_comparison']['reference_to_tcp_distance_mm'] for r in trace])
    return summary


def render(report):
    rows=[]
    for index,row in enumerate(report['trace']):
        ident=row['identity']
        values=[index,ident['observation_id'],ident['frame_id'],ident['timestamp_ms'],ident['reference_revision'],
                row['evaluation']['scenario'],row['source'],row['final_pixel'],row['evaluation']['final_pixel_error'],
                row['released_positioner_tcp_mm'],row['tcp_comparison']['tcp_record_id'],row['message']]
        rows.append('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in values)+'</tr>')
    counts=report['summary']
    def percentage(key):
        value=counts[key]
        return f'{value:.2%}' if value is not None else '未知'
    concise=(f"共 {counts['total_records']} 条：实测 {counts['measured_records']}（{percentage('measured_ratio')}），"
             f"估计 {counts['estimated_records']}（{percentage('estimated_ratio')}），"
             f"不可用 {counts['unavailable_records']}（{percentage('unavailable_ratio')}）。")
    intervals=[]
    for interval in counts['failure_intervals']:
        first,last=interval['first_record_index']+1,interval['last_record_index']+1
        label=f'第 {first} 条' if first==last else f'第 {first}–{last} 条'
        begin,end=interval['first_exposure_ms'],interval['last_exposure_ms']
        timing=f'（{begin}–{end} ms）' if begin is not None and end is not None else '（曝光时间未知）'
        intervals.append(label+timing)
    failures='不可用区间：'+('；'.join(intervals) if intervals else '无')+'。全部输入均纳入分母。'
    return '''<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>组合离线回放</title>
<style>*{box-sizing:border-box}body{font:15px system-ui;margin:clamp(12px,3vw,24px);background:#f7f8fa;color:#18212b;overflow-wrap:anywhere}h1{font-size:clamp(22px,3vw,30px)}.summary{background:#edf1f6;border-radius:6px;padding:12px;line-height:1.6}.summary p{margin:0 0 6px}.table-scroll{max-width:100%;overflow-x:auto;border:1px solid #ccd2db}table{border-collapse:collapse;width:100%;min-width:1100px}td,th{border:1px solid #ccd2db;padding:7px;text-align:left;max-width:360px}pre{white-space:pre-wrap;overflow-wrap:anywhere;word-break:break-word;max-height:60vh;overflow:auto}details{margin-top:10px}summary{cursor:pointer}tr:nth-child(even){background:#edf1f6}a{color:#1557a5}</style>
<h1>实测 KCF 为主：组合离线回放</h1>
<p>实际 process → 身份/母材/恢复证据 → 空间资格 → 独立校验 → 实测参考 → 轨迹 ICP 对照。TCP仅为执行记录，合成TCP不是真值。现场最大横向偏差仍待定。</p>
<p><a href="detector/viewer.html">原图与检测</a> · <a href="report.json">完整分层记录</a> · <a href="report.md">验收及生产清单</a> · <a href="reproducibility.json">版本/配置/哈希</a></p>
<h2>全部输入统计</h2><div class="summary"><p>'''+html.escape(concise)+'''</p><p>'''+html.escape(failures)+'''</p><details><summary>展开完整统计与分层误差</summary><pre>'''+html.escape(json.dumps(report['summary'],ensure_ascii=False,indent=2))+'''</pre></details></div>
<h2>逐记录追溯</h2><p>表格可在下方区域横向滚动，完整字段保留。</p><div class="table-scroll" role="region" aria-label="逐记录追溯表" tabindex="0"><table><thead><tr><th>序号</th><th>观测</th><th>曝光</th><th>时间ms</th><th>参考版本</th><th>评分场景</th><th>来源</th><th>最终像素</th><th>误差px</th><th>释放参考mm</th><th>后续同材料TCP</th><th>诊断</th></tr></thead><tbody>'''+''.join(rows)+'''</tbody></table></div>
<p>未知值保留None/null。缺少曝光、工件路径或材料位置时不配对；不按最近点或同时刻产生精度结论。候选参数未现场标定，无运动发送。</p>'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest',type=pathlib.Path)
    for key in ('context','tcp','evaluation','policy-config','control-config'):
        parser.add_argument('--'+key,type=pathlib.Path,required=key=='context')
    parser.add_argument('--output',type=pathlib.Path,required=True)
    parser.add_argument('--cache',type=pathlib.Path,default=ROOT/'output/analysis/detector_replay_build')
    args=parser.parse_args()
    if args.output.exists():
        parser.error('output exists; preserve evidence and choose another output')
    inputs=jsonl(args.manifest)
    ids=[r.get('observation_id') for r in inputs]
    if any(v is not None and (not isinstance(v,str) or not v.strip()) for v in ids):
        raise ValueError('observation_id must be nonempty string or null')
    known=[v for v in ids if v is not None]
    if len(known)!=len(set(known)):
        raise ValueError('observation_id must uniquely identify each record')
    context=json.loads(args.context.read_text(encoding='utf-8'))
    tcp=jsonl(args.tcp) if args.tcp else []
    evaluation=json.loads(args.evaluation.read_text(encoding='utf-8')) if args.evaluation else {}
    args.output.mkdir(parents=True)
    excluded=('truth_pixel','scene','scenario','truth','evaluation','material_position','observation_id')
    detector_inputs=[{k:v for k,v in r.items() if k not in excluded} for r in inputs]
    for frame in detector_inputs:
        if frame.get('image'):
            frame['image']=str((args.manifest.parent/frame['image']).resolve())
    manifest=args.output/'detector-input.jsonl'
    manifest.write_text(''.join(json.dumps(v)+'\n' for v in detector_inputs),encoding='utf-8')
    command=[sys.executable,str(HERE/'replay.py'),str(manifest),'--evidence','--cache',str(args.cache),'--output',str(args.output/'detector')]
    if args.policy_config:
        command+=['--policy-config',str(args.policy_config)]
    run=subprocess.run(command,capture_output=True,text=True)
    (args.output/'detector-run.txt').write_text(run.stdout+run.stderr,encoding='utf-8')
    if run.returncode:
        raise RuntimeError('detector replay failed: '+run.stderr)
    evidence=jsonl(args.output/'detector/results.jsonl')
    control_config=json.loads(args.control_config.read_text()) if args.control_config else None
    control=ControlReplaySession(control_config).replay(evidence,context)
    validation=ValidationReplaySession().replay(evidence,context)
    if len(evidence)!=len(inputs) or len(control['release_diagnostics'])!=len(inputs):
        raise RuntimeError('trace cardinality mismatch; refusing to truncate difficult records')
    trace=[]
    release_index=0
    for frame,row,diagnostic in zip(inputs,evidence,control['release_diagnostics']):
        released=None
        if diagnostic['accepted']:
            released=control['released_positioner_tcp_mm'][release_index]
            release_index+=1
        identity={k:frame.get(k) for k in ('observation_id','frame_id','timestamp_ms','reference_revision')}
        identity.update(kind=frame.get('kind'),pixel_coordinate_frame='source_image_pixel',spatial_coordinate_frame=context.get('coordinate_frame'))
        trace.append(dict(identity=identity,material_position=frame.get('material_position'),
            raw_measurement_pixel=row.get('raw_measurement_pixel'),raw_detector_pixel=row['raw_detector'].get('final_pixel'),
            raw_projected_positioner_tcp_mm=row['raw_detector'].get('positioner_tcp_mm'),
            final_pixel=row['final_pixel'],final_positioner_tcp_mm=row.get('positioner_tcp_mm'),
            recovery_measurement_pixel=row.get('recovery_measurement_pixel'),source=row['source'],trust=row['trust'],
            mode=row['mode'],available=row['available'],message=row['message'],released_positioner_tcp_mm=released,
            control_qualification=diagnostic,validation_qualification=validation['qualification'][len(trace)],
            processing_displacement_mm=math.dist(released,row['positioner_tcp_mm']) if released is not None else None,
            tcp_comparison=compare_tcp(frame,released,tcp)))
    summary=score(trace,evaluation)
    control['released_reference_observation_ids']=[r['identity']['observation_id'] for r in trace
        if r['released_positioner_tcp_mm'] is not None]
    control['snapshot_endpoint_observation_ids']=([control['released_reference_observation_ids'][0],
        control['released_reference_observation_ids'][-1]] if len(control['released_reference_observation_ids'])>=2 else [])
    control['icp_same_material_correspondence_available']=False
    control['icp_comparison_scope']='actual processor output and nearest-polyline diagnostic only; no invented resampled material identity'
    report=dict(trace=trace,summary=summary,control=control,validation=validation,tcp_records=tcp,
        real_archived_execution=archived_execution_inventory(),
        field_accuracy_verified=False,maximum_lateral_deviation_mm=None,production_modified=False,motion_sent=False,
        synthetic_material_scope='caller-defined frozen path parameter and pairing labels; not verified physical arc-length or a camera/workpiece/robot simulation',
        passed=['production detector image branch executed','evidence policy and actual surface validation executed',
                'all records retained; explicit material association; evaluator truth isolated'],
        failed=[dict(record_index=i,reason=r['message']) for i,r in enumerate(trace) if not r['available']],
        unverified=['field seam truth and maximum lateral tolerance','real continuous tack/occlusion sequence',
                    'real exposure calibration, pose and material mapping','actual TCP material association and accuracy',
                    'full online queue and continuous PID','production permission and Vision/Control coordination',
                    'Windows VS2022/MSVC build/runtime','equipment motion'])
    save(args.output/'report.json',report)
    (args.output/'index.html').write_text(render(report),encoding='utf-8')
    (args.output/'report.md').write_text(REPORT_CHECKLIST,encoding='utf-8')
    portable=jsonl(args.output/'detector/manifest.jsonl')
    for original,frame in zip(inputs,portable):
        frame.update(observation_id=original.get('observation_id'),material_position=original.get('material_position'))
        if frame.get('image'):
            frame['image']='detector/'+frame['image']
    (args.output/'manifest.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in portable),encoding='utf-8')
    for name,value in [('context',context),('policy',json.loads(args.policy_config.read_text()) if args.policy_config else {}),
                       ('control-config',control_config or {}),('evaluation',evaluation)]:
        save(args.output/(name+'.json'),value)
    (args.output/'tcp.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in tcp),encoding='utf-8')
    sources=list(HERE.glob('*.py'))+list(HERE.glob('*.cpp'))
    save(args.output/'reproducibility.json',dict(
        source_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),baseline_revision=BASELINE,
        source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        original_manifest_sha256=hashlib.sha256(args.manifest.read_bytes()).hexdigest(),
        files_sha256={str(p.relative_to(args.output)):hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.rglob('*') if p.is_file()},
        dependency_versions=json.loads((args.output/'detector/metadata.json').read_text())['dependency_versions'],
        msvc_validated=False,parameters_field_validated=False,truth_enters_detector=False,
        production_changed_paths=subprocess.check_output(['git','diff','--name-only',BASELINE,'--',
            'Vision','Control','UI','Localization','DigitalTwin','RobotComm','Camera','IBVS'],cwd=ROOT,text=True).splitlines()))
    print(json.dumps(dict(output=str(args.output),summary=summary),ensure_ascii=False))


REPORT_CHECKLIST = '''# 组合离线验收

现场最大横向偏差：待定。误差只对明确声明的评分真值有效。TCP为执行结果，不能作焊缝真值。有真实TCP归档，但缺逐曝光/材料位置关联，未做现场精度；report.json保留实际记录数量、表头和哈希。

## 已运行与失败项

实际检测器图像分支、证据会话、独立焊点资格和归档母材后验均运行。控制参考、真实跟随器及ICP状态详见report.json。不可用原因见failed，不从分母剔除；表面后验未晋升不等于未运行。

## 生产与Windows验证清单

- [ ] Vision/Control维护者确认生产契约及修改授权。
- [ ] 现场标定、曝光姿态、参考版本及工件坐标关联；偏移只应用一次。
- [ ] 沿缝材料位置可追溯，前视观测与后续TCP按同材料位置关联。
- [ ] 独立现场真值、最大横向偏差与估计点执行/减速/停止策略确定。
- [ ] 真实连续预焊、反光、遮挡、单侧、参考切换和恢复回放。
- [ ] Windows VS2022当前配置构建及测试，观察最终输出；macOS不替代MSVC。
- [ ] 完整在线队列、连续PID及日志验证；设备运动另获授权。

复现组合入口：manifest.jsonl --context context.json --tcp tcp.jsonl --evaluation evaluation.json --policy-config policy.json --control-config control-config.json --output 新目录。版本、依赖、配置和哈希见reproducibility.json。历史输出不覆盖。
'''


if __name__=='__main__':
    main()
