"""Offline task04 examples: caller-declared progress, real detector, no devices."""
import argparse
import html
import json
import math
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent


def frame(identity, timestamp, progress, reference, **updates):
    result = dict(frame_id=identity, timestamp_ms=timestamp, image='blank.pgm', kind='synthetic',
                  projection={'fixture':'synthetic_identity_1mm_per_pixel'},
                  weld_progress_mm=progress, progress_reference='workpiece-A/frozen-seam-v1',
                  progress_provenance='DEFINED caller-supplied exposure-associated workpiece path arc; not derived from TCP or truth pixels',
                  progress_exposure_frame_id=identity, reference_revision=1,
                  reference_point_at_exposure=reference, reference_exposure_frame_id=identity)
    result.update(updates)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=pathlib.Path,required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output exists; preserve evidence and choose another directory')
    args.output.mkdir(parents=True)
    visible = bytes(225 if abs(y-(260-.45*abs(x-360))) < 1.5 else 14
                    for y in range(480) for x in range(640))
    (args.output/'visible.pgm').write_bytes(b'P5\n640 480\n255\n'+visible)
    (args.output/'blank.pgm').write_bytes(b'P5\n640 480\n255\n'+bytes([14])*640*480)
    anchor = [frame(1,1000,0,[100,100],image='visible.pgm'),
              frame(2,1100,1,[101,100],image='visible.pgm')]
    cases = {
        'time-limit': (anchor + [frame(3,1200,2,[103,101]),frame(4,1300,3,[104,102])],
                       {'max_prediction_ms':200,'max_prediction_travel_mm':10}),
        'reverse-external-axis': (anchor + [frame(3,1110,3,[102,100],robot_base_tcp_mm=[0,0,0],external_s2_deg=2),
                                            frame(4,1120,1,[103,100],robot_base_tcp_mm=[0,0,0],external_s2_deg=0)],
                                  {'max_prediction_ms':1000,'max_prediction_travel_mm':4}),
        'reference-bridge': (anchor + [frame(3,1200,2,[501,500],reference_revision=2,
                                            previous_reference_revision=1,previous_reference_point_at_exposure=[102,100],
                                            previous_reference_exposure_frame_id=3),
                                        frame(4,1300,3,[502,501],reference_revision=2),
                                        frame(5,1400,4,[900,900],reference_revision=3)],
                             {'max_prediction_ms':1000,'max_prediction_travel_mm':10}),
        'progress-gap': (anchor + [frame(3,1200,None,[102,100]),frame(4,1300,3,[103,100])],
                         {'max_prediction_ms':1000,'max_prediction_travel_mm':10}),
        'within-budget-error': (anchor + [frame(3,1200,2,[103,101])],
                                {'max_prediction_ms':1000,'max_prediction_travel_mm':10})}
    evaluation = []
    links = []
    for name,(frames,config) in cases.items():
        manifest = args.output/(name+'.jsonl')
        manifest.write_text(''.join(json.dumps(row)+'\n' for row in frames),encoding='utf-8')
        config_path = args.output/(name+'-config.json')
        config_path.write_text(json.dumps(config),encoding='utf-8')
        subprocess.run([sys.executable,str(HERE/'replay.py'),str(manifest),'--evidence',
                        '--policy-config',str(config_path),'--output',str(args.output/name)],check=True)
        rows = [json.loads(line) for line in (args.output/name/'results.jsonl').read_text().splitlines()]
        summary = [dict(frame_id=row['frame_id'],available=row['available'],source=row['source'],
                        pixel=row['final_pixel'],elapsed_ms=row['prediction_elapsed_ms'],
                        travel_mm=row['prediction_travel_mm'],failures=row['prediction_failures']) for row in rows]
        entry = dict(case=name,rows=summary)
        if name == 'within-budget-error':
            # Evaluator-only latent weld location; NEVER provided to detector or policy.
            truth = [370,270]
            entry['evaluator_only_defined_hidden_weld_pixel'] = truth
            entry['pixel_error'] = math.dist(rows[-1]['final_pixel'],truth)
            entry['interpretation'] = 'budget passes but latent path differs from predicted reference motion; not control qualified'
        evaluation.append(entry)
        links.append(f'<li><a href="{name}/viewer.html">{html.escape(name)}</a></li>')
    (args.output/'evaluation.json').write_text(json.dumps(evaluation,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (args.output/'index.html').write_text('''<!doctype html><meta charset="utf-8"><h1>有限续接：实际检测入口离线验收</h1>
<p>所有图像为明确合成输入。工件沿缝进度由调用方显式定义，未用 TCP 净位移、配置速度或真值像素推导；外轴例仅演示输入计量契约，未宣称已实现外轴计量器。</p><ul>'''+''.join(links)+'''</ul>
<p>预算内误差反例见 <a href="evaluation.json">独立评估</a>。隐藏焊缝真值仅在评估器中，检测/策略不可见。预算内预测也不可据此用于运动：空间坐标未知、control_eligible=false。实际生产模板训练未被后处理撤回。</p>''',encoding='utf-8')

if __name__=='__main__':
    main()
