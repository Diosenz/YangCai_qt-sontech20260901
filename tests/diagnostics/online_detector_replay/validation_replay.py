"""独立焊点资格消费及真实母材后验回放；不改变生产校验链。"""
import argparse
import copy
import hashlib
import json
import math
import pathlib
import re
import subprocess
import sys

from evidence_session import finite_point, matching_exposure
from evidence_qualification import independent_image_geometry, independent_spatial_geometry

HERE=pathlib.Path(__file__).resolve().parent
ROOT=HERE.parents[2]


def replay_archived_surface():
    """每次实际运行10466归档的生产surface evaluate/measure/compare/observe。"""
    script=HERE.parent/'online_correction_repro/surface_replay.py'
    run=subprocess.run([sys.executable,str(script)],capture_output=True,text=True)
    if run.returncode not in (0,1) or 'numerical_metrics_match_archive=1' not in run.stdout:
        raise RuntimeError('actual surface replay failed: '+run.stderr+run.stdout)
    posts=[]
    for line in run.stdout.splitlines():
        if line.startswith('POST '):
            posts.append({key:float(value) for key,value in re.findall(r'(\w+)=([-+\deE.]+)',line)})
    if not posts:
        raise RuntimeError('surface replay did not produce actual observations')
    return dict(executed=True,verified=run.returncode==0,
                decision='verified' if run.returncode==0 else 'not_verified',
                frame_metrics=posts, stdout=run.stdout, stderr=run.stderr,
                scope='actual pinned surface evaluator and posterior checker; no ICP solve/write',
                pinned_revision='430aaa204fd5319dd7f697d3de2b4e1faddbb06f',
                source_harness_sha256=hashlib.sha256(script.with_name('SurfaceReplay.cpp').read_bytes()).hexdigest())


class ValidationReplaySession:
    """公开离线验收入口：图像资格、空间资格、焊点结论和母材结论分别输出。"""
    def __init__(self,config=None):
        self.config={'max_weld_residual_mm':None}
        if config:
            if set(config)-set(self.config):
                raise ValueError('unknown validation configuration')
            self.config.update(config)
        threshold=self.config['max_weld_residual_mm']
        if threshold is not None and (isinstance(threshold,bool) or not isinstance(threshold,(float,int))
                                      or not math.isfinite(threshold) or threshold<=0):
            raise ValueError('weld residual candidate limit must be positive finite or null')

    def replay(self,evidence,context):
        """模型检验位置须按曝光显式提供；门槛未定时不擅自作通过结论。"""
        decisions=[]
        included=[]
        last_frame=None
        for row in evidence:
            raw=row.get('raw_detector') or {}
            frame=row.get('frame_id')
            fresh=(isinstance(frame,int) and not isinstance(frame,bool) and frame>=0
                   and (last_frame is None or frame>last_frame))
            if isinstance(frame,int) and not isinstance(frame,bool):
                last_frame=max(frame,last_frame if last_frame is not None else frame)
            image=independent_image_geometry(row,fresh)
            point=row.get('positioner_tcp_mm')
            spatial=independent_spatial_geometry(row,context,fresh)
            target=context.get('candidate_point_by_frame',{}).get(str(frame))
            residual=math.dist(point,target) if spatial and finite_point(target,3) else None
            used=residual is not None
            reason=('included independent image geometry in explicit synthetic space' if used
                    else 'no qualified image/space identity or same-exposure candidate; excluded')
            decisions.append(dict(frame_id=frame,image_geometry_eligible=image,
                                  independent_spatial_eligible=spatial,included=used,
                                  candidate_residual_mm=residual,reason=reason))
            if used:
                included.append((frame,residual))
        threshold=self.config['max_weld_residual_mm']
        verdict=('unavailable' if not included else 'threshold_pending' if threshold is None
                 else 'accepted' if all(value<=threshold for _,value in included) else 'rejected')
        # 表面云具有独立来源，焊点为空/被排除也必须执行原表面路径。
        surface=replay_archived_surface()
        return dict(raw_evidence=copy.deepcopy(evidence),context=copy.deepcopy(context),
                    qualification=decisions,weld_validation=dict(decision=verdict,
                        included_frame_ids=[frame for frame,_ in included],
                        maximum_residual_mm=max((v for _,v in included),default=None)),
                    surface_validation=surface,config=dict(self.config),
                    policy_parameters_field_validated=False,production_behavior_modified=False,
                    real_spatial_validation_enabled=False,
                    joint_verdict='separate evidence decisions; no invented production fusion rule',
                    policy_sha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evidence',type=pathlib.Path)
    parser.add_argument('context',type=pathlib.Path)
    parser.add_argument('--max-weld-residual-mm',type=float)
    parser.add_argument('--output',type=pathlib.Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        raise ValueError('output exists; preserve previous evidence')
    rows=[json.loads(line) for line in args.evidence.read_text().splitlines() if line.strip()]
    result=ValidationReplaySession({'max_weld_residual_mm':args.max_weld_residual_mm}).replay(
        rows,json.loads(args.context.read_text()))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'weld':result['weld_validation'],'surface':result['surface_validation']['decision']}))

if __name__=='__main__':
    main()
