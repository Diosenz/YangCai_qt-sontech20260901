"""实测主导的独立控制回放；调用真实跟随器，不发送运动。"""
import argparse
import copy
import hashlib
import json
import math
import pathlib
import shlex
import subprocess
import sys
import tempfile

from evidence_session import finite_point
from evidence_qualification import independent_spatial_geometry

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def build(cache):
    if sys.platform != 'darwin':
        raise RuntimeError('Auxiliary clang runner requires macOS; production uses VS2022')
    sources = [HERE/'ControlReplay.cpp', ROOT/'Control/src/OnlineCorrectionPathFollower.cpp',
               ROOT/'Vision/src/geometry/PositionerKinematics.cpp',
               ROOT/'Control/src/OnlineCorrectionExceptionProcessor.cpp',
               ROOT/'Vision/src/geometry/IcpRegistrationMatcher.cpp']
    sources += [ROOT/'Vision/third_party/libicp'/(name+'.cpp') for name in
                ['icp','icpPointToPoint','icpPointToPlane','kdtree','matrix']]
    includes = [ROOT/(p+'/include') for p in ['Control','Vision','RobotComm']] + [ROOT/'Vision/third_party']
    flags = shlex.split(subprocess.check_output(
        ['pkg-config','--cflags','--libs','Qt6Core','eigen3'],text=True))
    files = sources+[p for directory in includes for p in directory.rglob('*.h')]
    hashes = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    key = hashlib.sha256(json.dumps([hashes,flags],sort_keys=True).encode()).hexdigest()[:16]
    cache.mkdir(parents=True,exist_ok=True)
    binary = cache/('ControlReplay-'+key)
    if not binary.exists():
        subprocess.run(['clang++','-std=c++17','-O1',*[str(p) for p in sources],
                        *[arg for p in includes for arg in ['-I',str(p)]],*flags,'-o',str(binary)],check=True)
    return binary, hashes


def nearest_polyline_distance(point, polyline):
    distances = []
    for a,b in zip(polyline,polyline[1:]):
        ab = [q-p for p,q in zip(a,b)]
        length2 = sum(v*v for v in ab)
        t = max(0,min(1,sum((p-q)*d for p,q,d in zip(point,a,ab))/length2)) if length2 else 0
        distances.append(math.dist(point,[p+t*d for p,d in zip(a,ab)]))
    return min(distances) if distances else None


class ControlReplaySession:
    """公开离线参考释放入口；输入03证据契约，保留所有拒绝理由。"""
    def __init__(self, config=None, cache=None):
        self.config = {'max_processing_displacement_mm':0.0, 'max_reference_chord_deviation_mm':0.0}
        if config:
            if set(config)-set(self.config):
                raise ValueError('unknown control replay configuration')
            self.config.update(config)
        for limit in self.config.values():
            if isinstance(limit,bool) or not isinstance(limit,(float,int)) or not math.isfinite(limit) or limit<0:
                raise ValueError('processing limits must be finite nonnegative candidates')
        self.cache = pathlib.Path(cache) if cache else ROOT/'output/analysis/control_replay_build'

    def replay(self, evidence, context, proposals=None):
        """返回同工件系实测/释放、模型对照、真实跟随器首周期参考与P。"""
        if context.get('coordinate_frame') != 'PositionerTcp':
            raise ValueError('explicit PositionerTcp coordinate frame required; no double conversion')
        provenance = context.get('spatial_provenance',{})
        if (provenance.get('kind') != 'synthetic_fixture'
                or not isinstance(provenance.get('frame_ids'),list)):
            raise ValueError('only explicitly identified synthetic calibrated fixture is enabled; real exposure verification pending')
        dimensions = [('tool_tcp_in_last_link_mm',3),('station_base_translation_mm',3),
                      ('robot_pose_mm_deg',6),('axes_deg',2)]
        if context.get('calibration_valid') is not True or any(not finite_point(context.get(k),n) for k,n in dimensions):
            raise ValueError('valid finite calibration and feedback required')
        if len(context.get('teach_last_link_mm',[]))<2 or any(not finite_point(p,3) for p in context['teach_last_link_mm']):
            raise ValueError('finite taught direction required')
        model = context.get('model_positioner_tcp_mm',[])
        if any(not finite_point(p,3) for p in model):
            raise ValueError('model points must be finite')
        result = dict(raw_evidence=copy.deepcopy(evidence), raw_measured_positioner_tcp_mm=[],
                      released_positioner_tcp_mm=[], release_diagnostics=[], motion_sent=False, spatial_provenance=copy.deepcopy(provenance),
                      real_exposure_control_enabled=False,
                      policy_sha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),
                      policy_parameters_field_validated=False, config=dict(self.config),
                      model_counterfactual_kind='supplied model line only; original processor executes separately in actual_icp_baseline',
                      reference_selection='first and last accepted point; local straight-segment snapshot',
                      follow={'valid':False,'message':'fewer than two qualified measurements'})
        if proposals is not None and len(proposals)!=len(evidence):
            raise ValueError('one processing proposal per evidence record required')
        last_id = None
        for index,row in enumerate(evidence):
            frame = row.get('frame_id')
            fresh = isinstance(frame,int) and not isinstance(frame,bool) and (last_id is None or frame>last_id)
            if isinstance(frame,int) and not isinstance(frame,bool):
                last_id = max(frame,last_id if last_id is not None else frame)
            point = row.get('positioner_tcp_mm')
            qualified = independent_spatial_geometry(row,context,fresh)
            diagnostic = dict(frame_id=frame, accepted=False, reason='not qualified independent fresh measurement')
            if qualified:
                result['raw_measured_positioner_tcp_mm'].append(list(point))
                proposal = proposals[index] if proposals is not None else point
                displacement = math.dist(point,proposal) if finite_point(proposal,3) else math.inf
                if displacement<=self.config['max_processing_displacement_mm']:
                    result['released_positioner_tcp_mm'].append(list(proposal))
                    diagnostic.update(accepted=True, reason='qualified measured reference',processing_displacement_mm=displacement)
                else:
                    # 拒绝处理提案，保留实测，绝不静默以模型替换。
                    result['released_positioner_tcp_mm'].append(list(point))
                    diagnostic.update(accepted=True,reason='processing proposal rejected; original measurement retained',
                                      rejected_processing_displacement_mm=displacement if math.isfinite(displacement) else None)
            result['release_diagnostics'].append(diagnostic)
        distances = [nearest_polyline_distance(p,model) for p in result['raw_measured_positioner_tcp_mm']]
        distances = [v for v in distances if v is not None]
        result['model_distance_diagnostic'] = {
            'max_nearest_polyline_distance_mm':max(distances) if distances else None,
            'is_true_seam_error':False,
            'meaning':'geometric nearest-polyline discrepancy only; not row-paired or same-arc ground truth'}
        released = result['released_positioner_tcp_mm']
        if len(released)>=2:
            chord=[released[0],released[-1]]
            deviation=max(nearest_polyline_distance(p,chord) for p in released)
            result['reference_chord_deviation_mm']=deviation
            if math.dist(*chord)<=1e-9 or deviation>self.config['max_reference_chord_deviation_mm']+1e-9:
                result['follow']['message']='reference degenerate or chord would alter measured path beyond candidate limit'
                return result
            binary,hashes = build(self.cache)
            folder = pathlib.Path(tempfile.mkdtemp(prefix='kcf-control-snapshot-'))
            payload = folder/'input.json'
            payload.write_text(json.dumps({'released':result['released_positioner_tcp_mm'],'context':context,'raw_measured':result['raw_measured_positioner_tcp_mm']}),encoding='utf-8')
            output = subprocess.check_output([str(binary),str(payload)],text=True)
            result.update(json.loads(output))
            baseline=result.get('actual_icp_baseline')
            if baseline:
                distances=[nearest_polyline_distance(p,baseline['released_positioner_tcp_mm'])
                           for p in result['raw_measured_positioner_tcp_mm']]
                distances=[v for v in distances if v is not None]
                baseline['max_raw_to_released_polyline_distance_mm']=max(distances) if distances else None
                baseline['distance_is_true_seam_error']=False
            result['source_hashes'] = hashes
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evidence',type=pathlib.Path)
    parser.add_argument('context',type=pathlib.Path)
    parser.add_argument('--output',type=pathlib.Path,required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('output exists; preserve previous evidence')
    rows = [json.loads(line) for line in args.evidence.read_text().splitlines() if line.strip()]
    context = json.loads(args.context.read_text())
    result = ControlReplaySession().replay(rows,context)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'follow_valid':result['follow']['valid'],'motion_sent':False}))

if __name__=='__main__':
    main()
