"""原图外观身份候选策略：历史走廊优先，分数不能授权换目标。"""
import hashlib
import json
import math
import pathlib
import shlex
import subprocess
import tempfile

from native_runtime import configured_native

HERE = pathlib.Path(__file__).resolve().parent
_BINARY = None


def _binary():
    native = configured_native('IdentityEvidence')
    if native:
        return native[0]
    global _BINARY
    if _BINARY is None:
        source = HERE/'IdentityEvidence.cpp'
        flags = shlex.split(subprocess.check_output(
            ['pkg-config','--cflags','--libs-only-L','opencv4','Qt6Core'],text=True,encoding="utf-8"))
        flags += shlex.split(subprocess.check_output(['pkg-config','--libs','Qt6Core'],text=True,encoding="utf-8"))
        flags += ['-lopencv_imgcodecs','-lopencv_imgproc','-lopencv_core']
        digest=hashlib.sha256(source.read_bytes()+json.dumps(flags).encode()).hexdigest()[:16]
        cache=pathlib.Path(tempfile.gettempdir())/'sontech-kcf-identity-build'
        cache.mkdir(exist_ok=True)
        _BINARY=cache/('IdentityEvidence-'+digest)
        if not _BINARY.exists():
            subprocess.run(['clang++','-std=c++17','-O1',str(source),*flags,'-o',str(_BINARY)],check=True)
    return _BINARY


def inspect_identity(historical, raw, expected, config):
    """仅接收可靠像素历史、当前原图及非真值相对参考，不读取评估标签。"""
    candidate=raw.get('raw_measurement_pixel')
    reference_proposal=list(expected)
    reference_displacement=math.dist(expected,historical['pixel'])
    reference_motion_rejected=reference_displacement > config['identity_corridor_pixel']
    if reference_motion_rejected:
        expected=list(historical['pixel'])
    evidence={'expected_pixel':expected, 'reference_proposal_pixel':reference_proposal, 'candidate_pixel':candidate, 'reasons':[],
              'parameters_field_validated':False, 'geometry_accepted':False,
              'appearance_accepted':False, 'appearance_pixel':None}
    request={'historical_image':historical['image'], 'historical_pixel':historical['pixel'],
             'current_image':raw.get('image_png_base64'), 'expected_pixel':expected,
             'candidate_pixel':candidate, 'corridor_pixel':int(config['identity_corridor_pixel'])}
    if reference_motion_rejected:
        evidence['reasons'].append('reference_motion_exceeds_identity_corridor')
    process=subprocess.run([str(_binary())],input=json.dumps(request),capture_output=True,text=True,encoding="utf-8")
    if process.returncode:
        evidence['reasons'].append('identity_pixels_unavailable')
        evidence['diagnostic']=process.stderr.strip()
        return evidence
    evidence.update(json.loads(process.stdout))
    corridor=config['identity_corridor_pixel']
    if candidate and math.dist(candidate,expected)>corridor:
        evidence['reasons'].append('outside_historical_target_corridor')
    if candidate and min(evidence.get('left_branch_score',-1),evidence.get('right_branch_score',-1)) < config['identity_min_branch_score']:
        evidence['reasons'].append('bilateral_historical_appearance_not_confirmed')
    if evidence['historical_corridor_score'] < config['identity_min_patch_score']:
        evidence['reasons'].append('historical_target_not_visible')
    ambiguous=(evidence['second_corridor_score'] >= config['identity_min_patch_score']
               and evidence['historical_corridor_score']-evidence['second_corridor_score'] < config['identity_min_uniqueness'])
    if ambiguous:
        evidence['reasons'].append('multiple_plausible_historical_matches')
    if candidate and math.dist(candidate,evidence['matched_pixel'])>4:
        evidence['reasons'].append('candidate_disagrees_with_historical_match')
    evidence['geometry_accepted']=bool(candidate and not evidence['reasons'])
    # KCF 的真实响应和当前图像的局部历史匹配同时成立才形成外观支持；没有空间资格。
    peak=raw.get('kcf_response_peak')
    evidence['appearance_accepted']=(raw.get('raw_kcf_pixel') is not None
        and isinstance(peak,(int,float)) and peak >= config['identity_min_kcf_peak']
        and evidence['historical_corridor_score'] >= config['identity_min_patch_score'] and not ambiguous
        and math.dist(raw['raw_kcf_pixel'],evidence['matched_pixel']) <= corridor
        and not reference_motion_rejected)
    if evidence['appearance_accepted']:
        evidence['appearance_pixel']=evidence['matched_pixel']
    return evidence
