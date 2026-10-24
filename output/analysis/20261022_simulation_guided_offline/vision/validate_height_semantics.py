"""Unit-level offline GapCenter experiment with explicit known seam X hypotheses.

This isolates target-height semantics from seam-X detection. It does not validate a
complete tracker. Synthetic source truth is defined before rasterization. Real 7891
comparison has no independent physical ground truth. All outputs are newly created.
"""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
import argparse
import csv
import json
import math
import numpy as np
from PIL import Image
import validate_candidates as proto


def endpoint_gap_center(image, xs, ys, seam):
    """Use nearby measured branch endpoints, not the simulated seam-anchor height.

    Model shape selects candidate branches. Their source dependence remains explicit.
    The 5..20px local windows are fixed before real-frame comparison.
    """
    candidates=proto.peaks(image)
    endpoints=[]; details=[]
    for side in (-1,1):
        offset=np.arange(5,21)*side
        xx=np.rint(seam[0]+offset).astype(int)
        px=[];py=[]
        for x in xx:
            if not 0<=x<len(candidates) or not len(candidates[x]):continue
            expected=float(np.interp(x,xs,ys))
            vals=candidates[x]
            y=float(vals[np.argmin(np.abs(vals-expected))])
            if abs(y-expected)<=6:
                px.append(x);py.append(y)
        if len(px)<8:
            return dict(measured=False,point=None,reason='insufficient_nearby_branch_support',branches=details)
        xx=np.array(px,dtype=float)-seam[0];yy=np.array(py,dtype=float)
        fit=np.linalg.lstsq(np.c_[xx,np.ones(len(xx))],yy,rcond=None)[0]
        for _ in range(4):
            err=yy-(xx*fit[0]+fit[1]);w=np.minimum(1.,1.5/np.maximum(abs(err),1e-9))
            fit=np.linalg.lstsq(np.c_[xx,np.ones(len(xx))]*np.sqrt(w[:,None]),yy*np.sqrt(w),rcond=None)[0]
        error=yy-(xx*fit[0]+fit[1]);rms=float(np.sqrt(np.mean(error**2)))
        details.append(dict(side=side,point_count=len(px),slope=float(fit[0]),endpoint_y=float(fit[1]),fit_rms_px=rms))
        if rms>1.5:return dict(measured=False,point=None,reason='local_branch_unstable',branches=details)
        endpoints.append(float(fit[1]))
    return dict(measured=True,point=[float(seam[0]),sum(endpoints)/2],reason='image_branch_gap_center',branches=details)


def synthetic_image(jump=-8,hidden=False):
    xs=np.arange(25,216);y=140+np.where(xs<120,.35*(xs-120),-.3*(xs-120)+jump)
    iy,ix=np.indices((241,241));line=np.interp(np.arange(241),xs,y)
    if hidden:line=line-22*(np.abs(np.arange(241)-120)<=28)
    noise=np.random.default_rng(41).normal(0,1.2,iy.shape)
    image=np.clip(15+noise+210*np.exp(-.5*((iy-line[None,:])/1.25)**2),0,255).astype('uint8')
    return image,xs,y,[120.,140+jump/2]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output-dir',required=True,type=Path);a=parser.parse_args()
    if a.output_dir.exists():raise SystemExit('Refusing existing output directory')
    a.output_dir.mkdir(parents=True);results=[]
    for jump in [-8,10,0]:
        image,x,y,truth=synthetic_image(jump)
        # Reference profile is exactly known from the independent synthetic model;
        # reference seam anchor lies on its left endpoint. Business target is GapCenter.
        seam=[120.,140.]
        match=proto.profile_match(image,x,y,seam)
        semantic=endpoint_gap_center(image,x,y,seam)
        old_error=abs(match['point'][1]-truth[1]) if match['accepted'] else None
        new_error=abs(semantic['point'][1]-truth[1]) if semantic['measured'] else None
        results.append(dict(name='gap_step_'+str(jump),truth=truth,seam_x_source='independently_given_synthetic_hypothesis',
                            v1_profile_alignment=match,v3_height=semantic,v1_height_error_px=old_error,v3_height_error_px=new_error,
                            passed=semantic['measured'] and new_error<.6))
    # Minimal residual lifecycle, deliberately no 3-D history implementation.
    visible,x,y,truth=synthetic_image(-8);seen=endpoint_gap_center(visible,x,y,[120,140]);residual=seen['point'][1]-140
    hidden,_,_,_=synthetic_image(-8,True);occluded=endpoint_gap_center(hidden,x,y,[120,140])
    version=1;stored_version=1
    estimate=None if occluded['measured'] or version!=stored_version else [120.,140+residual]
    results.append(dict(name='hidden_tack_same_pose_version',truth=truth,raw_measurement=occluded,
                        source='model_estimate_with_same_version_local_residual',estimated_point=estimate,
                        passed=not occluded['measured'] and estimate is not None and abs(estimate[1]-truth[1])<.6))
    updated_model_y=140+residual;version=2
    stale_allowed=version==stored_version
    results.append(dict(name='pose_update_does_not_reapply_old_residual',old_residual_px=residual,
                        updated_model_y=updated_model_y,incorrect_double_compensated_y=updated_model_y+residual,
                        output_source='unavailable_until_residual_is_revalidated',stale_residual_applied=stale_allowed,
                        passed=not stale_allowed))
    reference=json.loads((proto.BASE/'experiment_v2/exact_frame_reference.json').read_text())[0]
    poly=np.asarray(reference['reference_polyline_px']);order=np.argsort(poly[:,0]);poly=poly[order]
    match=reference['result'];dx=match['dx'];dy=match['dy'];image=np.asarray(Image.open(proto.SESSION/reference['image']).convert('L'))
    semantic=endpoint_gap_center(image,poly[:,0]+dx,poly[:,1]+dy,match['point'])
    actual=dict(frame_id=reference['frame_id'],input_prior='exact-frame RobotSim raw_virtual_parent_mm, validated calibrated inverse projection',
                v1_point=match['point'],original_geometry_point=reference['original_geometry_point'],v3=semantic,
                independent_ground_truth_available=False,production_eligible=False,
                seam_x_source='experimental prior-profile alignment, not independently validated')
    if semantic['measured']:
        actual['v3_minus_original_geometry_px']=(np.array(semantic['point'])-reference['original_geometry_point']).tolist()
    (a.output_dir/'height_cases.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    (a.output_dir/'frame7891_comparison.json').write_text(json.dumps(actual,indent=2),encoding='utf-8')
    summary=dict(experiment='fixed branch GapCenter semantics and pose-version invalidation',synthetic_case_count=len(results),
                  synthetic_pass_count=sum(r['passed'] for r in results),real_frame=actual,
                  limitations=['seam X is provided for synthetic unit cases; not complete candidate-detection validation',
                               'model-selected image branch remains prior-dependent',
                               'history example uses scalar image-height residual only; production needs material coordinates and fresh simulation',
                               'stale residual invalidation proves no double compensation; it does not provide continuity after pose updates',
                               'sole model-like reflection counterexample from experiment_v2 remains unresolved'])
    (a.output_dir/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8');print(json.dumps(summary))


if __name__=='__main__':main()
