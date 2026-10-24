"""Offline proof of concept. No production changes, devices, fitting-based truth or overwrites.

Image peaks are measurements; the simulated profile only supplies a soft shape hypothesis.
The experiment estimates a bounded 2-D translation using outer shoulders, then evaluates
separate central samples. Missing central evidence is an estimate, never a measured seam.
This is not the proposed complete production detector or a continuous closed-loop replay.
"""
from pathlib import Path
import csv
import hashlib
import json
import math
import sys
import argparse
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
SESSION = ROOT / 'output/online_correction/20261021_235225_324'
ARCHIVE = ROOT / 'output/online_local_registration/20261021_155225_278_session_1'


def read_rows():
    return list(csv.DictReader((SESSION / 'vision_diagnostics.tsv').open(encoding='utf-8-sig'), delimiter='\t'))


def peaks(image):
    result = []
    for x in range(image.shape[1]):
        col = gaussian_filter1d(image[:, x].astype(float), .65)
        found, properties = find_peaks(col, prominence=40., distance=7, width=(.5, 12.))
        found = found[np.argsort(col[found])[-6:]]
        result.append(np.asarray(found, dtype=float))
    return result


def profile_match(image, reference_x, reference_y, seam, search_x=35, search_y=25):
    """Bounded soft-shape registration. Absolute proximity to prior has zero reward.

    It reports competing hypotheses and refuses one-sided or unsupported profiles.
    Central exclusion is an explicit experimental design, not automatic tack detection.
    """
    candidates = peaks(image)
    x = np.asarray(reference_x, dtype=int)
    y = np.asarray(reference_y, dtype=float)
    shoulder = np.abs(x - seam[0]) >= 24
    left = shoulder & (x < seam[0])
    right = shoulder & (x > seam[0])
    hypotheses = []
    for dx in range(-search_x, search_x + 1):
        xx = x + dx
        if xx.min() < 0 or xx.max() >= image.shape[1]:
            continue
        # Distances to every raw candidate are evaluated, not just the brightest.
        dist = np.full((2 * search_y + 1, len(x)), 30.)
        shifts = np.arange(-search_y, search_y + 1)
        for j, cx in enumerate(xx):
            if len(candidates[cx]):
                dist[:, j] = np.min(np.abs((y[j] + shifts[:, None]) - candidates[cx][None, :]), axis=1)
        for k, dy in enumerate(shifts):
            dd = dist[k]
            lc = float(np.mean(dd[left] <= 3.))
            rc = float(np.mean(dd[right] <= 3.))
            cost = float(np.mean(np.minimum(dd[shoulder], 8.)))
            hypotheses.append((cost, dx, int(dy), lc, rc, float(np.mean(dd[~shoulder] <= 3.))))
    if not hypotheses:
        return {'accepted': False, 'reason': 'no_search_support', 'point': None}
    hypotheses.sort()
    best = hypotheses[0]
    alt = next((h for h in hypotheses if math.hypot(h[1]-best[1], h[2]-best[2]) >= 8), None)
    margin = None if alt is None else alt[0] - best[0]
    accepted = best[0] <= 1.8 and min(best[3], best[4]) >= .72 and (margin is None or margin >= .28)
    reason = 'accepted' if accepted else ('ambiguous_hypotheses' if margin is not None and margin < .28 else 'insufficient_image_support')
    return dict(accepted=accepted, reason=reason, point=[float(seam[0]+best[1]), float(seam[1]+best[2])] if accepted else None,
                dx=best[1], dy=best[2], score=best[0], alternate_margin=margin,
                left_coverage=best[3], right_coverage=best[4], central_coverage=best[5],
                source=('image_supported_model_alignment' if best[5] >= .72 else 'model_estimate_from_visible_shoulders') if accepted else 'unavailable',
                fitting_exclusion_half_width_px=24)


def synthetic_suite():
    x = np.arange(30, 211)
    nominal = 140 + np.where(x < 120, .35*(x-120), -.30*(x-120))
    # An extra CAD bend is deliberately unrelated to the seam apex.
    multi = nominal + np.where(x < 65, .20*(x-65), 0.)
    yy, xx = np.indices((241, 281))
    rng = np.random.default_rng(20261022)
    cases = []
    specs = [('clear_model_offset', nominal, (18, -6), 'clear'),
             ('reflection_parallel', nominal, (18, -6), 'reflection'),
             ('tack_convex_multifold', nominal, (18, -6), 'tack'),
             ('tack_plus_reflection', nominal, (18, -6), 'both'),
             ('nominal_extra_bend', multi, (-16, 8), 'clear'),
             ('equal_strength_competing_seams', nominal, (0, 0), 'equal'),
             ('one_shoulder_missing', nominal, (18, -6), 'one_side'),
             ('blank_no_evidence', nominal, (18, -6), 'blank'),
             ('wrong_model_shape', nominal, (18, -6), 'wrong'),
             ('offset_outside_validated_search', nominal, (60, -6), 'clear')]
    arrays = {}
    for name, prior_y, shift, kind in specs:
        dx, dy = shift
        truth = np.array([120+dx, 140+dy], dtype=float)
        def stripe(offset=0., strength=210., deform=False):
            values = np.interp(np.arange(281)-dx, x, prior_y)+dy+offset
            if deform:
                values -= 22*np.maximum(0., 1-np.abs(np.arange(281)-truth[0])/18)
            if kind == 'wrong':
                values = np.full(281, truth[1]-20.)
            line = strength*np.exp(-.5*((yy-values[None, :])/1.3)**2)
            if kind == 'one_side': line[:, np.arange(281)>truth[0]-6] = 0.
            return line
        image = 14 + rng.normal(0, 1.5, yy.shape)
        if kind != 'blank': image += stripe(deform=kind in ('tack', 'both'))
        if kind in ('reflection', 'both'):
            # Partial reflection creates competing peaks, but not a second full supported seam.
            reflection = stripe(offset=-18, strength=200.)
            reflection[:, np.arange(281)>truth[0]-5] = 0.
            image = np.maximum(image, 14+reflection)
        if kind == 'equal': image = np.maximum(image, 14+stripe(offset=-18))
        image = np.clip(image, 0, 255).astype('uint8')
        found = profile_match(image, x, prior_y, [120, 140])
        expected = kind not in ('equal', 'one_side', 'blank', 'wrong') and abs(dx) <= 35
        error = None if found['point'] is None else float(np.linalg.norm(np.asarray(found['point'])-truth))
        passed = found['accepted'] == expected and (not expected or error <= 2.)
        cases.append(dict(name=name, kind='synthetic_known_generator_truth', expected_accept=expected,
                          truth_point=truth.tolist(), error_px=error, passed=passed, **found))
        arrays[name] = image
    return cases, arrays


def rx(a):
    c,s=np.cos(a),np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def ry(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rz(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def rot(deg):
    a=np.deg2rad(deg); return rz(a[2])@ry(a[1])@rx(a[0])


def exact_reference(rows):
    """Only exact frame-ID overlap. Calibration fingerprint is checked separately."""
    saved={int(r['frame_id']):r for r in rows if r['source_image']}
    pairs=[]
    for path in ARCHIVE.glob('*.json'):
        if any(t in path.name for t in ['.verified.','.writeback.','.rollback.']): continue
        obj=json.loads(path.read_text(encoding='utf-8-sig'))
        for frame in obj['raw_frames']:
            fid=int(frame['frame_id'])
            if fid in saved: pairs.append((path,frame,saved[fid]))
    vals={line.split()[0]:float(line.split()[1]) for line in (ROOT/'config/calibration/SENSORPARA.txt').read_text().splitlines() if len(line.split())==2 and not line.startswith(('More','2026'))}
    cal=np.array([float(v) for line in (ROOT/'config/calibration/RBCALIB.DAT').read_text(encoding='utf-8-sig').splitlines() if line.startswith('////SRANG ') for v in line.split(' ',1)[1].split(',')])
    sr=rx(np.deg2rad(-cal[3]))@ry(np.deg2rad(-cal[4]))@rz(np.deg2rad(-cal[5])); st=-sr@cal[:3]
    cr=rot([vals[f'pose_CinT[{i}]/deg'] for i in (3,4,5)]); ct=np.array([vals[f'pose_CinT[{i}]/m']*1000 for i in range(3)])
    result=[]
    for path,frame,r in pairs:
        pose=frame['evidence_context']['robot_pose_mm_deg']; rr=rot(pose[3:6]); s1,s2=np.deg2rad(pose[6:8]); er=ry(s1)@rz(s2)
        def to_pixel(points):
            p=np.asarray(points,float)+[0,0,231]
            fixed=p@er.T; rb=(fixed-st)@sr; tool=(rb-np.array(pose[:3]))@rr; cam=(tool-ct)@cr
            und=cam[:,:2]/cam[:,2,None]*vals['f/m']; ru2=np.sum(und**2,axis=1)
            factor=2/(1+np.sqrt(1-4*vals['Kappa']*ru2))
            return und*factor[:,None]/[vals['sx/m'],vals['sy/m']]+[vals['cx'],vals['cy']]
        independent=frame['independent']; real_pixel=to_pixel([independent['real_weld_parent_mm']])[0]
        expected=np.array([float(r['geometry_x_px']),float(r['geometry_y_px'])]); error=float(np.linalg.norm(real_pixel-expected))
        virtual=to_pixel(frame['raw_virtual_parent_mm']); seam=to_pixel([independent['virtual_weld_parent_mm']])[0]
        sim_logged=[float(r['sim_x_px']),float(r['sim_y_px'])]
        image=np.asarray(Image.open(SESSION/r['source_image']).convert('L'))
        order=np.argsort(virtual[:,0]); xx=virtual[order,0];yy=virtual[order,1]
        # Exact projected CAD polyline is resampled, never fitted to the real image.
        begin=max(0,int(seam[0]-90));end=min(image.shape[1]-1,int(seam[0]+90)); xs=np.arange(begin,end+1)
        ys=np.interp(xs,xx,yy)
        found=profile_match(image,xs,ys,seam,35,25)
        result.append(dict(frame_id=int(r['frame_id']), archive=str(path.relative_to(ROOT)), capture_time_matches=str(frame['evidence_context']['capture_time_ms'])==r['camera_timestamp_ms'],
                           independent_pixel_roundtrip_error_px=error, sim_seam_projected=seam.tolist(),sim_seam_logged=sim_logged,
                           sim_seam_pixel_disagreement_px=float(np.linalg.norm(seam-sim_logged)),
                           profile_point_count=len(virtual), original_geometry_valid=r['geometry_valid'],
                           prior_source='exact-frame archived RobotSim virtual laser points, inverse calibrated projection',
                           result=found, reference_polyline_px=virtual.tolist(), image=r['source_image']))
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output-dir',type=Path,required=True);args=parser.parse_args()
    out=args.output_dir
    if out.exists(): raise SystemExit('Refusing to overwrite existing output directory')
    out.mkdir(parents=True)
    rows=read_rows();cases,arrays=synthetic_suite();pairs=exact_reference(rows)
    fingerprints=json.loads((ROOT/'output/analysis/20261021_235225_seam_refinement_composite_teach/input_hashes.json').read_text(encoding='utf-8-sig'))
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [SESSION/'vision_diagnostics.tsv',ROOT/'config/calibration/SENSORPARA.txt',ROOT/'config/calibration/RBCALIB.DAT']}
    calibration_match=all(hashes[str(p.relative_to(ROOT))]==fingerprints[str(p)] for p in [ROOT/'config/calibration/SENSORPARA.txt',ROOT/'config/calibration/RBCALIB.DAT'])
    (out/'synthetic_cases.json').write_text(json.dumps(cases,indent=2),encoding='utf-8')
    (out/'exact_frame_reference.json').write_text(json.dumps(pairs,indent=2),encoding='utf-8')
    (out/'input_hashes.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
    # Synthetic raster generation is scientific test data, not editing user media.
    for name,array in arrays.items():Image.fromarray(array).save(out/(name+'.png'))
    with (out/'synthetic_metrics.csv').open('w',newline='',encoding='utf-8-sig') as f:
        keys=['name','expected_accept','accepted','source','reason','error_px','score','alternate_margin','left_coverage','right_coverage','central_coverage','passed'];w=csv.DictWriter(f,keys,extrasaction='ignore');w.writeheader();w.writerows(cases)
    summary=dict(synthetic_case_count=len(cases),synthetic_pass_count=sum(c['passed'] for c in cases),
                 original_saved_images=sum(bool(r['source_image']) for r in rows), exact_frame_prior_pairs=len(pairs),
                 calibration_hash_matches_prior_run=calibration_match,
                 claims=['bounded shape matching can estimate a hidden nominal seam from visible shoulders in synthetic scenes',
                         'no absolute distance-to-simulation reward, competing full profiles must remain ambiguous'],
                 limitations=['not a production algorithm; fixed translation, no 3-D registration',
                              'central exclusion is specified experimental design, not automatic tack detection',
                              'only one exact-frame real image/reference pair, no physical seam ground truth',
                              'remaining 27 saved images cannot be replayed with full same-frame model profile',
                              'no continuous image recording or closed-loop accuracy validation'],
                 files=['synthetic_metrics.csv','synthetic_cases.json','exact_frame_reference.json','input_hashes.json'])
    (out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False))


if __name__=='__main__':main()
