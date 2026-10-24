"""Independent artifact consistency and input-preservation checks; no devices."""
from pathlib import Path
import csv
import hashlib
import json
import math

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]


def load(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def check_hashes(path):
    hashes = load(path)
    changed = []
    for relative, digest in hashes.items():
        source = ROOT / relative
        if not source.exists() or hashlib.sha256(source.read_bytes()).hexdigest() != digest:
            changed.append(relative)
    assert not changed, str(changed)
    return len(hashes)


def main():
    protected = check_hashes(OUT / 'protected_inputs_before.json')
    sources = check_hashes(OUT / 'incident_input_hashes.json')
    surface_hashes = check_hashes(OUT / 'surface/input_hashes.json')
    field = load(OUT / 'surface/field_cases.json')
    summary = load(OUT / 'surface/summary.json')
    assert len(field) == summary['field_candidate_count'] == 52
    with (OUT / 'surface/field_metrics.csv').open(encoding='utf-8-sig') as stream:
        table = list(csv.DictReader(stream))
    assert len(table) == len(field)
    for case in field:
        fit = set(map(int, case['train_frame_ids']))
        holdout = set(map(int, case['holdout_frame_ids']))
        assert not fit.intersection(holdout)
        assert max(fit) < min(holdout)
        assert len(holdout) == 2
        assert math.isclose(case['symmetric_gain_mm'],
                            case['baseline']['symmetric_mean_mm'] - case['temporal_holdout']['symmetric_mean_mm'],
                            rel_tol=1e-10, abs_tol=1e-10)
    assert sum(case['holdout_improved'] for case in field) == summary['temporal_holdout_improved_count'] == 10
    assert all(case['all_independent_weld_points_valid'] for case in field)
    vision = load(OUT / 'vision/summary.json')
    visual_cases = load(OUT / 'vision/experiment_v4_review/synthetic_cases.json')
    assert len(visual_cases) == vision['synthetic_candidate_case_count'] == 14
    assert sum(case['passed'] for case in visual_cases) == vision['synthetic_candidate_pass_count'] == 12
    failures = [case for case in visual_cases if not case['passed']]
    assert len(failures) == 2
    for case in failures:
        assert case['accepted'] is True and case['expected_accept'] is False
        error = math.dist(case['point'], case['truth_point'])
        assert math.isclose(error, case['error_px'], abs_tol=1e-9)
    height_cases = load(OUT / 'vision/height_v3/height_cases.json')
    assert len(height_cases) == 5 and all(case['passed'] for case in height_cases)
    paired = load(OUT / 'vision/experiment_v4_review/exact_frame_reference.json')
    assert len(paired) == 1 and paired[0]['frame_id'] == 7891
    assert paired[0]['capture_time_matches'] is True
    assert paired[0]['independent_pixel_roundtrip_error_px'] < 1e-5
    visual_hashes = check_hashes(OUT / 'vision/experiment_v4_review/input_hashes.json')
    for name, digest in vision['script_sha256'].items():
        assert hashlib.sha256((OUT / 'vision' / name).read_bytes()).hexdigest() == digest
    review = load(OUT / 'review/final_review.json')
    assert review['status'] == 'offline_phase_review_complete_not_ready_for_production'
    math_cases = load(OUT / 'review/review_math_results.json')
    intersection = math_cases['laser_intersection_is_not_material_point']
    assert intersection['new_intersection_recomputed_x_z'] == [0.0, 9.0]
    assert math.isclose(intersection['inverse_transform_counterfactual_distance'], math.sqrt(17))
    assert math_cases['surface_observability']['v_surfaces_extruded_along_seam_rank'] == 3
    assert math_cases['assertions_passed'] == 8
    executable_checks = load(OUT / 'existing_executable_checks.json')
    assert len(executable_checks) == 2
    assert all(check['status'] == 'passed' and check['rebuilt'] is False for check in executable_checks)
    browser_path = OUT / 'browser_verification.json'
    browser = load(browser_path) if browser_path.exists() else None
    if browser is not None:
        assert browser['status'] == 'passed' and browser['mouseUsed'] is False
        assert browser['coordinateImmutability'] == 'passed'
    result = {
        'status': 'passed',
        'protected_source_config_test_doc_files_unchanged': protected,
        'incident_input_hashes_verified': sources,
        'surface_archive_hashes_verified': surface_hashes,
        'visual_calibration_and_log_hashes_verified': visual_hashes,
        'temporal_blocks_disjoint_and_ordered': len(field),
        'holdout_result_consistency': 'passed',
        'visual_candidate_cases_checked': len(visual_cases),
        'known_visual_failures_preserved': len(failures),
        'height_unit_cases_checked': len(height_cases),
        'exact_real_frame_pairs_checked': len(paired),
        'independent_review': 'complete_not_ready_for_production',
        'analytic_geometry_crosscheck': 'passed',
        'existing_executable_tests_passed': len(executable_checks),
        'browser': 'passed' if browser is not None else 'pending',
        'production_code_modified': False,
        'msvc_rebuild_performed': False,
        'device_or_motion_calls': False,
        'mouse_used': False,
        'scope_limit': 'artifact consistency checks, not production acceptance or physical accuracy'
    }
    (OUT / 'verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
