"""身份始终启用，同时打开母材约束及恢复候选参数的公开 CLI 组合验收。"""
import unittest
import test_identity_continuation
import test_recovery
import test_mother_constraints


def combined(config):
    options=dict(enable_mother_constraints=True, recovery_confirmation_frames=2,
                 max_recovery_step_pixel=1, max_reacquisition_distance_pixel=12)
    options.update(config or {})
    return options


class CombinedRecoveryTests(test_recovery.RecoveryTests):
    def run_sequence(self, frames, config=None):
        rows=test_recovery.replay_tools.EvidenceReplayTests.run_sequence(self,frames,combined(config))
        self.assertTrue(rows[1]['mother_observation']['left']['valid'])
        self.assertTrue(rows[1]['policy_config']['enable_mother_constraints'])
        self.assertIn('identity_evidence',rows[1])
        return rows


class CombinedIdentityTests(unittest.TestCase):
    def run_sequence(self, images, config=None, extra=None):
        rows=test_identity_continuation.IdentityContinuationTests.run_sequence(
            self,images,combined(config),extra)
        self.assertTrue(rows[1]['mother_observation']['left']['valid'])
        self.assertTrue(rows[1]['policy_config']['enable_mother_constraints'])
        self.assertIn('identity_evidence',rows[1])
        return rows

    def test_brighter_competitor_cannot_take_identity_or_template_with_mother_enabled(self):
        test_identity_continuation.IdentityContinuationTests.test_brighter_competitor_cannot_take_identity_or_retained_template(self)

    def test_reference_identity_conflict_still_closes_all_estimate_routes(self):
        test_identity_continuation.IdentityContinuationTests.test_relative_reference_cannot_authorize_far_similar_appearance_as_same_target(self)

    def test_normal_moving_target_remains_geometry_with_both_gates_enabled(self):
        test_identity_continuation.IdentityContinuationTests.test_normal_target_motion_keeps_geometry_and_truth_labels_cannot_change_decisions(self)


    def test_actual_wrong_corner_training_is_discarded_before_clean_target_returns(self):
        raster=test_mother_constraints.raster
        rows=test_mother_constraints.MotherConstraintTests.run_case(self,
            [raster(cap=True,cap_radius=12,cap_height=1),raster(),raster()],
            config_override=combined({}))
        rejected=rows[2]
        self.assertEqual(rejected['raw_detector']['source'],'geometry')
        self.assertTrue(rejected['raw_detector']['training_committed'])
        self.assertTrue(rejected['temporary_training_discarded'])
        self.assertFalse(rejected['template_retained_for_next_exposure'])
        self.assertEqual(rejected['checkpoint_source_frame'],2)
        self.assertEqual(rejected['anchor']['frame_id'],2)
        self.assertTrue(rows[-1]['can_independent_validate'])
        self.assertLess(abs(rows[-1]['final_pixel'][0]-360),2)

if __name__=='__main__':
    unittest.main()
