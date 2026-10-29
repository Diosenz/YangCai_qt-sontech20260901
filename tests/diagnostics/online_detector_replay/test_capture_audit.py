"""连续采集审计必须识别漏图、预算跳过及遗漏曝光，不读取或启动设备。"""
import csv
import json
import pathlib
import struct
import tempfile
import unittest
import zlib

from audit_capture import audit


class CaptureAuditTests(unittest.TestCase):
    def fixture(self):
        # 保留测试证据，不删除目录。
        root = pathlib.Path(tempfile.mkdtemp(prefix='i2r-capture-audit-'))
        images = root / 'replay_images'
        images.mkdir()
        def chunk(kind, value):
            return struct.pack('>I', len(value)) + kind + value + struct.pack('>I', zlib.crc32(kind + value))
        png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', 1, 1, 8, 0, 0, 0, 0))
        png += chunk(b'IDAT', zlib.compress(b'\0\x80')) + chunk(b'IEND', b'')
        contexts, entries, manifest, events = [], [], [], []
        for frame in (100, 104):
            sequence = len(contexts) + 1
            stem = f'frame_{frame}'
            image = f'replay_images/{stem}_source.png'
            (root / image).write_bytes(png)
            contexts.append(dict(frame_id=frame, session_id=1, capture_sequence=sequence,
                image=image, image_size=[1, 1], detector_input_complete=True,
                paper_tracking=True, bounded_search=True, detector_actions=['reset'] if sequence == 1 else [],
                simulation_valid=False, has_weld_point=False, simulation_frame_id=frame,
                sim_line=[], seam_index=-1, timestamp_ms=1000 + frame, reference_revision=1,
                evidence_policy=True, enable_gap=False, input_image_kind='detector_gray8',
                reference_point_at_exposure=None, reference_exposure_frame_id=-1,
                evidence_options=dict(enable_mother_constraints=False, enable_continuous_output=True,
                    max_continuous_step_pixel=8, anchor_confirmation_frames=2,
                    recovery_confirmation_frames=2, identity_corridor_pixel=24,
                    identity_min_patch_score=0.55, identity_min_branch_score=0.4,
                    identity_min_uniqueness=0.02, max_recovery_step_pixel=6,
                    max_prediction_ms=0, max_prediction_travel_mm=0),
                projection=dict(provenance='synthetic audit fixture',
                    calibration=dict(focal_length_m=0.01, sx_m=1e-5, sy_m=1e-5,
                        cx_pixel=0, cy_pixel=0, kappa=0, laser_plane_a=0, laser_plane_b=0, laser_plane_c=-1),
                    camera_in_tool=[1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
                    tool_in_robot_base=[1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
                    robot_base_to_positioner_tcp=[1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0])))
            entries.append([sequence, frame, 'queued', image])
            manifest.append([stem, 'saved', len(png)])
            events.append(dict(event='observation', frame_id=str(frame), session_id='1',
                               trigger_accepted=True, phase='online_running'))
        for name, header, values in [('replay_index.tsv',
                ['sequence', 'frame_id', 'request_status', 'source_image'], entries),
                ('replay_images/image_manifest.tsv', ['stem', 'status', 'encoded_bytes'], manifest)]:
            with (root / name).open('w', encoding='utf-8', newline='') as stream:
                writer = csv.writer(stream, delimiter='\t')
                writer.writerow(header)
                writer.writerows(values)
        (root / 'session_summary.txt').write_text('session_state\tcompleted\n', encoding='utf-8')
        for name, values in [('replay_context.jsonl', contexts), ('unified_session.jsonl', events)]:
            (root / name).write_text(''.join(json.dumps(value) + '\n' for value in values), encoding='utf-8')
        return root

    def test_processed_exposures_need_not_have_consecutive_camera_ids(self):
        report = audit(self.fixture())
        self.assertTrue(report['complete'], report)
        self.assertEqual([row['frame_id'] for row in report['images']], [100, 104])

    def test_queued_without_saved_is_incomplete(self):
        root = self.fixture()
        (root / 'replay_images/image_manifest.tsv').write_text('stem\tstatus\tencoded_bytes\n', encoding='utf-8')
        self.assertFalse(audit(root)['complete'])

    def test_budget_skips_are_not_a_complete_replay(self):
        root = self.fixture()
        path = root / 'replay_index.tsv'
        path.write_text(path.read_text().replace('queued', 'frame_limit', 1), encoding='utf-8')
        self.assertFalse(audit(root)['complete'])

    def test_missing_capture_and_wrong_simulation_identity_are_detected(self):
        for fault in ('missing', 'wrong_reference'):
            with self.subTest(fault=fault):
                root = self.fixture()
                path = root / 'replay_context.jsonl'
                values = [json.loads(line) for line in path.read_text().splitlines()]
                if fault == 'missing':
                    values.pop()
                else:
                    values[0].update(simulation_valid=True, simulation_frame_id=99)
                path.write_text(''.join(json.dumps(row) + '\n' for row in values), encoding='utf-8')
                self.assertFalse(audit(root)['complete'])

    def test_running_session_and_corrupt_png_are_incomplete(self):
        for fault in ('running', 'png'):
            with self.subTest(fault=fault):
                root = self.fixture()
                if fault == 'running':
                    (root / 'session_summary.txt').write_text('session_state\trunning\n', encoding='utf-8')
                else:
                    image = next((root / 'replay_images').glob('*.png'))
                    image.write_bytes(image.read_bytes()[:-10])
                self.assertFalse(audit(root)['complete'])

    def test_missing_inputs_cannot_be_replaced_by_native_defaults(self):
        for fault in ('projection', 'evidence_options', 'reference_revision', 'timestamp_ms',
                      'has_weld_point', 'enable_gap', 'unknown_history', 'valid_point_missing',
                      'zero_rotation', 'reflection_rotation'):
            with self.subTest(fault=fault):
                root = self.fixture()
                path = root / 'replay_context.jsonl'
                values = [json.loads(line) for line in path.read_text().splitlines()]
                if fault == 'projection':
                    values[0]['projection'] = {}
                elif fault == 'unknown_history':
                    values[0]['detector_actions'] = []
                elif fault == 'valid_point_missing':
                    values[0].update(simulation_valid=True, has_weld_point=True)
                elif fault == 'enable_gap':
                    values[0]['enable_gap'] = True
                elif fault == 'zero_rotation':
                    values[0]['projection']['camera_in_tool'] = [0] * 12
                elif fault == 'reflection_rotation':
                    values[0]['projection']['tool_in_robot_base'][0] = -1
                else:
                    values[0].pop(fault)
                path.write_text(''.join(json.dumps(row) + '\n' for row in values), encoding='utf-8')
                self.assertFalse(audit(root)['complete'])


if __name__ == '__main__':
    unittest.main()
