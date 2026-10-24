"""真实检测结果之上的离线证据会话；可靠checkpoint运输由replay负责，不涉及在线控制。"""
import copy
import math

DEFAULT_CONFIG = {'anchor_confirmation_frames': 2, 'max_prediction_ms': None,
                  'max_prediction_travel_mm': None,
                  'identity_corridor_pixel':12, 'identity_min_patch_score':.70,
                  'identity_min_branch_score':.55, 'identity_min_uniqueness':.04,
                  'identity_min_kcf_peak':.20}


def finite_point(value, dimensions=2):
    return (isinstance(value, (list, tuple)) and len(value)==dimensions
            and all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in value))


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def matching_exposure(value, identity):
    return (isinstance(value, int) and not isinstance(value, bool) and value >= 0
            and isinstance(identity, int) and not isinstance(identity, bool) and value == identity)


def progress_sample(frame):
    value = frame.get('weld_progress_mm')
    reference = frame.get('progress_reference')
    provenance = frame.get('progress_provenance')
    if (not finite_number(value) or not isinstance(reference, str) or not reference.strip()
            or not isinstance(provenance, str) or not provenance.strip()
            or not matching_exposure(frame.get('progress_exposure_frame_id'), frame.get('frame_id'))):
        return None
    return value, reference


def reference_sample(frame):
    revision = frame.get('reference_revision')
    point = frame.get('reference_point_at_exposure')
    if (not isinstance(revision, int) or isinstance(revision, bool) or revision < 0
            or not finite_point(point)
            or not matching_exposure(frame.get('reference_exposure_frame_id'), frame.get('frame_id'))):
        return None
    return revision, list(point)


class EvidenceSession:
    """公开回放结果的证据契约；仅新曝光的已验收几何可更新锚点。"""
    def __init__(self, config=None):
        self.config = dict(DEFAULT_CONFIG)
        if config:
            if set(config)-set(DEFAULT_CONFIG):
                raise ValueError('unknown policy configuration keys')
            self.config.update(config)
        count = self.config['anchor_confirmation_frames']
        if isinstance(count,bool) or not isinstance(count,int) or count<1:
            raise ValueError('anchor_confirmation_frames must be a positive candidate integer')
        for key in ('max_prediction_ms', 'max_prediction_travel_mm'):
            value = self.config[key]
            if value is not None and (not finite_number(value) or value <= 0):
                raise ValueError(key + ' must be a positive unvalidated candidate limit or null')
        corridor=self.config['identity_corridor_pixel']
        if isinstance(corridor,bool) or not isinstance(corridor,int) or not 1 <= corridor <= 128:
            raise ValueError('identity_corridor_pixel must be a candidate integer in [1,128]')
        for key in ('identity_min_patch_score','identity_min_branch_score','identity_min_uniqueness'):
            value=self.config[key]
            if not finite_number(value) or not 0 <= value <= 1:
                raise ValueError(key+' must be a candidate number in [0,1]')
        if not finite_number(self.config['identity_min_kcf_peak']) or self.config['identity_min_kcf_peak'] < 0:
            raise ValueError('identity_min_kcf_peak must be a nonnegative candidate number')
        self.anchor = None
        self.last_frame_id = None
        self.pending = 0
        self.context_key = None
        self.travel = 0.0
        self.last_progress = None
        self.last_timestamp = None
        self.budget_provenance_failures = []
        self.reference_mapping = None
        self.identity_history = None

    def _seed_continuation(self, frame):
        """新独立几何才重置预算；参考映射与真实锚点分开保存。"""
        self.travel = 0.0
        self.last_progress = progress_sample(frame)
        self.last_timestamp = frame.get('timestamp_ms')
        self.budget_provenance_failures = []
        if self.last_progress is None:
            self.budget_provenance_failures.append('progress_provenance_unknown')
        if not finite_number(self.last_timestamp):
            self.budget_provenance_failures.append('time_provenance_unknown')
        reference = reference_sample(frame)
        self.reference_mapping = (dict(revision=reference[0], point=reference[1],
                                       estimate=list(self.anchor['pixel']),
                                       exposure=frame['frame_id']) if reference else None)

    def _account_exposure(self, frame):
        """沿固定工件路径累计绝对弧长差，反向行程也消耗预算。缺口锁定至新锚。"""
        timestamp = frame.get('timestamp_ms')
        if (not finite_number(timestamp) or not finite_number(self.last_timestamp)
                or timestamp <= self.last_timestamp):
            self.budget_provenance_failures.append('time_provenance_unknown_or_nonmonotonic')
        self.last_timestamp = timestamp
        progress = progress_sample(frame)
        if progress is None or self.last_progress is None:
            self.budget_provenance_failures.append('progress_provenance_unknown')
        elif progress[1] != self.last_progress[1]:
            self.budget_provenance_failures.append('progress_reference_changed')
        else:
            self.travel += abs(progress[0] - self.last_progress[0])
        self.last_progress = progress

    def _continue(self, frame, result, fresh):
        failures = list(dict.fromkeys(self.budget_provenance_failures))
        result.update(prediction_elapsed_ms=None, prediction_travel_mm=self.travel,
                      prediction_failures=failures, reference_alignment='unavailable',
                      control_eligible=False, space_available=False,
                      prediction_travel_known=False, prediction_time_known=False)
        if self.anchor is None:
            failures.append('reliable_anchor_missing')
            return
        if not fresh:
            failures.append('exposure_not_fresh')
        timestamp = frame.get('timestamp_ms')
        anchored_time = self.anchor.get('timestamp_ms')
        if finite_number(timestamp) and finite_number(anchored_time):
            elapsed = timestamp - anchored_time
            result['prediction_elapsed_ms'] = elapsed
            if elapsed < 0:
                failures.append('time_provenance_unknown_or_nonmonotonic')
        else:
            elapsed = None
            failures.append('time_provenance_unknown')
        for key in ('max_prediction_ms', 'max_prediction_travel_mm'):
            if self.config[key] is None:
                failures.append(key + '_unconfigured')
        if elapsed is not None and self.config['max_prediction_ms'] is not None:
            if elapsed >= self.config['max_prediction_ms']:
                failures.append('time_budget_exhausted')
        if self.config['max_prediction_travel_mm'] is not None:
            if self.travel >= self.config['max_prediction_travel_mm']:
                failures.append('travel_budget_exhausted')
        reference = reference_sample(frame)
        if reference is None or self.reference_mapping is None:
            failures.append('reference_geometry_unavailable')
        elif reference[0] != self.reference_mapping['revision']:
            mapping = self.reference_mapping
            bridge = frame.get('previous_reference_point_at_exposure')
            if (not isinstance(frame.get('previous_reference_revision'), int)
                    or isinstance(frame.get('previous_reference_revision'), bool)
                    or frame.get('previous_reference_revision') != mapping['revision']
                    or not matching_exposure(frame.get('previous_reference_exposure_frame_id'), frame.get('frame_id'))
                    or not finite_point(bridge)):
                failures.append('reference_revision_unaligned')
            elif fresh:
                # Only OLD revision motion at the CURRENT exposure is added.
                # New-minus-old absolute CAD placement is never a motion estimate.
                mapping['estimate'] = [estimate + current - previous for estimate, current, previous
                                       in zip(mapping['estimate'], bridge, mapping['point'])]
                mapping.update(revision=reference[0], point=reference[1], exposure=frame['frame_id'])
                result['reference_alignment'] = 'same_exposure_bridge'
        else:
            mapping = self.reference_mapping
            result['reference_alignment'] = 'same_revision_relative_motion'
            if fresh:
                mapping['estimate'] = [estimate + current - previous for estimate, current, previous
                                       in zip(mapping['estimate'], reference[1], mapping['point'])]
                mapping['point'] = reference[1]
                mapping['exposure'] = frame['frame_id']
        if self.reference_mapping is not None:
            predicted = self.reference_mapping['estimate']
            size = self.context_key[1] if self.context_key is not None else ()
            if (not finite_point(predicted) or len(size) != 2
                    or not 0 <= predicted[0] < size[0] or not 0 <= predicted[1] < size[1]):
                failures.append('prediction_outside_image')
        result['prediction_travel_known'] = not any('progress_' in reason for reason in failures)
        result['prediction_time_known'] = not any('time_provenance' in reason for reason in failures)
        result['reference_mapping'] = copy.deepcopy(self.reference_mapping)
        if failures:
            result['message'] = 'bounded continuation unavailable: ' + ', '.join(dict.fromkeys(failures))
            return
        result.update(available=True, final_pixel=list(self.reference_mapping['estimate']),
                      source='bounded_prediction', trust='estimated_not_independent',
                      mode='predicted', availability_scope='pixel_only_prediction',
                      message='bounded relative-reference prediction; not a measured weld point; no spatial/control qualification')

    def _identity(self, frame, raw):
        """独立几何入锚前验收身份；外观与参考都不能自行建立可靠锚。"""
        reconstruction=raw.get('checkpoint_reconstruction')
        if reconstruction is not None and not reconstruction['valid']:
            return dict(geometry_accepted=False, appearance_accepted=False,
                        reasons=['trusted_checkpoint_reconstruction_failed'])
        if self.identity_history is None:
            return dict(geometry_accepted=True, appearance_accepted=False, reasons=[],
                        status='initial_acquisition_without_prior_target')
        from identity_evidence import inspect_identity
        expected=list(self.identity_history['pixel'])
        previous=self.identity_history.get('reference')
        current=reference_sample(frame)
        if previous and current and previous[0]==current[0]:
            expected=[p+c-h for p,c,h in zip(expected,current[1],previous[1])]
        return inspect_identity(self.identity_history,raw,expected,self.config)

    def process(self, frame, raw):
        """输入当前曝光与真实process记录，返回最终业务结果及不变原始证据。"""
        result = copy.deepcopy(raw)
        result['raw_detector'] = copy.deepcopy(raw)
        result.update(available=False, final_pixel=None, positioner_tcp_mm=None,
                      source='unavailable', trust='unavailable', can_independent_validate=False,
                      anchor=None, anchor_refreshed=False, mode='unavailable',
                      policy_unvalidated=True, policy_config=dict(self.config),
                      detector_training_policy='raw process training retained in diagnostics; checkpoint retention belongs to replay transport')
        key = (frame.get('seam_index'), tuple(raw.get('image_size') or []))
        if self.context_key is not None and key != self.context_key and raw.get('image_size'):
            self.anchor = None
            self.pending = 0
            self.identity_history = None
        if raw.get('image_size'):
            self.context_key = key
        identity = frame.get('frame_id')
        fresh = (isinstance(identity,int) and not isinstance(identity,bool) and identity>=0
                 and (self.last_frame_id is None or identity>self.last_frame_id))
        if isinstance(identity,int) and not isinstance(identity,bool) and identity>=0:
            self.last_frame_id = max(identity,self.last_frame_id if self.last_frame_id is not None else identity)
        if self.anchor is not None and fresh:
            self._account_exposure(frame)
        identity_evidence=self._identity(frame,raw)
        result['identity_evidence']=identity_evidence
        geometry = (identity_evidence['geometry_accepted'] and raw.get('available') is True and raw.get('image_observation_accepted') is True
                    and raw.get('trust')=='independent_geometry' and raw.get('image_source')=='geometry'
                    and finite_point(raw.get('final_pixel')) and finite_point(raw.get('positioner_tcp_mm'),3))
        if fresh and geometry:
            self.pending += 1
            if self.identity_history is None:
                self.identity_history=dict(image=raw.get('image_png_base64'),
                    pixel=list(raw['final_pixel']), reference=reference_sample(frame))
            if self.pending >= self.config['anchor_confirmation_frames']:
                self.anchor = dict(frame_id=identity, pixel=list(raw['final_pixel']),
                                   positioner_tcp_mm=list(raw['positioner_tcp_mm']),
                                   timestamp_ms=frame.get('timestamp_ms'),
                                   reference_revision=frame.get('reference_revision'),
                                   weld_progress_mm=frame.get('weld_progress_mm'),
                                   progress_reference=frame.get('progress_reference'),
                                   progress_provenance=frame.get('progress_provenance'))
                self._seed_continuation(frame)
                self.identity_history=dict(image=raw.get('image_png_base64'),
                    pixel=list(raw['final_pixel']), reference=reference_sample(frame))
                result.update(available=True, final_pixel=list(raw['final_pixel']),
                              positioner_tcp_mm=list(raw['positioner_tcp_mm']),
                              source='geometry',trust='independent_geometry', mode='measured',
                              can_independent_validate=True, anchor_refreshed=True,
                              message='fresh accepted image geometry; reliable measured KCF anchor')
            else:
                result['message']='new geometry confirmation pending; no reliable anchor yet'
        else:
            self.pending = 0
            result['message']=('duplicate, out-of-order or unknown exposure; cannot refresh anchor'
                               if not fresh else 'no accepted geometry or valid projection; not a reliable measurement')
        if not result['anchor_refreshed']:
            self._continue(frame, result, fresh)
            if result['available'] and identity_evidence.get('appearance_accepted'):
                result.update(final_pixel=list(identity_evidence['appearance_pixel']),
                    source='appearance_tracking', trust='tracked_not_independent', mode='tracked',
                    availability_scope='pixel_only_appearance',
                    message='current image and real KCF support historical target; old anchor budgets apply')
        else:
            result.update(prediction_elapsed_ms=0, prediction_travel_mm=0,
                          prediction_failures=[], reference_alignment='anchored',
                          space_available=True, control_eligible=False,
                          prediction_travel_known=progress_sample(frame) is not None,
                          prediction_time_known=finite_number(frame.get('timestamp_ms')))
        result['continuation_inputs'] = {key: copy.deepcopy(frame.get(key)) for key in (
            'timestamp_ms', 'weld_progress_mm', 'progress_reference', 'progress_provenance',
            'progress_exposure_frame_id', 'reference_revision', 'reference_point_at_exposure',
            'reference_exposure_frame_id', 'previous_reference_revision',
            'previous_reference_point_at_exposure', 'previous_reference_exposure_frame_id')}
        result['anchor'] = copy.deepcopy(self.anchor)
        result['confirmation_count'] = self.pending
        result['fresh_exposure'] = fresh
        return result
