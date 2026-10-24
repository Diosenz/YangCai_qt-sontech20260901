"""真实检测结果之上的离线证据会话。不是在线控制或模板训练策略。"""
import copy
import math

DEFAULT_CONFIG = {'anchor_confirmation_frames': 2}


def finite_point(value, dimensions=2):
    return (isinstance(value, (list, tuple)) and len(value)==dimensions
            and all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in value))


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
        self.anchor = None
        self.last_frame_id = None
        self.pending = 0
        self.context_key = None

    def process(self, frame, raw):
        """输入当前曝光与真实process记录，返回最终业务结果及不变原始证据。"""
        result = copy.deepcopy(raw)
        result['raw_detector'] = copy.deepcopy(raw)
        result.update(available=False, final_pixel=None, positioner_tcp_mm=None,
                      source='unavailable', trust='unavailable', can_independent_validate=False,
                      anchor=None, anchor_refreshed=False, mode='unavailable',
                      policy_unvalidated=True, policy_config=dict(self.config),
                      detector_training_policy='unmodified baseline; postprocessing cannot undo training')
        key = (frame.get('seam_index'), tuple(raw.get('image_size') or []))
        if self.context_key is not None and key != self.context_key and raw.get('image_size'):
            self.anchor = None
            self.pending = 0
        if raw.get('image_size'):
            self.context_key = key
        identity = frame.get('frame_id')
        fresh = (isinstance(identity,int) and not isinstance(identity,bool) and identity>=0
                 and (self.last_frame_id is None or identity>self.last_frame_id))
        if isinstance(identity,int) and not isinstance(identity,bool) and identity>=0:
            self.last_frame_id = max(identity,self.last_frame_id if self.last_frame_id is not None else identity)
        geometry = (raw.get('available') is True and raw.get('image_observation_accepted') is True
                    and raw.get('trust')=='independent_geometry' and raw.get('image_source')=='geometry'
                    and finite_point(raw.get('final_pixel')) and finite_point(raw.get('positioner_tcp_mm'),3))
        if fresh and geometry:
            self.pending += 1
            if self.pending >= self.config['anchor_confirmation_frames']:
                self.anchor = dict(frame_id=identity, pixel=list(raw['final_pixel']),
                                   positioner_tcp_mm=list(raw['positioner_tcp_mm']),
                                   timestamp_ms=frame.get('timestamp_ms'),
                                   reference_revision=frame.get('reference_revision'))
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
        result['anchor'] = copy.deepcopy(self.anchor)
        result['confirmation_count'] = self.pending
        result['fresh_exposure'] = fresh
        return result
