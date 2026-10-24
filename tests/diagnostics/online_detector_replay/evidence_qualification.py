"""离线消费者共用的独立几何/空间证据资格，不能由外层标签制造资格。"""
import math

from evidence_session import finite_point, matching_exposure


def independent_image_geometry(row, fresh):
    raw=row.get('raw_detector') or {}
    return (fresh and matching_exposure(raw.get('frame_id'),row.get('frame_id'))
            and raw.get('image_observation_accepted') is True
            and raw.get('image_source')=='geometry' and finite_point(raw.get('accepted_image_pixel')))


def independent_spatial_geometry(row, context, fresh):
    """要求同曝光原图接纳及原始/最终投影一致；仅合成空间契约已验收。"""
    raw=row.get('raw_detector') or {}
    frame=row.get('frame_id')
    point=row.get('positioner_tcp_mm')
    provenance=context.get('spatial_provenance') or {}
    return (independent_image_geometry(row,fresh) and row.get('available') is True
            and row.get('kind')=='synthetic' and raw.get('kind')=='synthetic'
            and raw.get('projection_kind')=='synthetic_identity_1mm_per_pixel'
            and raw.get('available') is True and row.get('mode')=='measured'
            and row.get('source')=='geometry' and row.get('trust')=='independent_geometry'
            and raw.get('trust')=='independent_geometry' and row.get('fresh_exposure') is True
            and row.get('anchor_refreshed') is True and row.get('can_independent_validate') is True
            and isinstance(row.get('anchor'),dict) and row['anchor'].get('frame_id')==frame
            and finite_point(point,3) and finite_point(raw.get('positioner_tcp_mm'),3)
            and math.dist(point,raw['positioner_tcp_mm'])<=1e-9
            and context.get('coordinate_frame')=='PositionerTcp'
            and provenance.get('kind')=='synthetic_fixture' and frame in provenance.get('frame_ids',[]))
