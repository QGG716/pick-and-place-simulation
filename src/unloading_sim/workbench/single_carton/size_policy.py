"""Whether the configured carton prior applies to this real scene."""
import numpy as np


def size_policy(geometry):
    geometry=geometry or {}
    policy=geometry.get('size_policy','CONFIGURED_SINGLE_TYPE')
    if policy not in ('CONFIGURED_SINGLE_TYPE','UNKNOWN_PER_INSTANCE'):
        raise ValueError('CARTON_SIZE_POLICY_INVALID')
    basis=geometry.get('size_policy_basis')
    if policy=='UNKNOWN_PER_INSTANCE' and (not isinstance(basis,str) or not basis.strip()):
        raise ValueError('CARTON_SIZE_POLICY_BASIS_REQUIRED')
    return policy


def neighbor_depth_bound(geometry):
    """Never use a known previous carton to bound a different unknown carton."""
    geometry=geometry or {}
    if size_policy(geometry)=='UNKNOWN_PER_INSTANCE':return None
    prior=geometry.get('prior_size_m')
    return float(np.linalg.norm(prior))+float(geometry.get('dimension_tolerance_m',0.)) if prior else None


def size_policy_notice(geometry):
    try:policy=size_policy(geometry)
    except ValueError as error:return '箱体尺寸配置错误：'+str(error)
    if policy=='UNKNOWN_PER_INSTANCE':
        return '各箱尺寸未知：保留实测接触面；历史统一箱型先验停用，完整体积尚未确定。'
    return '采用已配置的同箱型尺寸先验；仅适用于符合该箱型的纸箱。'


def mark_incomplete_scene(scene,geometry,rejected):
    if size_policy(geometry)=='UNKNOWN_PER_INSTANCE':
        scene.update(unbounded_carton_indices=[r['detection_index'] for r in rejected],
            geometry_qualification='INCOMPLETE_INSTANCE_EXTENTS',size_policy='UNKNOWN_PER_INSTANCE',
            shared_prior_applied=False,hidden_carton_volume_assumed_clear=False)
