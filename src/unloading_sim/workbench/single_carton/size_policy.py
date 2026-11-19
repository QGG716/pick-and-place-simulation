"""Explicit choice of per-instance measurement, patches, or a known size prior."""
import numpy as np

NO_PRIOR_POLICIES=('MEASURED_PER_INSTANCE','UNKNOWN_PER_INSTANCE')


def size_policy(geometry):
    geometry=geometry or {}
    policy=geometry.get('size_policy','CONFIGURED_SINGLE_TYPE')
    if policy not in ('CONFIGURED_SINGLE_TYPE',*NO_PRIOR_POLICIES):
        raise ValueError('CARTON_SIZE_POLICY_INVALID')
    basis=geometry.get('size_policy_basis')
    if policy in NO_PRIOR_POLICIES and (not isinstance(basis,str) or not basis.strip()):
        raise ValueError('CARTON_SIZE_POLICY_BASIS_REQUIRED')
    return policy


def neighbor_depth_bound(geometry):
    """Never use a known previous carton to bound a different unknown carton."""
    geometry=geometry or {}
    if size_policy(geometry) in NO_PRIOR_POLICIES:return None
    prior=geometry.get('prior_size_m')
    return float(np.linalg.norm(prior))+float(geometry.get('dimension_tolerance_m',0.)) if prior else None


def size_policy_notice(geometry):
    try:policy=size_policy(geometry)
    except ValueError as error:return '箱体尺寸配置错误：'+str(error)
    if policy=='MEASURED_PER_INSTANCE':
        return '逐箱 RGB-D 实测尺寸（不依赖先验）：从可见面及侧面边界恢复；证据不足时保留接触面并说明缺失维度。'
    if policy=='UNKNOWN_PER_INSTANCE':
        return '各箱尺寸未知：保留实测接触面；历史统一箱型先验停用，完整体积尚未确定。'
    return '采用已配置的同箱型尺寸先验；仅适用于符合该箱型的纸箱。'


def mark_incomplete_scene(scene,geometry,rejected):
    policy=size_policy(geometry)
    if policy in NO_PRIOR_POLICIES:
        scene.update(unbounded_carton_indices=[r['detection_index'] for r in rejected],
            geometry_qualification=('INCOMPLETE_INSTANCE_EXTENTS' if rejected else 'MEASURED_INSTANCE_EXTENTS'),size_policy=policy,
            shared_prior_applied=False,hidden_carton_volume_assumed_clear=False)
