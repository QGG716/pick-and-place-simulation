"""CPU-only state time contract; no rclpy or ML imports."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from unloading_perception.scene import state_time_reasons


@pytest.mark.parametrize('source,limit', [('robot', .5), ('mechanism', 2.)])
@pytest.mark.parametrize('case', ['now', 'boundary', 'expired', 'future', 'zero', 'negative',
                                 'nan', 'inf', 'uninitialized', 'domain'])
def test_state_time_bounds_and_clock_context(source, limit, case):
    stamp, now, initialized, domain, reason = 100., 100., True, 'ros_sim_time', None
    if case == 'boundary': stamp = now - limit
    elif case == 'expired': stamp, reason = math.nextafter(now - limit, -math.inf), 'STALE'
    elif case == 'future': stamp, reason = math.nextafter(now, math.inf), 'TIME_IN_FUTURE'
    elif case in ('zero', 'negative', 'nan', 'inf'):
        stamp, reason = {'zero': 0., 'negative': -1., 'nan': math.nan, 'inf': math.inf}[case], 'TIME_INVALID'
    elif case == 'uninitialized': now, initialized, reason = 0., False, 'CLOCK_NOT_INITIALIZED'
    elif case == 'domain': domain, reason = 'ros', 'CLOCK_DOMAIN_MISMATCH'
    result = state_time_reasons(stamp, source=source, now=now, max_age_seconds=limit,
        sample_clock_domain=domain, clock_domain='ros_sim_time', clock_initialized=initialized)
    assert result == (() if reason is None else (source.upper() + '_STATE_' + reason,))


@pytest.mark.parametrize('sec,nanosec,valid', [(100, 0, True), (100, 999999999, True),
    (0, 0, True), (-1, 0, False), (100, 1000000000, False), (100, -1, False),
    (2**31, 0, False), (math.nan, 0, False)])
def test_ros_state_time_encoding_before_float_conversion(sec, nanosec, valid):
    # common.py has no ROS import until float_to_time is used.
    path = Path(__file__).resolve().parents[1] / 'ros2_ws/src/unloading_ros_bridge/unloading_ros_bridge/common.py'
    spec = importlib.util.spec_from_file_location('state_time_common', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    stamp = SimpleNamespace(sec=sec, nanosec=nanosec)
    if valid:
        assert module.state_time_to_float(stamp) == sec + nanosec / 1e9
    else:
        with pytest.raises(ValueError, match='encoding'):
            module.state_time_to_float(stamp)
