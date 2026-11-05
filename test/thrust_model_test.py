"""fpv_test.md §4 runtime inference (Python mirror of quadratic_thrust_model.h)."""

import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
import propulsion_model as pm  # noqa: E402
from hw_control_command_test import ThrustModel  # noqa: E402

COEF = dict(a3=-15.0, a2=70.0, a1=6.0)  # F_std(1) = 61 N, monotonic on [0, 1]


def model(**kw):
    return pm.PropulsionModel(**{**COEF, **kw})


def exact_root(m, f_std):
    lo, hi = 0.0, pm.U_PROJECT_MAX
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if m.thrust(mid) < f_std else (lo, mid)
    return 0.5 * (lo + hi)


def test_step1_environment_scales_required_thrust():
    m = model()
    env = pm.environment_factor(85000.0, 30.0)
    # standard-state target = F_req*(P0/P)*(T/T0): same throttle as asking env*F at P0/T0
    assert m.throttle(20.0, env) == pytest.approx(m.throttle(20.0 * env, 1.0), abs=1e-12)
    assert m.throttle(20.0, env) > m.throttle(20.0, 1.0)  # thin, warm air needs more throttle


@pytest.mark.parametrize('coef', [COEF, dict(a3=0.0, a2=100.0, a1=1e-3), dict(a3=0.0, a2=0.0, a1=90.0),
                                  dict(a3=40.0, a2=60.0, a1=2.0)])
def test_step2_newton_reaches_1e5(coef):
    m = pm.PropulsionModel(**coef)
    assert m.inverse is not None
    for u in [0.05 + 0.0475 * i for i in range(21)]:
        f = m.thrust(u)
        assert abs(m.solve_standard_throttle(f) - exact_root(m, f)) < pm.NEWTON_TOL


def test_step2_inverse_cubic_initial_guess_quality():
    # Exact for pure quadratic / linear thrust, < 2e-3 throttle for a general cubic.
    assert pm.PropulsionModel(0.0, 100.0, 1e-9).inverse_max_error < 1e-6
    assert pm.PropulsionModel(0.0, 0.0, 90.0).inverse_max_error < 1e-9
    assert model().inverse_max_error < 2e-3


def test_step3_first_order_voltage_compensation():
    m = model()
    f = 30.0
    u_std = m.solve_standard_throttle(f)
    assert m.throttle(f, 1.0, 20.0) == pytest.approx(u_std * 22.2 / 20.0, abs=1e-12)
    assert m.throttle(f, 1.0, math.nan) == pytest.approx(u_std, abs=1e-12)    # no telemetry
    assert m.compensate(u_std, f, 5.0) == pytest.approx(u_std * 22.2 / 12.0, abs=1e-12)  # 12 V floor
    u_low = m.solve_standard_throttle(10.0)
    assert m.throttle(10.0, 1.0, 5.0) == pytest.approx(u_low * 22.2 / 12.0, abs=1e-12)


def test_step3_full_form_with_current_feedback():
    m = model(r_eq=0.03, k_i=1.5)
    f, current, voltage = 30.0, 50.0, 21.0
    u_std = m.solve_standard_throttle(f)
    expected = (u_std * 22.2 + (current - 1.5 * f) * 0.03) / voltage
    assert m.throttle(f, 1.0, voltage, current) == pytest.approx(expected, abs=1e-12)
    # no current sensor -> first-order formula
    assert m.throttle(f, 1.0, voltage, math.nan) == pytest.approx(u_std * 22.2 / voltage, abs=1e-12)


def test_step4_clip_and_zero_force():
    m = model()
    assert m.throttle(0.0) == m.u_idle
    assert m.throttle(0.0009) == m.u_idle
    assert m.throttle(-5.0) == m.u_idle and m.throttle(math.nan) == m.u_idle
    assert m.throttle(1e6) == 1.0
    assert m.throttle(0.05) == m.u_idle  # below idle thrust -> idle floor
    assert pm.PropulsionModel(0.0, 0.0, 0.0).throttle(10.0) == pm.DEFAULT_U_IDLE  # uncalibrated


def test_validation_and_low_pass():
    assert model().validate() == ''
    assert 'dF/du' in pm.PropulsionModel(-100.0, 50.0, 10.0).validate()
    assert pm.PropulsionModel(0.0, 50.0, 0.0).validate()                # a1 must be > 0
    assert model(u_idle=1.0).validate() and model(r_eq=-1.0).validate()
    lpf = pm.LowPassFilter(3.2, 22.2)
    lpf.update(21.2, 0.001)
    # doc §5: alpha = 0.02 at 1 kHz is a ~3.2 Hz first-order low-pass
    assert (22.2 - lpf.value) == pytest.approx(0.02, rel=0.01)
    lpf = pm.LowPassFilter(3.0, 22.2)
    tau = 1.0 / (2.0 * math.pi * 3.0)
    for _ in range(1000):
        lpf.update(20.0, tau / 1000.0)
    assert (22.2 - lpf.value) / 2.2 == pytest.approx(1.0 - math.exp(-1.0), rel=1e-3)


@pytest.mark.parametrize('voltage_comp', [False, True])
@pytest.mark.parametrize('baro_comp', [False, True])
def test_hardware_mirror_matches_node_branching(voltage_comp, baro_comp):
    mp = dict(COEF, r_eq=0.03, k_i=1.5)
    mirror = ThrustModel(mp, 6, voltage_comp, baro_comp)
    mirror.model.set_u_idle(0.06)  # as read back from the FC motor_idle
    m = pm.PropulsionModel.from_parameters(mp, 6, 0.06)
    env, vbat, amps = 1.1, 21.0, 40.0
    expected = m.collective(25.0, env if baro_comp else 1.0,
                            vbat if voltage_comp else math.nan, amps if voltage_comp else math.nan)
    assert mirror.force_to_throttle(25.0, env, vbat, amps) == pytest.approx(expected, abs=1e-12)
    force = mirror.force_for_throttle(expected, env, vbat, amps)
    assert force == pytest.approx(25.0, rel=1e-6)


def test_u_idle_is_fc_motor_idle_and_collective_zero():
    assert pm.idle_duty_from_cli('motor_idle = 550') == pytest.approx(0.055)
    assert pm.idle_duty_from_cli('motor_idle = 0\n') == 0.0
    for bad in ('motor_idle = 2001', 'motor_idle = 5.5', 'dshot_idle_value = 550', 'NOT FOUND', '', None):
        assert math.isnan(pm.idle_duty_from_cli(bad))
    m = model(u_idle=0.055)
    assert m.collective(0.0) == 0.0 and m.collective(1e6) == 1.0
    before = m.throttle(20.0)
    m.set_u_idle(0.08)                     # FC motor_idle changed: floor and mapping follow
    assert m.u_idle == 0.08 and m.inverse is not None
    assert m.throttle(0.0) == 0.08 and m.collective(0.0) == 0.0
    assert m.throttle(20.0) == pytest.approx(before, abs=1e-12)  # solved duty itself is idle-independent


@pytest.mark.parametrize('env', [0, -1, math.inf, math.nan])
def test_hardware_mirror_rejects_invalid_environment(env):
    with pytest.raises(ValueError, match='environment'):
        ThrustModel(dict(COEF), 6, True, True).force_to_throttle(10.0, env, 22.2)
