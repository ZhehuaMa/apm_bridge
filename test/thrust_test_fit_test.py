#!/usr/bin/python3
"""Pure-function unit tests for thrust_test_node (no ROS, no hardware).

Loads the standalone calibration tool as a module (its ROS imports are guarded,
so importing it needs only numpy) and exercises the throttle<->force model math
that backs fpv_test.md's calibration pipeline:

  * inv_spin_map        : inverse of (1-k)t + k t^2 (MOT_THST_EXPO decoupling)
  * fit_motor_cubic     : least-squares x = A f^3 + B f^2 + C f + D recovery
  * vbat_scale          : (a V + b)/(a V_ref + b) voltage normalization factor,
                          symmetric with quadratic_thrust_model.h's vbatScale
  * normalize_throttle  : command throttle -> polynomial input, incl. kp / volt
  * thrust_grid         : stepped sweep grid endpoints

The full e2e fake-bridge / SITL coverage lives in thrust_test_offline_test.py
and sitl_thrust_test.py (run standalone against a sourced workspace / SITL).
"""

import importlib.util
import math
import os

import numpy as np
import pytest

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', 'scripts', 'thrust_test_node.py')

SYN = {'A': 5.0e-5, 'B': -1.9e-3, 'C': 5.8e-2, 'D': 5.6e-3}
N_MOTORS = 4


def _load():
    spec = importlib.util.spec_from_file_location('thrust_test_node', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


tool = _load()


def _syn_force_total(t):
    """Invert the synthetic cubic (Newton) -> total force [N] at throttle t."""
    f = 0.5
    for _ in range(80):
        p = SYN['A'] * f ** 3 + SYN['B'] * f ** 2 + SYN['C'] * f + SYN['D']
        dp = 3.0 * SYN['A'] * f * f + 2.0 * SYN['B'] * f + SYN['C']
        f = max(f - (p - t) / dp, 0.0)
    return N_MOTORS * f


@pytest.mark.parametrize('k', [0.0, 0.1, 0.3])
@pytest.mark.parametrize('s', [0.1, 0.5, 0.9])
def test_inv_spin_map_round_trip(k, s):
    t = (1.0 - k) * s + k * s * s
    assert tool.inv_spin_map(t, k) == pytest.approx(s, abs=1e-9)


def test_fit_motor_cubic_recovers_coefficients():
    points = [{'x_poly': t, 'force_n': _syn_force_total(t)}
              for t in (0.10, 0.20, 0.30, 0.40)]
    fit, err = tool.fit_motor_cubic(points, N_MOTORS)
    assert err is None
    for key in SYN:
        assert fit[key] == pytest.approx(SYN[key], abs=1e-6)
    # fitted force->throttle curve must be strictly increasing on the sample span
    assert fit['deriv_min'] > 0.0
    assert fit['rms'] < 1e-6


def test_fit_motor_cubic_rejects_too_few_points():
    fit, err = tool.fit_motor_cubic([{'x_poly': 0.1, 'force_n': 1.0}], N_MOTORS)
    assert fit is None and err


def test_vbat_scale_is_unity_at_reference_and_symmetric():
    vref, a, b = 22.2, -0.0776732, 2.7346374
    assert tool.vbat_scale(a, b, vref, vref) == pytest.approx(1.0, abs=1e-12)
    # mirrors quadratic_thrust_model.h vbatScale: (a V + b)/(a Vref + b)
    for v in (18.0, 22.2, 25.2):
        expected = (a * v + b) / (a * vref + b)
        assert tool.vbat_scale(a, b, vref, v) == pytest.approx(expected, abs=1e-12)


def test_normalize_throttle_degrades_without_compensation():
    # AirSim: kp=1, no voltage comp -> polynomial input equals the raw throttle.
    assert tool.normalize_throttle(0.3, 1.0, 0.0, 0.0, 25.2, False) == pytest.approx(0.3)
    # kp scales the force domain (baro); linear voltage comp scales by V/volt_max.
    assert tool.normalize_throttle(0.3, 0.8, 0.0, 0.0, 25.2, False) == pytest.approx(0.3 / 0.8)
    assert tool.normalize_throttle(0.3, 1.0, 0.0, 21.0, 25.2, True) == \
        pytest.approx(0.3 * 21.0 / 25.2)


def test_voltage_linear_fit_round_trip():
    a, b, vref = -0.08, 2.8, 22.2
    mean_f = 10.0
    points = []
    for v in (18.0, 20.0, 22.2, 24.0, 25.2):
        phi = (a * v + b) / (a * vref + b)
        # relative thrust y = mean(F)/F and F = mean_f / phi -> y = phi * (mean/mean)
        points.append({'voltage': v, 'force_n': mean_f / phi})
    fit, err = tool.fit_voltage_linear(points)
    assert err is None
    # fit returns (a, b); up to the mean(F) scaling the slope/intercept ratio holds
    fa, fb = fit
    assert fa / fb == pytest.approx(a / b, rel=1e-6)


def test_thrust_grid_includes_endpoints():
    args = type('A', (), {'min_thrust': 10.0, 'max_thrust': 40.0, 'step': 10.0})()
    assert tool.thrust_grid(args) == [10.0, 20.0, 30.0, 40.0]
    # a step that does not divide the span still terminates exactly at max_thrust
    args = type('A', (), {'min_thrust': 5.0, 'max_thrust': 20.0, 'step': 7.0})()
    grid = tool.thrust_grid(args)
    assert grid[0] == 5.0 and grid[-1] == 20.0
    assert all(grid[i] < grid[i + 1] for i in range(len(grid) - 1))
