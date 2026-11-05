import math

import pytest

from hw_control_command_test import ThrustModel


PARAMS = dict(A=2.26453885e-6, B=-0.00026519864, C=0.0179075134,
              D=0.0119440187, n=1, thrust_limit=0.9)


@pytest.mark.parametrize("voltage_comp", [False, True])
@pytest.mark.parametrize("baro_comp", [False, True])
@pytest.mark.parametrize("voltage", [0.0, 22.2, 25.2])
@pytest.mark.parametrize("vbat_a", [0.0, -0.0776732])
@pytest.mark.parametrize("kp", [0.8, 1.0, 1.2])
def test_hardware_mirror_uses_force_domain_density(voltage_comp, baro_comp, voltage, vbat_a, kp):
    model = ThrustModel(dict(PARAMS, vbat_a=vbat_a, vbat_b=2.7346374, spin_k=0.3),
                        6, voltage_comp, baro_comp)
    force = 10.0
    x = force / kp if baro_comp else force
    if voltage_comp and voltage > 0 and vbat_a != 0:
        x *= (vbat_a * voltage + 2.7346374) / (vbat_a * 22.2 + 2.7346374)
    expected = ((PARAMS["A"] * x + PARAMS["B"]) * x + PARAMS["C"]) * x + PARAMS["D"]
    if voltage_comp and voltage > 0 and vbat_a == 0:
        expected *= 25.2 / voltage
    expected = 0.7 * expected + 0.3 * expected * expected
    assert model.force_to_throttle(force, kp, voltage) == pytest.approx(expected, abs=1e-12)
    assert model.force_for_throttle(expected, kp, voltage) == pytest.approx(force, abs=1e-9)


@pytest.mark.parametrize("kp", [0, -1, math.inf, math.nan])
def test_hardware_mirror_rejects_invalid_density(kp):
    model = ThrustModel(PARAMS, 6)
    with pytest.raises(ValueError, match="Air density"):
        model.force_to_throttle(10.0, kp, 22.2)


@pytest.mark.parametrize("vbat_a", [0.0, -0.0776732])
def test_hardware_mirror_voltage_floor_and_force_clamp(vbat_a):
    # Mirrors quadratic_thrust_model.h: volt_min = cells*3.0 floor, no negative force.
    model = ThrustModel(dict(PARAMS, vbat_a=vbat_a, vbat_b=2.7346374), 6, True, False)
    assert model.force_to_throttle(10.0, 1.0, 2.0) == model.force_to_throttle(10.0, 1.0, 18.0)
    assert model.force_to_throttle(10.0, 1.0, math.nan) == model.force_to_throttle(10.0, 1.0, 0.0)
    assert model.force_to_throttle(-5.0, 1.0, 22.2) == model.force_to_throttle(0.0, 1.0, 22.2)
    assert model.force_to_throttle(math.nan, 1.0, 22.2) == model.force_to_throttle(0.0, 1.0, 22.2)
