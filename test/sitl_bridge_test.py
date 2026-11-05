import math
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from sitl_motor_calibration import fit_samples, read_samples, write_parameters
from sitl_support import VerticalEstimator, enu_flu, euler, projected_force
import propulsion_model as pm


ROOT = Path(__file__).resolve().parents[1]


def load_launch(monkeypatch, tmp_path, launch_file, yaml_text):
    """Load a launch file with Node captured and the package share redirected to tmp_path."""
    from launch_ros.utilities import normalize_parameters

    spec = importlib.util.spec_from_file_location(launch_file.replace(".", "_"), ROOT / "launch" / launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class CaptureNode:
        def __init__(self, **kwargs):
            self.parameters = normalize_parameters(kwargs["parameters"])

    (tmp_path / "parameters").mkdir()
    parameter_file = tmp_path / "parameters" / (
        "sitl_parameters.yaml" if launch_file == "sitl.launch.py" else "custom_link_parameters.yaml")
    parameter_file.write_text(yaml_text)
    monkeypatch.setattr(module, "Node", CaptureNode)
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(tmp_path))
    return module, parameter_file


def launch_context(module, **values):
    """LaunchContext seeded with every declared argument's default, then the given values.

    Seeding from the launch file itself keeps these tests valid when new launch
    arguments are added (unset arguments behave exactly as on the command line).
    """
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions

    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            default = entity.default_value
            context.launch_configurations[entity.name] = \
                "" if default is None else perform_substitutions(context, default)
    context.launch_configurations.update(values)
    return context


def launch_parameters(module, context):
    from launch_ros.utilities import evaluate_parameters
    return evaluate_parameters(context, module._launch_node(context)[0].parameters)


def declared_arguments(module):
    from launch.actions import DeclareLaunchArgument
    return {entity.name for entity in module.generate_launch_description().entities
            if isinstance(entity, DeclareLaunchArgument)}


def overrides(parameters, name):
    return [entry[name] for entry in parameters if isinstance(entry, dict) and name in entry]


@pytest.mark.parametrize("value,expected", [("", None), ("-20.0", -20.0), ("0.0", 0.0), ("15.0", 15.0)])
def test_shared_pitch_launch_preserves_yaml_unless_overridden(monkeypatch, tmp_path, value, expected):
    module, parameter_file = load_launch(
        monkeypatch, tmp_path, "custom_link.launch.py",
        "custom_link_bridge:\n  ros__parameters:\n    gimbal:\n      pitch_hold: -12.0\n")
    parameters = launch_parameters(module, launch_context(module, transport="tcp", pitch_hold=value))
    assert parameters[0] == parameter_file
    assert yaml.safe_load(parameter_file.read_text())["custom_link_bridge"]["ros__parameters"]["gimbal"]["pitch_hold"] == -12.0
    pitch = overrides(parameters, "gimbal.pitch_hold")
    assert pitch == ([] if expected is None else [expected])
    if expected is not None:
        assert isinstance(pitch[0], float)
    assert overrides(parameters, "link.transport") == ["tcp"]
    names = declared_arguments(module)
    assert "pitch_hold" in names and "fixed_pitch" not in names


@pytest.mark.parametrize("launch_file", ["custom_link.launch.py", "sitl.launch.py"])
@pytest.mark.parametrize("value,expected", [("", None), ("0.0", 0.0), ("-55.0", -55.0), ("10.0", 10.0)])
def test_aim_launch_preserves_yaml_unless_overridden(monkeypatch, tmp_path, launch_file, value, expected):
    module, parameter_file = load_launch(
        monkeypatch, tmp_path, launch_file,
        "custom_link_bridge:\n  ros__parameters:\n    gimbal:\n      pitch_aim: -20.0\n")
    parameters = launch_parameters(module, launch_context(module, pitch_aim=value,
                                                          parameters=str(parameter_file)))
    assert overrides(parameters, "gimbal.pitch_aim") == ([] if expected is None else [expected])
    assert yaml.safe_load(parameter_file.read_text())["custom_link_bridge"]["ros__parameters"]["gimbal"]["pitch_aim"] == -20.0
    assert "pitch_aim" in declared_arguments(module)


@pytest.mark.parametrize("value,expected", [("", None), ("true", True), ("false", False)])
def test_bind_rx_launch_override(monkeypatch, tmp_path, value, expected):
    module, _ = load_launch(
        monkeypatch, tmp_path, "custom_link.launch.py",
        "custom_link_bridge:\n  ros__parameters:\n    cli:\n      bind_rx_on_connect: true\n")
    parameters = launch_parameters(module, launch_context(module, bind_rx=value))
    bind = overrides(parameters, "cli.bind_rx_on_connect")
    assert bind == ([] if expected is None else [expected])
    if expected is not None:
        assert isinstance(bind[0], bool)
    # no other argument leaks an override when left at its default
    assert all(set(entry) == {"cli.bind_rx_on_connect"} for entry in parameters if isinstance(entry, dict))


def q(x=0.0, y=0.0, z=0.0, w=1.0):
    return SimpleNamespace(x_val=x, y_val=y, z_val=z, w_val=w)


def test_frames_and_specific_force():
    assert euler(enu_flu(q())) == pytest.approx((0, 0, math.pi / 2))
    assert euler(enu_flu(q(z=math.sqrt(.5), w=math.sqrt(.5)))) == pytest.approx((0, 0, 0))
    k = SimpleNamespace(orientation=q(), linear_acceleration=SimpleNamespace(x_val=0, y_val=0, z_val=-3))
    assert projected_force(k, 2.0, 9.80665) == pytest.approx(25.6133)
    # A tilted stationary body does not support weight with its full body-Z thrust.
    k.orientation = q(y=math.sin(math.pi / 6), w=math.cos(math.pi / 6))
    k.linear_acceleration.z_val = 0
    assert projected_force(k, 2.0, 9.80665) == pytest.approx(9.80665)


def test_default_parameters_and_model():
    config = yaml.safe_load((ROOT / "parameters/sitl_parameters.yaml").read_text())
    params = config["custom_link_bridge"]["ros__parameters"]
    assert params["mass"] == 2.0
    assert not params["voltage_compensation"] and not params["baro_compensation"]
    assert params["gimbal_port"] == "" and params["airsim"]["enabled"]
    motor = params["motor_parameters"]
    assert set(motor) == {"a3", "a2", "a1", "u_idle", "motor_idle", "r_eq", "k_i"}
    model = pm.PropulsionModel.from_parameters(motor, 6)
    assert model.validate() == ""
    assert model.collective(2 * 9.80665) == pytest.approx(0.1732, abs=.002)
    assert model.throttle(0) == model.u_idle and model.collective(0) == 0
    assert model.throttle(1000) == 1.0
    assert model.throttle(float("nan")) == model.u_idle


def duty_samples(force_of_duty, collective):
    return [(force_of_duty(pm.collective_to_duty(c)), c) for c in collective]


def test_fit_and_output_shape(tmp_path):
    collective = np.linspace(.08, .5, 30)
    samples = duty_samples(lambda u: 90.0 * u, collective)
    model, report = fit_samples(samples, 2.0, 9.80665, .01)
    assert (model.a3, model.a2, model.a1) == pytest.approx((0, 0, 90.0), abs=1e-6)
    assert report["throttle_max_error"] < 1e-9
    output = tmp_path / "fitted.yaml"
    write_parameters(ROOT / "parameters/sitl_parameters.yaml", output, model, 2.0, 9.80665)
    p = yaml.safe_load(output.read_text())["custom_link_bridge"]["ros__parameters"]
    assert p["motor_parameters"] == {"a3": model.a3, "a2": model.a2, "a1": model.a1, "u_idle": .05,
                                     "motor_idle": .055, "r_eq": 0.0, "k_i": 0.0}
    assert p["airsim"]["enabled"] and p["mass"] == 2.0
    assert not p["voltage_compensation"] and not p["baro_compensation"]
    for bad in ([(float("nan"), .1)] * 5, [(20, .1)] * 5, [(20, -.1)] * 5):
        with pytest.raises(ValueError):
            fit_samples(bad, 2.0, 9.80665, .01)


def test_cubic_fit_and_residual_rejection():
    collective = np.linspace(.08, .5, 40)
    truth = (-15.0, 70.0, 30.0)
    samples = duty_samples(lambda u: (truth[0] * u + truth[1]) * u * u + truth[2] * u, collective)
    model, report = fit_samples(samples, 2, 9.80665, .01)
    assert (model.a3, model.a2, model.a1) == pytest.approx(truth, rel=1e-9)
    assert report["throttle_max_error"] < 1e-9
    samples[10] = (samples[10][0] * 1.4, samples[10][1])
    with pytest.raises(ValueError, match="residual"):
        fit_samples(samples, 2, 9.80665, .01)
    falling = duty_samples(lambda u: 60 * u - 40 * u * u * u, np.linspace(.08, 1.0, 30))
    with pytest.raises(ValueError, match="dF/du"):
        fit_samples(falling, 2, 9.80665, .01)


def test_vertical_estimator_filters_stationary_baro_noise():
    estimator = VerticalEstimator()
    estimator.update(123.4)
    noise = np.random.default_rng(1234)
    for index in range(1000):
        estimator.predict(0.0, .005)
        if index % 2 == 0:
            estimator.update(123.4 + noise.normal(0, .35))
    assert abs(estimator.height - 123.4) < .15
    assert abs(estimator.velocity) < .3
    with pytest.raises(ValueError):
        estimator.predict(float("nan"), .005)
    with pytest.raises(ValueError):
        estimator.update(float("nan"))


def test_vertical_estimator_tracks_accelerating_flight():
    estimator = VerticalEstimator()
    estimator.update(100.0)
    noise = np.random.default_rng(5678)
    for index in range(1, 1001):
        t = index * .005
        estimator.predict(1.0, .005)
        if index % 2 == 0:
            estimator.update(100 + .5 * t * t + noise.normal(0, .35))
    assert estimator.height == pytest.approx(112.5, abs=.2)
    assert estimator.velocity == pytest.approx(5.0, abs=.2)


def test_measured_motor_calibration_matches_parameters():
    samples = read_samples(ROOT / "parameters/sitl_motor_calibration.csv")
    model, report = fit_samples(samples, 2.0, 9.80665, .01)
    params = yaml.safe_load((ROOT / "parameters/sitl_parameters.yaml").read_text())
    configured = params["custom_link_bridge"]["ros__parameters"]["motor_parameters"]
    assert len(samples) == 42
    for key in ("a3", "a2", "a1"):
        assert configured[key] == pytest.approx(getattr(model, key), rel=1e-10, abs=1e-12)
    assert report["throttle_max_error"] < .002
    assert report["r_squared"] > .9999
