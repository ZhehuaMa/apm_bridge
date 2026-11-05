#!/usr/bin/env python3
"""Collect AirSim throttle/acceleration pulses or fit an existing CSV to the fpv_test.md model.

The model is F_std(u) = a3*u^3 + a2*u^2 + a1*u (total thrust, N) over the DShot duty
u = u_idle + (1 - u_idle) * collective, fitted by the zero-intercept normal
equations of fpv_test.md section 3.2 (scripts/propulsion_model.py). AirSim has no
barometer, battery sag or RPM feedback, so the environment/voltage steps are skipped
and both node compensations stay disabled.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import time

import numpy as np
import yaml

import propulsion_model as pm
from sitl_support import VerticalEstimator, conjugate, euler, multiply


def fit_samples(samples, mass: float, gravity: float, max_error: float, u_idle: float):
    """(force_N, collective) samples -> (PropulsionModel, report).

    u_idle is the FC motor_idle duty at collection time (collective 0 = duty u_idle).
    """
    if not math.isfinite(mass) or mass <= 0 or not math.isfinite(gravity) or gravity <= 0:
        raise ValueError("mass and gravity must be finite and positive")
    values = np.asarray(samples, dtype=float)
    if values.ndim != 2 or values.shape[1] != 2 or len(values) < 4:
        raise ValueError("too few (force_N, throttle) samples")
    force, collective = values[:, 0], values[:, 1]
    if not np.isfinite(values).all() or np.any(force <= 0) or \
            np.any(collective < 0) or np.any(collective > 1):
        raise ValueError("samples must be finite, airborne and inside the throttle domain")
    if len(np.unique(collective)) < 3 or np.ptp(force) < mass * gravity * 0.2:
        raise ValueError("insufficient throttle levels or force range")
    duty = np.array([pm.collective_to_duty(c, u_idle) for c in collective])
    (a3, a2, a1), stats = pm.fit_zero_intercept_cubic(duty, force)
    monotonic, min_slope = pm.monotonic_check(a3, a2, a1)
    if not monotonic:
        raise ValueError(f"fitted F(u) violates dF/du > 0 on [0,1] (min slope {min_slope:.4g})")
    model = pm.PropulsionModel(a3=a3, a2=a2, a1=a1, u_idle=u_idle)
    # Residual in throttle (duty) units: model inversion of each measured force.
    error = np.array([model.solve_standard_throttle(f) for f in force]) - duty
    rmse, peak = float(np.sqrt(np.mean(error ** 2))), float(np.max(np.abs(error)))
    if peak > max_error:
        raise ValueError(f"fit residual {peak:.6f} exceeds {max_error:.6f}; inspect transient/drag samples")
    hover = model.throttle(mass * gravity)
    if not u_idle < hover < 1.0:
        raise ValueError("fitted hover throttle lies outside the usable domain")
    report = {"samples": len(values), "mass_kg": mass, "gravity_m_s2": gravity,
              "force_range_N": [float(force.min()), float(force.max())],
              "duty_range": [float(duty.min()), float(duty.max())],
              "throttle_rmse": rmse, "throttle_max_error": peak,
              "force_rms_N": stats["rms"], "r_squared": stats["r_squared"],
              "min_slope_N": min_slope, "hover_duty": hover,
              "hover_collective": pm.duty_to_collective(hover, u_idle), "u_idle": u_idle}
    return model, report


def read_samples(path: Path):
    """CSV -> ((force_N, collective) samples, u_idle at collection time).

    CSVs written before u_idle was recorded fall back to the Betaflight motor_idle
    default (550 = 0.055), which is what the shipped SITL measurement used.
    """
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    samples = [(float(row["force_N"]), float(row["throttle"])) for row in rows]
    idles = {float(row["u_idle"]) for row in rows if row.get("u_idle")}
    if len(idles) > 1:
        raise ValueError(f"inconsistent u_idle in {path}: {sorted(idles)}")
    return samples, (idles.pop() if idles else pm.DEFAULT_U_IDLE)


def model_parameters(model: pm.PropulsionModel):
    return {"a3": model.a3, "a2": model.a2, "a1": model.a1, "r_eq": 0.0, "k_i": 0.0}


def write_parameters(template: Path, output: Path, model: pm.PropulsionModel, mass: float, gravity: float):
    with template.open() as stream:
        config = yaml.safe_load(stream)
    params = config["custom_link_bridge"]["ros__parameters"]
    params.update(mass=mass, gravity_const=gravity, voltage_compensation=False, baro_compensation=False)
    params["motor_parameters"] = model_parameters(model)
    with output.open("w") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)


def collect(args, config):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter_client import AsyncParameterClient
    from mavros_msgs.msg import State
    from quadrotor_msgs.msg import ControlCommand
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Bool, Float64

    class Observer(Node):
        def __init__(self):
            super().__init__("sitl_motor_calibration")
            self.state, self.imu = None, None
            self.state_time, self.imu_time, self.altitude_time = 0.0, 0.0, 0.0
            self.altitude, self.velocity = 0.0, 0.0
            self.estimator = VerticalEstimator()
            self.imu_stamp = None
            self.warmup_start = None
            self.create_subscription(State, "/mavros/state", self.on_state, 10)
            self.create_subscription(Imu, "/mavros/imu/data", self.on_imu, qos_profile_sensor_data)
            self.create_subscription(Float64, "/fpv/altitude", self.on_altitude, qos_profile_sensor_data)
            self.arm_pub = self.create_publisher(Bool, "/fpv/arm", 10)
            self.command_pub = self.create_publisher(ControlCommand, "/fpv/control_command_raw",
                                                     qos_profile_sensor_data)

        def on_state(self, msg):
            self.state, self.state_time = msg, time.monotonic()

        def on_imu(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.estimator.initialized and self.imu_stamp is not None:
                dt = stamp - self.imu_stamp
                if 0 < dt <= 0.2:
                    q = msg.orientation
                    a = msg.linear_acceleration
                    rotation = (q.x, q.y, q.z, q.w)
                    world = multiply(multiply(rotation, (a.x, a.y, a.z, 0.0)), conjugate(rotation))
                    self.estimator.predict(world[2] - args.gravity, dt)
                    self.altitude, self.velocity = self.estimator.height, self.estimator.velocity
                else:
                    self.get_logger().warning("Skipped discontinuous FC IMU timestamp",
                                              throttle_duration_sec=2.0)
            self.imu_stamp = stamp
            self.imu, self.imu_time = msg, time.monotonic()

        def on_altitude(self, msg):
            now = time.monotonic()
            self.estimator.update(msg.data)
            if self.warmup_start is None:
                self.warmup_start = now
            self.altitude, self.velocity = self.estimator.height, self.estimator.velocity
            self.altitude_time = now

    rclpy.init()
    node = Observer()
    owned, airborne = False, False
    csv_stream = None
    origin = None
    bootstrap = pm.PropulsionModel.from_parameters(config["motor_parameters"], config.get("lipo_cells", 6),
                                                   pm.DEFAULT_U_IDLE)
    model_error = bootstrap.validate()
    if model_error:
        raise RuntimeError(f"SITL parameters need a valid model to hover: {model_error}")
    idle = rotor_hover = None  # set from the FC motor_idle before the first drive()
    samples = []

    def command(rates, throttle):
        msg = ControlCommand(armed=True, control_mode=ControlCommand.BODY_RATES,
                             collective_thrust=float(throttle))
        msg.bodyrates.x, msg.bodyrates.y, msg.bodyrates.z = rates
        node.command_pub.publish(msg)

    def pump():
        until = time.monotonic() + 0.02
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.002)

    def healthy():
        now = time.monotonic()
        if node.state is None or not node.state.connected or now - node.state_time > 0.5 or \
                node.imu is None or now - node.imu_time > 0.2 or now - node.altitude_time > 0.2:
            raise RuntimeError("fresh custom-link state/IMU/altitude telemetry required")

    def drive(target_height, duration, pulse=None, record=False):
        nonlocal airborne
        start = time.monotonic()
        while time.monotonic() - start < duration:
            healthy()
            if not node.state.armed or node.state.mode != "OFFBOARD":
                raise RuntimeError("firmware left armed OFFBOARD during calibration")
            q = node.imu.orientation
            roll, pitch, yaw = euler((q.x, q.y, q.z, q.w))
            height = node.altitude - origin
            airborne = airborne or height > 0.5
            if height > 8 or abs(node.velocity) > 5 or \
                    max(abs(roll), abs(pitch)) > math.radians(20):
                raise RuntimeError(f"calibration flight envelope exceeded: height={height:.2f} m, "
                                   f"vz={node.velocity:.2f} m/s, "
                                   f"roll/pitch={math.degrees(roll):.1f}/{math.degrees(pitch):.1f} deg")
            rates = (max(-0.5, min(0.5, -4 * roll)),
                     max(-0.5, min(0.5, -4 * pitch)), 0.0)
            tilt = max(0.7, math.cos(roll) * math.cos(pitch))
            throttle = (rotor_hover / tilt - idle) / (1 - idle) + \
                rotor_hover / 0.587 * (0.05 * (target_height - height) - 0.10 * node.velocity)
            if pulse is not None:
                throttle = pulse
            command(rates, max(0.0, min(args.thrust_limit, throttle)))
            elapsed = time.monotonic() - start
            if record and elapsed >= args.settle_time:
                if height < 1.0 or max(abs(roll), abs(pitch)) > math.radians(5):
                    raise RuntimeError("contact/tilt would invalidate thrust samples")
                a = node.imu.linear_acceleration
                force = args.mass * a.z
                stamp = node.imu.header.stamp
                writer.writerow((time.time(), pulse, force, height,
                                 a.x, a.y, a.z, q.x, q.y, q.z, q.w,
                                 stamp.sec + stamp.nanosec * 1e-9, idle))
                samples.append((force, pulse))
            pump()
        return height

    try:
        parameters = AsyncParameterClient(node, "custom_link_bridge")
        if not parameters.wait_for_services(timeout_sec=3):
            raise RuntimeError("custom_link_bridge parameter service unavailable")
        future = parameters.get_parameters(["airsim.enabled", "mass", "gravity_const"])
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not future.done():
            pump()
        if not future.done() or future.result() is None:
            raise RuntimeError("cannot verify that the bridge is running SITL")
        values = future.result().values
        if len(values) != 3 or not values[0].bool_value:
            raise RuntimeError("motor experiment is permitted only with airsim.enabled=true")
        if not math.isclose(values[1].double_value, args.mass) or \
                not math.isclose(values[2].double_value, args.gravity):
            raise RuntimeError("experiment mass/gravity must match the running bridge")
        idle = pm.fetch_u_idle(node, timeout=5.0, spin=True)  # FC motor_idle = duty at collective 0
        bootstrap.set_u_idle(idle)
        rotor_hover = bootstrap.throttle(args.mass * args.gravity)  # hover DShot duty
        print(f"FC motor_idle -> u_idle = {idle:.4f}, hover duty = {rotor_hover:.4f}", flush=True)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and (
                node.state is None or node.imu is None or node.warmup_start is None or
                time.monotonic() - node.warmup_start < 1.0):
            pump()
        healthy()
        if node.count_publishers("/fpv/control_command_raw") > 1 or \
                node.count_publishers("/fpv/control_command") > 0:
            raise RuntimeError("stop other control publishers before calibration")
        if node.state.armed:
            raise RuntimeError("calibration requires a disarmed SITL vehicle")
        origin = node.altitude
        node.arm_pub.publish(Bool(data=True))
        owned = True
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not (node.state.armed and node.state.mode == "OFFBOARD"):
            command((0.0, 0.0, 0.0), 0)
            pump()
        if not node.state.armed or node.state.mode != "OFFBOARD":
            raise RuntimeError("custom-link arming/OFFBOARD refused; inspect SITL console")
        height = drive(4.0, 10.0)
        if abs(height - 4.0) > 0.6 or abs(node.velocity) > 0.5:
            raise RuntimeError("calibration hover did not settle")
        csv_stream = args.collect.open("x", newline="")
        writer = csv.writer(csv_stream)
        writer.writerow(("time_s", "throttle", "force_N", "height_m",
                         "ax_flu", "ay_flu", "az_flu", "qx", "qy", "qz", "qw", "fc_stamp_s", "u_idle"))
        for repeat in range(args.repeats):
            for level in args.levels:
                drive(4.0, 3.0)
                drive(4.0, args.pulse_time, pulse=level, record=True)
                csv_stream.flush()
                print(f"pulse {repeat + 1}: throttle={level:.3f}, samples={len(samples)}", flush=True)
        return samples, idle
    finally:
        if csv_stream is not None:
            csv_stream.close()
        try:
            if owned:
                if airborne:
                    height = drive(-1.0, 12.0)
                    if height > 0.3 or abs(node.velocity) > 0.3:
                        raise RuntimeError("calibration landing did not settle")
        finally:
            try:
                if owned:
                    node.arm_pub.publish(Bool(data=False))
                    deadline = time.monotonic() + 3
                    while time.monotonic() < deadline and node.state.armed:
                        pump()
                    if node.state.armed:
                        raise RuntimeError("custom-link disarm was not confirmed after calibration")
            finally:
                node.destroy_node()
                rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--collect", type=Path, help="new CSV; requires an idle running AirSim/SITL")
    mode.add_argument("--fit", type=Path, help="fit a previously collected CSV without flying")
    parser.add_argument("--parameters", type=Path, required=True, help="SITL parameter template")
    parser.add_argument("--output", type=Path, required=True, help="standalone fitted ROS parameters")
    parser.add_argument("--mass", type=float, default=2.0)
    parser.add_argument("--gravity", type=float, default=9.80665)
    parser.add_argument("--thrust-limit", type=float, default=0.5,
                        help="maximum Betaflight collective commanded during the experiment")
    parser.add_argument("--max-error", type=float, default=0.01,
                        help="maximum inverse-model residual in throttle (duty) units")
    parser.add_argument("--levels", type=float, nargs="+", default=[0.08, 0.12, 0.17, 0.22, 0.3, 0.4, 0.5])
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--pulse-time", type=float, default=0.3)
    parser.add_argument("--settle-time", type=float, default=0.24,
                        help="discard the measured FC acceleration-filter/transport transient")
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.mass, args.gravity, args.pulse_time,
                                          args.settle_time, args.thrust_limit, args.max_error)) or \
            args.mass <= 0 or args.gravity <= 0:
        parser.error("mass/gravity must be positive; numeric parameters must be finite")
    if args.repeats < 1 or not 0 <= args.settle_time < args.pulse_time or \
            args.pulse_time > 0.3 or any(not 0 < v <= args.thrust_limit for v in args.levels):
        parser.error("invalid repeats, pulse duration or throttle levels")
    if not 0 < args.thrust_limit <= 1 or args.max_error <= 0:
        parser.error("invalid thrust limit or residual threshold")
    with args.parameters.open() as stream:
        config = yaml.safe_load(stream)["custom_link_bridge"]["ros__parameters"]
    samples, u_idle = collect(args, config) if args.collect else read_samples(args.fit)
    model, report = fit_samples(samples, args.mass, args.gravity, args.max_error, u_idle)
    write_parameters(args.parameters, args.output, model, args.mass, args.gravity)
    print(json.dumps({"motor_parameters": model_parameters(model), **report}, indent=2))


if __name__ == "__main__":
    main()
