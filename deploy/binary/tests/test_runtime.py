#!/usr/bin/env python3
"""Focused packaging and two-process service lifecycle regressions.

Run with: python3 -B -m unittest discover -s deploy/binary/tests -v
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch


BINARY = Path(__file__).resolve().parents[1]
DEPLOY = BINARY.parent
spec = importlib.util.spec_from_file_location("collect_runtime", BINARY / "collect_runtime.py")
collector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(collector)


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


def arm_elf(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    header = bytearray(64)
    header[:6] = b"\x7fELF\x02\x01"
    struct.pack_into("<H", header, 18, 183)
    path.write_bytes(header)
    return path


class PackagingTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.app, self.messages = self.root / "app", self.root / "messages"
        arm_elf(self.app / "lib/apm_bridge/custom_link_bridge_node")
        for name in ["bench_pid_tuner", "bench_pid_core.py", "sitl_support.py"]:
            write(self.app / "lib/apm_bridge" / name, f"# Installed {name}\n")
        write(self.app / "share/apm_bridge/parameters/custom_link_parameters.yaml", "bridge: {}\n")
        write(self.app / "share/apm_bridge/parameters/bench_pid_README.md", "# Bench tuner\n")
        write(self.app / "share/apm_bridge/parameters/bench_pid.yaml",
              "bench_pid_tuner:\n  ros__parameters:\n    enabled: true\n    rig_confirmed: true\n")
        arm_elf(self.messages / "lib/libquadrotor_msgs__rosidl_typesupport_c.so")
        self.python = self.messages / collector.PYTHON_RELATIVE / "quadrotor_msgs"
        self.extension = arm_elf(self.python / "quadrotor_msgs_s__rosidl_typesupport_c.so")
        for name in ["msg/_control_command.py", "msg/_low_level_feedback.py", "srv/_cli_command.py"]:
            write(self.python / name, "# Generated interface\n")
        for prefix, name in [(self.app, "apm_bridge"), (self.messages, "quadrotor_msgs")]:
            write(prefix / "share" / name / "package.xml", "<package/>\n")
            write(prefix / "share/ament_index/resource_index/packages" / name, "")
            write(prefix / "share/ament_index/resource_index/parent_prefix_path" / name,
                  "/opt/arm64-sysroot/opt/ros/humble\n")
        write(self.messages / "share/quadrotor_msgs/msg/ControlCommand.msg", "float32 value\n")
        write(self.messages / "share/quadrotor_msgs/srv/CliCommand.srv", "---\n")

    def collect(self, enabled=False):
        args = argparse.Namespace(app_prefix=self.app, messages_prefix=self.messages,
                                  templates=BINARY, enable_bench_pid=enabled)
        package = self.root / ("enabled" if enabled else "default")
        with patch.object(collector.platform, "machine", return_value="x86_64"):
            collector.collect(args, package)
        return package

    def test_default_disables_both_even_if_install_yaml_enabled(self):
        package = self.collect()
        config = (package / "config/bench_pid.yaml").read_text()
        self.assertIn("enabled: false", config)
        self.assertIn("rig_confirmed: false", config)
        self.assertIn("BENCH_PID_ENABLED=false", (package / "config/runtime.env").read_text())
        for relative in ["supervise.sh", "runtime/libexec/apm_bridge/bench_pid_tuner",
                         "runtime/libexec/apm_bridge/bench_pid_core.py",
                         "runtime/libexec/apm_bridge/sitl_support.py",
                         "runtime/share/quadrotor_msgs/srv/CliCommand.srv",
                         "runtime/share/ament_index/resource_index/packages/quadrotor_msgs"]:
            self.assertTrue((package / relative).is_file(), relative)
        self.assertTrue((package / "runtime" / collector.PYTHON_RELATIVE / "quadrotor_msgs"
                         / self.extension.name).is_file())
        self.assertFalse((package / "runtime/share/ament_index/resource_index/parent_prefix_path").exists())

    def test_enable_sets_both(self):
        package = self.collect(enabled=True)
        config = (package / "config/bench_pid.yaml").read_text()
        self.assertIn("enabled: true", config)
        self.assertIn("rig_confirmed: true", config)
        self.assertIn("BENCH_PID_ENABLED=true", (package / "config/runtime.env").read_text())

    def test_rejects_mixed_python_extension_architecture(self):
        header = bytearray(self.extension.read_bytes())
        struct.pack_into("<H", header, 18, 62)
        self.extension.write_bytes(header)
        with self.assertRaisesRegex(RuntimeError, "ARM64 ELF"):
            self.collect()

    def test_rejects_missing_generated_service(self):
        (self.python / "srv/_cli_command.py").unlink()
        with self.assertRaisesRegex(RuntimeError, "Missing generated Python interface"):
            self.collect()


class SupervisorTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        taskset = write(self.root / "taskset", '#!/bin/bash\nshift 2\nexec "$@"\n')
        taskset.chmod(0o755)
        worker = write(self.root / "worker.py", '''#!/usr/bin/python3
import json, os, signal, sys, time
from pathlib import Path
root = Path(os.environ["TEST_OUTPUT"])
name = "tuner" if "__node:=bench_pid_tuner" in sys.argv else "bridge"
(root / (name + ".json")).write_text(json.dumps({"pid": os.getpid(), "args": sys.argv}))
def stop(*args):
    (root / (name + ".stopped")).touch()
    sys.exit(0)
signal.signal(signal.SIGTERM, stop)
if os.environ.get("TEST_STUBBORN") == name:
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
while True: time.sleep(.02)
''')
        worker.chmod(0o755)
        self.env = dict(os.environ, PATH=f"{self.root}:{os.environ['PATH']}", TEST_OUTPUT=str(self.root),
                        APM_BRIDGE_EXECUTABLE=str(worker), APM_BENCH_EXECUTABLE=str(worker),
                        APM_CONFIG=str(write(self.root / "bridge.yaml", "{}\n")),
                        APM_BENCH_CONFIG=str(write(self.root / "bench.yaml", "{}\n")))
        self.env.pop("BENCH_PID_ENABLED", None)

    def start(self, *args):
        process = subprocess.Popen(["bash", str(DEPLOY / "supervise.sh"), *args], env=self.env,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        self.addCleanup(lambda: process.poll() is None and process.kill())
        deadline = time.monotonic() + 5
        while not all((self.root / (node + ".json")).exists() for node in ["bridge", "tuner"]):
            if process.poll() is not None:
                self.fail(process.stderr.read())
            if time.monotonic() > deadline:
                self.fail("Both child nodes did not start")
            time.sleep(.01)
        return process

    def info(self, node):
        return json.loads((self.root / (node + ".json")).read_text())

    def test_default_and_signal_cleanup(self):
        process = self.start()
        args = self.info("tuner")["args"]
        self.assertIn("enabled:=false", args)
        self.assertIn("rig_confirmed:=false", args)
        for topic in ["control_command", "control_command_raw", "arm", "offboard", "low_level_feedback"]:
            self.assertIn(f"~/{topic}:=/fpv/{topic}", self.info("bridge")["args"])
        process.send_signal(signal.SIGTERM)
        self.assertEqual(process.wait(timeout=7), 0)
        process.stderr.close()
        for node in ["bridge", "tuner"]:
            self.assertTrue((self.root / (node + ".stopped")).exists())

    def test_enable_and_child_exit_cleanup(self):
        process = self.start("--enable-bench-pid")
        args = self.info("tuner")["args"]
        self.assertIn("enabled:=true", args)
        self.assertIn("rig_confirmed:=true", args)
        os.kill(self.info("bridge")["pid"], signal.SIGTERM)
        self.assertNotEqual(process.wait(timeout=7), 0)
        process.stderr.close()
        self.assertTrue((self.root / "tuner.stopped").exists())

    def test_disable_overrides_enabled_environment(self):
        self.env["BENCH_PID_ENABLED"] = "true"
        process = self.start("--disable-bench-pid")
        self.assertIn("enabled:=false", self.info("tuner")["args"])
        self.assertIn("rig_confirmed:=false", self.info("tuner")["args"])
        process.terminate()
        self.assertEqual(process.wait(timeout=7), 0)
        process.stderr.close()

    def test_unresponsive_companion_is_killed(self):
        self.env["TEST_STUBBORN"] = "tuner"
        process = self.start()
        tuner = self.info("tuner")["pid"]
        os.kill(self.info("bridge")["pid"], signal.SIGTERM)
        self.assertNotEqual(process.wait(timeout=8), 0)
        process.stderr.close()
        with self.assertRaises(ProcessLookupError):
            os.kill(tuner, 0)


if __name__ == "__main__":
    unittest.main()
