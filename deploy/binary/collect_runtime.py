#!/usr/bin/env python3
"""Package an installed ARM64 bridge, bench tuner and custom ROS interfaces.

Cross-packaging checks ELF architecture without running target binaries. The
installer checks library resolution and Python imports on the target board.
"""
import argparse
from datetime import date
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import struct
import subprocess
import tarfile


PYTHON_RELATIVE = Path("local/lib/python3.10/dist-packages")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_aarch64(path):
    """Reject wrong architectures, including accidentally bundled Python extensions."""
    header = path.read_bytes()[:20]
    if (len(header) < 20 or header[:6] != b"\x7fELF\x02\x01"
            or struct.unpack("<H", header[18:20])[0] != 183):
        raise RuntimeError(f"Expected a little-endian ARM64 ELF file: {path}")


def copy_tree(source, destination):
    if not source.is_dir():
        raise RuntimeError(f"Missing installed runtime directory: {source}")
    shutil.copytree(source, destination, symlinks=False,
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))


def collect(args, package):
    app, messages = args.app_prefix, args.messages_prefix
    for directory in ["bin", "runtime/lib", "runtime/libexec/apm_bridge", "config", "licenses"]:
        (package / directory).mkdir(parents=True, exist_ok=True)
    executable = app / "lib/apm_bridge/custom_link_bridge_node"
    shutil.copy2(executable, package / "bin/custom_link_bridge_node", follow_symlinks=True)
    for name in ["bench_pid_tuner", "bench_pid_core.py", "sitl_support.py"]:
        shutil.copy2(app / "lib/apm_bridge" / name,
                     package / "runtime/libexec/apm_bridge" / name, follow_symlinks=True)
    for name in ["install.sh", "run.sh", "performance.sh", "apm-bridge.service", "README.md"]:
        shutil.copy2(args.templates / name, package / name)
    shutil.copy2(args.templates.parent / "supervise.sh", package / "supervise.sh")
    enabled = str(args.enable_bench_pid).lower()
    environment = (args.templates / "runtime.env").read_text()
    environment, count = re.subn(r"^BENCH_PID_ENABLED=.*$", "BENCH_PID_ENABLED=" + enabled,
                                 environment, flags=re.MULTILINE)
    if count != 1:
        raise RuntimeError("runtime.env must define BENCH_PID_ENABLED exactly once")
    (package / "config/runtime.env").write_text(environment)
    shutil.copy2(app / "share/apm_bridge/parameters/custom_link_parameters.yaml",
                 package / "config/parameters.yaml", follow_symlinks=True)
    shutil.copy2(app / "share/apm_bridge/parameters/bench_pid_README.md",
                 package / "config/bench_pid_README.md", follow_symlinks=True)
    bench = (app / "share/apm_bridge/parameters/bench_pid.yaml").read_text()
    for flag in ["enabled", "rig_confirmed"]:
        bench, count = re.subn(rf"^(\s+{flag}:)\s*(true|false)\s*$", rf"\g<1> {enabled}",
                               bench, flags=re.MULTILINE)
        if count != 1:
            raise RuntimeError(f"bench_pid.yaml must define {flag} exactly once")
    (package / "config/bench_pid.yaml").write_text(bench)
    libraries = sorted((messages / "lib").glob("libquadrotor_msgs*.so*"))
    if not libraries:
        raise RuntimeError("Missing compiled quadrotor_msgs libraries")
    for library in libraries:
        shutil.copy2(library, package / "runtime/lib" / library.name, follow_symlinks=True)
    copy_tree(messages / PYTHON_RELATIVE / "quadrotor_msgs",
              package / "runtime" / PYTHON_RELATIVE / "quadrotor_msgs")
    python_messages = package / "runtime" / PYTHON_RELATIVE / "quadrotor_msgs"
    for relative in ["msg/_control_command.py", "msg/_low_level_feedback.py", "srv/_cli_command.py"]:
        if not (python_messages / relative).is_file():
            raise RuntimeError(f"Missing generated Python interface: {relative}")
    if not list(python_messages.glob("*.so")):
        raise RuntimeError("Missing compiled quadrotor_msgs Python type support")
    for prefix, name in [(app, "apm_bridge"), (messages, "quadrotor_msgs")]:
        dest = package / "runtime/share" / name
        dest.mkdir(parents=True, exist_ok=True)
        shutil.copy2(prefix / "share" / name / "package.xml", dest / "package.xml")
        if name == "quadrotor_msgs":
            for interface in ["msg", "srv"]:
                copy_tree(prefix / "share" / name / interface, dest / interface)
        index = prefix / "share/ament_index/resource_index"
        resources = list(index.glob("*/" + name))
        if not any(resource.parent.name == "packages" for resource in resources):
            raise RuntimeError(f"Missing ament package index: {name}")
        for resource in resources:
            # Parent prefixes describe the build machine and must not escape into the bundle.
            if resource.parent.name == "parent_prefix_path":
                continue
            target = package / "runtime/share/ament_index/resource_index" / resource.parent.name / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(resource, target)
    elfs = [package / "bin/custom_link_bridge_node"]
    elfs += sorted((package / "runtime").rglob("*.so*"))
    for elf in elfs:
        check_aarch64(elf)
    check = "ELF architecture; target install.sh --check must validate dependencies"
    if platform.machine() == "aarch64":
        env = dict(os.environ, LC_ALL="C", LD_LIBRARY_PATH=":".join([
            str(package / "runtime/lib"), "/opt/ros/humble/lib",
            "/opt/ros/humble/lib/aarch64-linux-gnu"]))
        for elf in elfs:
            result = subprocess.check_output(["ldd", str(elf)], env=env, text=True)
            if "not found" in result:
                raise RuntimeError(f"Missing dependency: {elf}\n{result}")
        check = "ELF architecture and native ARM64 ldd; target Python preflight still required"
    return libraries, check


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--templates", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--app-prefix", type=Path, required=True,
                        help="Fresh target apm_bridge install prefix")
    parser.add_argument("--messages-prefix", type=Path, required=True,
                        help="Matching target quadrotor_msgs install prefix")
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--enable-bench-pid", action="store_true",
                        help="Set enabled AND rig_confirmed true; defaults are false")
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    # Stage independently: a failed collection cannot leave a misleading release.
    import tempfile
    with tempfile.TemporaryDirectory(prefix=".apm-stage-", dir=args.output) as staging:
        package = Path(staging) / "package"
        libraries, dependency_check = collect(args, package)
        identity = hashlib.sha256(args.source_commit.encode())
        for path in sorted(package.rglob("*")):
            if path.is_file():
                identity.update(str(path.relative_to(package)).encode())
                identity.update(bytes.fromhex(digest(path)))
        mode = "enabled" if args.enable_bench_pid else "disabled"
        version = f"{date.today():%Y%m%d}-{identity.hexdigest()[:16]}-cpu0-3-performance-two-nodes-{mode}"
        destination = args.output / ("apm_bridge-ubuntu22.04-arm64-" + version)
        archive = Path(str(destination) + ".tar")
        if destination.exists() or archive.exists():
            raise RuntimeError(f"Refusing to replace existing release: {destination}")
        (package / "VERSION").write_text(version + "\n")
        metadata = {
            "release": version, "architecture": "aarch64", "os": "Ubuntu 22.04", "glibc": "2.35",
            "source_commit": args.source_commit,
            "binary_sha256": digest(package / "bin/custom_link_bridge_node"),
            "config_sha256": digest(package / "config/parameters.yaml"),
            "bench_pid_tuner": {"enabled": args.enable_bench_pid,
                                "rig_confirmed": args.enable_bench_pid},
            "nodes": ["custom_link_bridge_node", "bench_pid_tuner"],
            "dependency_check": dependency_check,
            "ros_runtime_included": False, "required_ros_prefix": "/opt/ros/humble",
            "required_python": "3.10 with rclpy, NumPy, PyYAML and standard/custom ROS interfaces",
            "cpu_affinity": [0, 1, 2, 3], "cpu_governor": "performance", "npu_governor": "performance",
            "bundled_custom_message_libraries": [
                {"file": "runtime/lib/" + lib.name, "sha256": digest(package / "runtime/lib" / lib.name)}
                for lib in libraries],
        }
        (package / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n")
        (package / "licenses/README.txt").write_text(
            "This bundle contains apm_bridge and quadrotor_msgs runtime artifacts.\n"
            "Both installed package.xml files declare MIT; their declarations are\n"
            "preserved in runtime/share/<package>/package.xml. Standard ROS and\n"
            "system runtime libraries are provided by the target system.\n")
        executable_names = {"install.sh", "run.sh", "supervise.sh", "performance.sh",
                            "custom_link_bridge_node", "bench_pid_tuner"}
        for path in package.rglob("*"):
            if path.is_symlink():
                raise RuntimeError(f"Unexpected package symlink: {path}")
            path.chmod(0o755 if path.is_dir() or path.name in executable_names else 0o644)
        checksums = "".join(f"{digest(path)}  {path.relative_to(package)}\n"
                            for path in sorted(package.rglob("*")) if path.is_file())
        (package / "SHA256SUMS").write_text(checksums)
        package.rename(destination)
    with tarfile.open(archive, "w") as tar:
        tar.add(destination, arcname=destination.name)
    checksum = digest(archive)
    Path(str(archive) + ".sha256").write_text(f"{checksum}  {archive.name}\n")
    print(json.dumps({"package": str(destination), "tar": str(archive),
                      "bytes": archive.stat().st_size, "sha256": checksum,
                      "bench_pid_enabled": args.enable_bench_pid}))


if __name__ == "__main__":
    main()
