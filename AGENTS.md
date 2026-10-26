# apm_bridge repository instructions

- Use C++17 and ROS 2 Humble. Read `AGENTS.local.md` and the parent workspace's local instructions before building.
- Use `drone-tracker-rk3576:humble-amd64-rknn2.3.2` by default. Native builds produce x86_64; its ARM cross toolchain produces ARM64. Container architecture is not a target restriction.
- Follow [docs/build-and-test.md](docs/build-and-test.md) for native builds and tests, ARM64 cross-builds and the explicit ARM64-image fallback.
- Build the real adjacent `quadrotor_msgs` source for the same target before this package. `dist/` runtime files are not a development dependency. Do not copy generated message headers or shared libraries into this repository.
- Mount source read-only and keep build/install/log directories outside source. Keep output architectures and image caches separate.
- After C++, CMake or dependency changes, build `custom_link_bridge_node` and `gimbal_selftest` with `BUILD_TESTING=ON`. Run the eight registered CTest groups and the separate `test/cli_service_test.py` regression. Compilation or zero discovered tests is not a test pass. Run applicable firmware, time-sync and device checks separately, and report unavailable prerequisites.
- Keep Betaflight firmware and Cosys-AirSim external. `APM_BRIDGE_AIRSIM=ON` requires the real Cosys-AirSim rpclib source selected by `AIRSIM_ROOT`; the default build disables camera RPC (remote procedure calls) but supports serial and TCP custom-link transports.
- Preserve launch remappings for private `~/control_command`, `~/control_command_raw`, `~/arm`, `~/offboard` and `~/low_level_feedback` topics. Distinguish class defaults from the hardware and SITL YAML overrides when documenting behavior.
- Physical Betaflight/custom-link and gimbal acceptance requires the actual devices. See `docs/production-deployment.md`; preserve its dated board evidence.
