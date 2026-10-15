# apm_bridge repository instructions

- Use C++17 and ROS 2 Humble. Read `AGENTS.local.md` and the parent workspace's local instructions before building.
- Use `drone-tracker-rk3576:humble-amd64-rknn2.3.2` by default. Native builds produce x86_64; its ARM cross toolchain produces ARM64. Container architecture is not a target restriction.
- Follow [docs/build-and-test.md](docs/build-and-test.md) for complete native, ARM64 cross-build, explicit ARM64-image fallback and protocol-test commands.
- Build the real adjacent `quadrotor_msgs` source for the same target before this package. `dist/` runtime files are not a development dependency. Do not copy generated message headers or shared libraries into this repository.
- Mount source read-only and keep build/install/log directories outside source. Keep output architectures and image caches separate.
- After C++, CMake or dependency changes, build both executables and run applicable protocol/time-sync tests. No registered CTests exist; compilation alone is not a test pass.
- Physical Betaflight/custom-link and gimbal acceptance requires the actual devices. See `docs/production-deployment.md`; preserve its dated board evidence.
