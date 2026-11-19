# apm_bridge Project Conventions

Read [AGENTS.local.md](AGENTS.local.md) if present before building or running device checks.

## Repository boundaries

- Make shared ROS message and service definition changes in `quadrotor_msgs`. Keep generated interface headers and libraries out of version control. Make Betaflight firmware and Cosys-AirSim changes in their owning repositories.
- Keep the optional AirSim camera API dependency separate from the serial and TCP custom-link transports.
- Preserve the five private-to-`/fpv` topic remappings across hardware launch, firmware simulation (SITL) launch and deployment. When documenting parameter behavior, distinguish node defaults from hardware YAML and SITL YAML overrides.

## Verification

- Follow the supported default workflow in the [build and test guide](doc/build-and-test.md) unless a different documented build mode is explicitly selected. Keep outputs and caches separate by build mode and target architecture.
- After C++, CMake or dependency changes, build `custom_link_bridge_node`, `gimbal_selftest` and the test targets with `BUILD_TESTING=ON`. Discover and run all CTest groups registered by the selected configuration and the additional regressions in the build guide.
- Report host tests, emulated execution, firmware simulation, serial time-sync checks and physical acceptance separately when applicable. Identify missing prerequisites for checks you could not run. Follow the [production deployment guide](doc/production-deployment.md) for board checks and preserve its dated evidence in the original deployment context.
