# apm_bridge local development

- Read [the parent workspace's local instructions](../AGENTS.local.md). Resolve this checkout and the adjacent `quadrotor_msgs` source from the current workspace before mounting them into containers.
- Use the Docker workflows in [doc/build-and-test.md](doc/build-and-test.md). Check the selected image and toolchain before building, and choose native output, cross-compilation or emulated execution explicitly. Keep their output directories and caches separate.
- Write build outputs and new validation evidence outside source. Treat temporary directories as disposable; retain required evidence before cleanup. Record the source revision, actual build environment, commands and results in dated reports rather than in this instruction file.
- Resolve board access from current SSH or other connection configuration. Confirm the intended board, deployed revision, services and connected devices before hardware work. Historical aliases, deployment paths and reports do not establish current access or acceptance. Container checks do not establish physical serial, gimbal or Betaflight firmware acceptance.
