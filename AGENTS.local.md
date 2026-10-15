# apm_bridge local development

- Read `/home/mzh/nickluo/AGENTS.local.md`. Source is `/home/mzh/nickluo/apm_bridge`; shared messages are `/home/mzh/nickluo/quadrotor_msgs`.
- Follow `docs/build-and-test.md`. Default image architecture is x86_64, with native x86_64 or cross-compiled ARM64 output. The ARM64 image remains an explicit fallback.
- Put new evidence outside source, for example `/tmp/apm_bridge-native` and `/tmp/apm_bridge-cross`. Historical outputs under `/home/mzh/nickluo-validation-20261015/apm_bridge/` remain unchanged.
- Board SSH aliases in production documentation belong to the original workstation. This host has no configured board access. Container checks do not establish serial, gimbal or Betaflight firmware acceptance.
