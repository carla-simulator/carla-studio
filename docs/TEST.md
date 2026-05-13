# CARLA Studio — Test Guide

## Build

```bash
make test          # build + run full suite
make all           # build everything (app + test binaries)
```

Produced binaries in `app/`:
- `test-suite_carla-studio-cli-linux-x86_64` — CLI pipeline + GUI matrix runner
- `test-suite_carla-studio-gui-linux-x86_64` — GUI source-scan tests

---

## Run

```bash
cd app/
./test-suite_carla-studio-cli-linux-x86_64
./test-suite_carla-studio-gui-linux-x86_64
```

Tests that need a live sim or display use `QSKIP` — the suite never hangs.

---

## Test key legend

| Prefix | Coverage area |
|--------|--------------|
| `a*`   | Binary presence, startup, status states |
| `b*`   | Process monitor (table columns, RAM, CPU, GPU probes) |
| `c*`   | Calibration view dialog and scene |
| `d*`   | Sensor mount dialog (presets, orientation clamps) |
| `e*`   | View tile buttons (FPV / TPV / CPV / BEV) |
| `f*`   | OBJ name sanitizer and mesh AABB parser |
| `g*`   | Extra GUI wiring (actuate level, CARLA root, stop timer, etc.) |
| `h*`   | Home tab (tooltips, weather presets, co-sim checkboxes, runtime target) |
| `i*`   | Interfaces tab (joystick control, keyboard event filter) |
| `j*`   | Live sim reachability (requires CARLA on port 2000) |
| `k*`   | Vehicle import (chassis lift, wheel placement, drive baseline) |
| `m*`   | Help menu (system info, license, docs actions) |
| `n*`   | Health Check tab (refresh button, disk/GPU/RAM rows) |
| `o*`   | ROS / Gazebo / Foxglove tools menu |
| `p*`   | CLI dispatch table (all subcommand entry points compiled in) |
| `q*`   | Plugins menu (rescan action) |
| `r*`   | Render / launch settings persistence (quality, offscreen, NVIDIA) |
| `s*`   | Scenario Builder tab |
| `t*`   | Theme system (exclusive group, applyStudioTheme, themeByKey) |
| `u*`   | Setup / Install dialogs (wizard, community maps, SSH, Docker) |
| `v*`   | Version label click handlers and Vehicle Import tab toggle |
| `x*`   | Protocol detector backends (LIN, FlexRay, MOST, gRPC, DSRC, C-V2X, XCP) |
| `y*`   | Smart launch discovery (findLaunchable, shipping binary, subdirectory search) |

---

## Gated test groups

| Env var | Gates | Notes |
|---------|-------|-------|
| `CARLA_SIM_ROOT` | `j*`, live TC-L* | path to extracted CARLA directory |
| `CARLA_SETUP_TESTS=1` | TC-S* download tests | downloads ~8 GB |
| `CARLA_LONG_TESTS=1` | source build test | ~2 h; also needs `CARLA_UE5_PATH` |
| `DISPLAY` | viewport + screenshot tests | `k01`, `a02` |
| `CARLA_SRC_ROOT` | CarlaTools source scan (`k04`) | CARLA UE plugin source |
| `CARLA_STUDIO_SOURCE_PATH` | all `sourceContains` tests | set automatically by CMake |
| `CARLA_STUDIO_BINARY_PATH` | binary presence tests | set automatically by CMake |
