# CARLA Studio

Qt based Studio SW application for use with CARLA 
Currently provides features like setup from src or from binaries, vehicle import, sensor calibration, and simulation control with health check.
Uses interfaces like libCARLA and PythonAPI to allow further standard interfacing.

## Quick build

```bash
make integrate CARLA_DIR=/path/to/carla
cd /path/to/carla/Build
cmake .. -DBUILD_CARLA_STUDIO=ON -DCMAKE_BUILD_TYPE=Release
make all CARLA_DIR=/path/to/carla
```

Binaries land in `Build/Apps/CarlaStudio/`.

## Docs

- [BUILD.md](BUILD.md) — prerequisites, build options, troubleshooting
- [USAGE.md](USAGE.md) — CLI and GUI usage reference
- [DEVELOP.md](DEVELOP.md) — code structure and contribution notes
