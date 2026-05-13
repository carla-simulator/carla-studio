# CARLA Studio

Qt based Studio SW application for use with CARLA 
Currently provides features like setup from src or from binaries, vehicle import, sensor calibration, and simulation control with health check.
Uses interfaces like libCARLA and PythonAPI to allow further standard interfacing when built with a carla installation.(see below)

## Build 

```bash
make
```
Binary: `app/carla-studio`. Point the app to your CARLA installation at runtime via `CARLA_ROOT` and it resolves paths automatically.

### Build with CARLA source code (ue5-dev / ue4-dev) branches

```bash
make all CARLA_DIR=/path/to/carla/source
```

## Docs

- [BUILD.md](docs/BUILD.md) - prerequisites, build options, troubleshooting
- [USAGE.md](docs/usage/USAGE.md) - CLI and GUI usage reference
- [DEVELOP.md](docs/DEVELOP.md) - code structure and contribution notes
