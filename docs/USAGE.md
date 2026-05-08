# CARLA Studio - Usage

## CLI

### Environment variables

Set these before running the vehicle import CLI or the UE import pipeline:

| Variable | Example | Required for |
|----------|---------|-------------|
| `CARLA_SRC_ROOT` | `/path/to/carla/CARLA_src` | UE editor launch, cook, deploy |
| `CARLA_UNREAL_ENGINE_PATH` | `/path/to/UnrealEngine` | UE editor binary |
| `CARLA_SHIPPING_ROOT` | `/path/to/Carla-0.10.0-Linux-Shipping` | Ship target deploy |
| `CARLA_STUDIO_FIXTURES` | `/path/to/tests/fixtures/vehicles` | Fixture-dependent tests |

### Vehicle import

Import a 3D car model into CARLA:

```bash
carla-studio vehicle-import --mesh /path/to/car.obj
```

With a separate tires mesh:

```bash
carla-studio vehicle-import --mesh car.obj --tires tires.obj
```

Drive-test the vehicle after cook:

```bash
carla-studio vehicle-import --mesh car.obj --manual-drive
```

Debug wheel anchor positions without launching the editor:

```bash
carla-studio vehicle-import --mesh car.obj --debug-merge-only
```

The pipeline runs five stages: **preflight → interchange (UE5) → build SK → build BP → persist**, then optionally cooks, deploys, and spawns in shipping CARLA.

### Other subcommands

```bash
carla-studio help       # list all subcommands
carla-studio cleanup    # kill all CARLA / UE processes
carla-studio setup      # run the first-run setup wizard
```

### Running tests

```bash
# Unit + pipeline tests (no CARLA installation needed)
./Build/Apps/CarlaStudio/carla-studio-cli-test

# With vehicle fixtures
CARLA_STUDIO_FIXTURES=/path/to/tests/fixtures/vehicles \
  ./Build/Apps/CarlaStudio/carla-studio-cli-test
```

See [tests/fixtures/README.md](tests/fixtures/README.md) for fixture requirements.

---

## GUI

Launch:

```bash
./Build/Apps/CarlaStudio/carla-studio
```

### Connection

- Enter the CARLA server address and port (default `localhost:2000`).
- **Connect** opens the session; **STOP** is the kill switch for all managed processes (CARLA, UE editor, shipping client).

### Vehicle import wizard

GUI path to the same pipeline as the CLI. Open **Vehicle Import** from the sidebar:

1. Select a mesh file (`.obj`, `.fbx`, `.glb`).
2. Optionally pick a separate tires mesh.
3. Adjust wheel anchor positions in the 3D preview.
4. Click **Import** - progress tracks the five pipeline stages.
5. After cook, click **Deploy & Drive** to spawn in shipping CARLA.

### Sensor calibration

Open **Sensor Calibration** to place and orient sensors on a vehicle model:

- Drag-and-drop sensors from the panel onto the 3D view.
- Adjust position and rotation via the transform gizmo or numeric fields.
- Export a calibration JSON for use in ROS2 / Autoware.

### World and actor inspection

- **World** tab lists all actors in the connected CARLA world.
- Click an actor to inspect its attributes and bounding box.
- **Spectate** locks the camera to the selected actor.

### Replay and recording

- **Record** captures a CARLA `.log` recording.
- **Replay** plays back a recording; scrub with the timeline slider.
- Frame-step forward/back with the arrow keys.

### Control surfaces

- **Keyboard** - WASD / arrow keys drive the ego vehicle when a session is active.
- **Joystick** - any SDL2-compatible gamepad is auto-detected; axis mapping is configurable in Settings.

### Scenario tools

- Load an XODR map or a scenario JSON from `resources/maps/`.
- **Scenario Builder** lets you place actors, set waypoints, and export a scenario file.

### Settings

- Connection presets (name, host, port) are persisted across sessions.
- SSH tunnel configuration for remote CARLA servers.
- Theme, language, and log-level options.
