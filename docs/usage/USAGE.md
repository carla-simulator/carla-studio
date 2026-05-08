# CARLA Studio — Usage Guide

CARLA Studio is a do it all user facing GUI for the CARLA simulator primarily to help new users and also the pro users in the iterative process of debugging some known/common issues.
Every action performed through the GUI would normally require separate terminal commands, Python scripts, libCARLA interfacing and re-compilation and/or manual config edits is made accessible from one app (as much as possible and is still in-works.).

---

## Quick start

### 1. Set required environment variables

CARLA Studio reads these at startup. Please set them in your shell before launching app:

| Variable | Example value | When it is needed |
|---|---|---|
| `CARLA_SRC_ROOT` | `</user/path/prefix>/carla/CARLA_src` | UE editor launch, cook, deploy |
| `CARLA_UNREAL_ENGINE_PATH` | `<user_preferred_path_prefix>/UnrealEngine` | UE editor binary location |
| `CARLA_SHIPPING_ROOT` | `<user_preferred_path_prefix>/Carla-0.10.0-Linux-Shipping` | Deploying to the shipping client |
| `CARLA_STUDIO_FIXTURES` | `<user_preferred_path_prefix>/tests/fixtures/vehicles` | Running fixture-dependent tests only |

Alternatively you can add them to your `~/.bashrc` (or `~/.zshrc`) so you never have to set them again (if you are not switching between multiple versions of carla this is the best thing to do):

```bash
export CARLA_ROOT=<user_preferred_path_prefix>/carla/                       # REQUIRED
export CARLA_SRC_ROOT=<user_preferred_path_prefix>/carla/CARLA_src          # (only required if cooking maps, or vehciles)
export CARLA_UNREAL_ENGINE_PATH=<user_preferred_path_prefix>/UnrealEngine   #(only required if cooking maps, or vehciles)
export CARLA_SHIPPING_ROOT=<user_preferred_path_prefix>/Carla-0.10.0-Linux-Shipping #(only required if cooking maps, or vehciles)
```

### 2. Launch the GUI

```bash
./app/carla-studio-<linux/mac/win>-<arch>
```

The window opens showing the **Home** tab. CARLA does not start automatically —
click **START** when you are ready.

### 3. First-time setup wizard

If this is your first run, click **Cfg → Install / Update CARLA** and follow the
wizard to point Studio at your CARLA installation. You can also run it from
the CLI:

```bash
carla-studio setup
```

---

## The tab strip

All functionality lives on eight tabs. The **screenshot tool** (camera icon,
top-right corner of the strip) waits 3 seconds then captures the entire
desktop to a PNG and your clipboard — useful for capturing open menus before
they close.

---

### ⌂ Home tab

The day-to-day launcher. Configure your session here before clicking START.

| Row | What to set |
|---|---|
| **Map** | Pick a Town, browse for a custom `.umap`, or reload the running sim's current map |
| **View** | Camera mode: Driver / Chase / Cockpit / Bird Eye View |
| **Host** | Where CARLA runs: Local · Docker · Remote (SSH) · NVIDIA Brev |
| **Weather** | Human-readable preset ("Clear · Noon", "Soft Rain · Night", etc.) |
| **Port** | RPC port and endpoint address (default `localhost:2000`) |
| **START / STOP** | START launches the full configured pipeline. STOP is the kill switch for every managed process — CARLA, UE editor, and the shipping client. |
| **Process Control** | Live per-PID rows showing CPU / Memory / GPU. Bars go blue → yellow → red as load crosses 75 % → 95 %. |

The status row at the top shows `CARLA x.y.z with Unreal N` when connected,
`Not Configured` when environment variables are missing, or a red mismatch
badge when the SDK and simulator versions differ.

> **If CARLA will not start:** run the **Health Check** tab first. It will
> tell you exactly which prerequisite is missing before you waste time
> debugging a launch failure.

---

### ⇄ Interfaces tab

Controls *what the ego vehicle senses* and *who drives it*.

The tab is split into two columns that gate progressively — you must configure
**EGO** before any POV or V2X slot becomes interactive.

#### Sense column

| Row | What to configure |
|---|---|
| **Vehicle** | Blueprint ID, paint colour, Substrate finish |
| **Camera × N** | One row per mounted camera. Gear icon opens the per-sensor editor; preview button pops a live view. |
| **RADAR × N** | Same pattern as Camera |
| **LiDAR × N** | Same pattern as Camera |
| **NAV × N** | GNSS / IMU group |
| **GT × N** | Ground-truth sensor taps |

*N* is the count of sensors currently mounted on the EGO. Mount or remove
sensors from **Sensor Calibration** (described below).

#### Actuate column

| Row | What to configure |
|---|---|
| **EGO** | Root of the chain. Pick the driver identity and control method first. |
| **POV.01 – POV.10** | Other-vehicle slots. Locked until EGO is set. |
| **V2X.01 – V2X.06** | Connected-actor slots. Locked until EGO is set. |
| **Self Drive (SAE)** | Autonomy level: L0 (manual) through L5 (full self-drive) |
| **Backend** | **LibCarla** (in-process, lower latency) or **Python** (`manual_control.py` shell-out). Use the ↻ button to refresh if the backend crashes. |

---

### 🩺 Health Check tab

Run this before your first launch and whenever something behaves
unexpectedly. Each row is an independent probe; a red row blocks launch
and shows exactly what is wrong.

| Probe | What it checks |
|---|---|
| CARLA root | Path exists and is readable |
| Engine | Unreal Engine 4 / 5 detected, CARLA version parsed |
| PythonAPI / C++ API | Both bindings present and importable |
| Disk free | Warns below a safe threshold |
| NVIDIA driver | Driver visible; CUDA version reported |
| ROS 2 / ROS-bridge | ROS 2 distro detected, bridge package resolves |
| Scenario Runner | Module importable from the active Python env |
| Leaderboard | Module importable |
| Autoware | Workspace found and `setup.bash` sourceable |
| Plugins | Studio plugin directory scan |
| **SDK ↔ Sim version** | Compares the linked `libcarla-client` build tag against the running simulator's reported version. A mismatch here means a crash is likely — fix it before clicking START. |

Footer buttons: **[Cleanup]** kills stale CARLA/UE processes. **[Re-check]**
reruns all probes. The legend is `●OK · ●optional/not found · ●blocks launch`.

> **SDK ↔ Sim version mismatch:** rebuild `libcarla-client` against the same
> CARLA commit, or use **Cfg → Install / Update CARLA** to align them.

---

### 🛠 Scenario Builder tab

Build and load driving scenarios without leaving Studio.

| Step | Group | What it does |
|---|---|---|
| 1 | **Quickstart Examples** | One-click load of `empty_flat.xodr`, `sample_osm_rural.xodr`, or `scenario_town01_demo.json` to get started immediately |
| 2 | **Map Browser** | List all maps known to the running sim; load, reload, or install more |
| 3 | **OpenDRIVE Tools** | Browse `.xodr` files · Validate offline · Import via `GenerateOpenDriveWorld` · Export via `map.GetOpenDrive()` · Convert OSM → XODR |
| 4 | **2D Map Preview** | Topology and spawn-point visualiser in a scrollable/zoomable view |
| 5 | **Scenario File** | Load / edit / save scenario JSON; **Apply** pushes weather changes and calls `TrySpawnActor` + `WalkerAIController` on the running sim |

---

### ◇ Scenario Re-Construction tab

NVIDIA-powered scene reconstruction pipeline, broken into four tiles:
**NuRec** renderer · **Asset-Harvester** · **NCore** · **Lyra 2.0**.

Each tile is gated by your local hardware. If a tile is greyed out,
hover over it — the tooltip names exactly which Health Check probe is
failing (VRAM, CUDA version, NVIDIA driver). Resolve that probe and the
tile unlocks.

---

### 🏆 Leaderboard tab *(roadmap)*

Submit and benchmark autonomous-driving agents against standard routes.

| Control | Purpose |
|---|---|
| Ranking browser | Filter and search the live `leaderboard.carla.org` rankings |
| Local agent runner | Pick a Python agent + route and run it locally before submitting |
| Track 1 / Track 2 | Sensors-only vs privileged-data selectors |
| Submission helper | Package the agent container, push to the autograder, monitor status |

Driving Score = Route Completion × Infraction Penalty.

---

### 🛡 Testing tab *(roadmap)*

Run regulator-defined safety protocols and generate scored reports.

| Regime | Covered scenarios |
|---|---|
| **Euro NCAP** | AEB CCR, AEB FTAP, AEB VRU, LSS (LKA / LDW / ELK), SAS, DSM |
| **NHTSA** | FMVSS-126, FMVSS-127, FMVSS-208, FMVSS-214, NCAP Frontal / Side / Rollover, AEB Pedestrian (NCAP 2024+) |

Studio loads the matching scenario, configures actors per the protocol,
runs, scores, and emits a PDF / JSON report.

---

### 📋 Logging tab

| Group | What you can do |
|---|---|
| **Recording & Playback** | Start a CARLA `.log` recording, stop it, and replay it with a timeline scrubber |
| **Debug Visualization** | Toggle world-debug overlays (bounding boxes, waypoints, traffic sign labels) |
| **Display Settings** | Pass render flags and driver overlays to the engine without restarting |
| **Python distributions** | Switch the active CARLA `PythonAPI` wheel / egg without leaving the GUI |

---

## Menus

Three top-level menus sit above the tab strip.

### Cfg menu

Everything that configures Studio's environment. Each item opens a modal
dialog; settings persist across sessions via `QSettings`.

| Item | What it does |
|---|---|
| **Install / Update CARLA…** | Pull or refresh the simulator binary against the configured CARLA root |
| **Load Additional Maps…** | Add maps from official CARLA content packs |
| **Community Maps…** | Browse and install community-contributed maps |
| **Cleanup / Uninstall…** | Two-step Detect → Preview → Confirm wipe of CARLA artifacts |
| **Docker Settings…** | Set image tag, runtime flags, and NVIDIA toolkit toggle |
| **Hugging Face Account** | Sign in with a Personal Access Token (stored obfuscated under `hf/token`) |
| **HF Datasets** | Browse and pull datasets from your HF account |
| **HF Models** | Browse and pull models from your HF account |
| **Third-Party Tools** | Detection and path overrides for ROS / Gazebo / Foxglove |
| **Compute (HPC) Settings…** | Configure Slurm / cluster targets for offloaded jobs |
| **Python Environments (uv)** | Manage `uv`-backed Python environments for the CARLA client |
| **Remote / SSH Settings…** | Add SSH hosts, keys, and tunnel ports for the Remote backend |
| **Render flags** | Forward `-RenderOffScreen`, `-quality-level`, etc. to the engine |
| **Driver flags** | NVIDIA-specific knobs: Vulkan / Optix / DLSS gating |
| **ROS-native flags** | Set DDS implementation, RMW, and domain ID for the ros-bridge path |

### Tools menu

Only visible when supported third-party tools are found on `PATH` or
configured via **Cfg → Third-Party Tools**. Items grey out if their binary
disappears between sessions.

**ROS Tools submenu**

| Item | What it launches |
|---|---|
| RViz | `rviz2` connected to the live ROS graph |
| rqt | The `rqt` umbrella UI |
| rqt_graph | Node and topic graph visualiser |
| rqt_console | ROS log console |
| rqt_plot | Live numeric topic plotter |
| rqt_topic | Topic inspector |
| topic-list | `ros2 topic list` snapshot in a side panel |

**Other items**

| Item | Notes |
|---|---|
| Gazebo | Launches `gz sim` or `gazebo` |
| Foxglove Studio | Opens Foxglove pointed at the live websocket bridge |

### Help menu

| Item | What it does |
|---|---|
| Documentation | Opens the CARLA docs site |
| Health Check | Toggles the Health Check tab if you closed it |
| License | Shows the AGPL-3.0-or-later notice, third-party manifest, citation block, and the §13 network-use source link |

---

## Vehicle import wizard

The GUI path to importing a custom 3D vehicle model into CARLA.

1. Open the **Vehicle Import** tab.
2. Click **Browse** and select your mesh file (`.obj`, `.fbx`, or `.glb`).
3. Optionally click **Browse Tires** to supply a separate tires mesh.
4. Inspect and adjust wheel anchor positions in the 3D preview pane.
5. Click **Import**. A progress bar tracks the five pipeline stages:
   `preflight → interchange (UE5) → build SK → build BP → persist`.
6. After a successful cook, click **Deploy & Drive** to spawn the vehicle
   in the running shipping client and drive-test it immediately.

> **No textures?** That is fine. The importer accepts flat-shaded geometry;
> missing texture files are not an error.

---

## Sensor calibration

1. Open the **Sensor Calibration** dialog (accessible from the Interfaces tab
   or the sidebar).
2. Drag sensors from the palette onto the 3D vehicle model.
3. Use the transform gizmo or the numeric fields to set exact position and
   rotation.
4. Click **Save / Apply** to persist the calibration.
5. Click **Export JSON** to write a calibration file for use in ROS 2 /
   Autoware.

---

## Control surfaces

| Method | How to activate |
|---|---|
| **Keyboard** | WASD or arrow keys drive the ego vehicle while a session is active |
| **Joystick** | Any SDL2-compatible gamepad is auto-detected; remap axes in **Cfg → Settings** |

---

## Configuration reference (QSettings)

Studio persists all settings to `QSettings` under
`CARLA Simulator → CARLA Studio`. You do not normally edit these directly,
but they are useful for scripting or resetting a stuck value.

| Key | What it controls |
|---|---|
| `runtime/target` | Backend: Local / Docker / Remote / Brev |
| `vehicle/blueprint` | Default ego blueprint ID |
| `vehicle/color` / `vehicle/finish` | Default ego paint colour and Substrate finish |
| `actuate/player_<NAME>` | Per-POV control method (Keyboard / Joystick N) |
| `actuate/sae_level` | SAE J3016 autonomy level (0–5) |
| `actuate/backend` | LibCarla vs Python API |
| `addon/terasim_enabled` / `addon/autoware_enabled` | Integration toggles |
| `addon/terasim_scenario` / `addon/autoware_stack` | Per-integration selection |
| `addon/renderer` | Default vs NuRec renderer |
| `hf/token` | Hugging Face PAT (stored obfuscated) |
| `python/active_dist` | Active CARLA PythonAPI wheel or egg |
| `ros/pseudo/<id>` | Per-pseudo-sensor enable toggle |

---

## CLI reference

The same pipeline available in the GUI is also accessible headlessly.

### Vehicle import

```bash
# Minimal — mesh only
carla-studio vehicle-import --mesh /path/to/car.obj

# With a separate tires mesh
carla-studio vehicle-import --mesh car.obj --tires tires.obj

# Cook, deploy, and open a drive session immediately after import
carla-studio vehicle-import --mesh car.obj --manual-drive

# Debug wheel anchor geometry only — no editor launch
carla-studio vehicle-import --mesh car.obj --debug-merge-only
```

### Other subcommands

```bash
carla-studio help       # list all available subcommands
carla-studio cleanup    # terminate all CARLA and UE editor processes
carla-studio setup      # re-run the first-time setup wizard
```

### Running tests

```bash
# Unit and pipeline tests — no CARLA installation required
./app/carla-studio-<linux/mac/win>-<arch>/test-suite_carla-studio-cli

# With vehicle fixture files
CARLA_STUDIO_FIXTURES=/path/to/tests/fixtures/vehicles \
  ./app/carla-studio-<linux/mac/win>-<arch>/test-suite_carla-studio-cli
```

See [tests/fixtures/README.md](../tests/fixtures/README.md) for fixture file requirements.

---

## If Studio crashes

Studio installs a crash handler at startup that catches unhandled exceptions
from background threads and writes a readable error to `stderr` before exiting
cleanly. You will see something like:

```
CARLA Studio: std::terminate fired
  type:  std::bad_array_new_length
  what:  std::bad_array_new_length
  hint:  LibCarla RPC version mismatch — rebuild libcarla-client
         against the running simulator, or run Health Check →
         "SDK ↔ Sim version" to confirm compatibility.
```

**What each field means:**

- `type:` — the C++ exception class (demangled). Use this to search issues.
- `what:` — the exception's own message.
- `hint:` — the remediation step matched to that exception type.

**Common causes and fixes:**

| `type:` value | Cause | Fix |
|---|---|---|
| `std::bad_array_new_length` | `libcarla-client` built against a different CARLA commit than the running sim | Rebuild the client or reinstall via **Cfg → Install / Update CARLA** |
| `std::system_error` | Broken pipe — sim process died while Studio was connected | Restart the sim; check disk space and GPU memory |
| `std::bad_alloc` | Out of memory in a client thread | Close other applications; reduce sensor count |

The **Health Check** tab's **SDK ↔ Sim version** row detects the most common
crash cause *before* it happens. If that row is green, `std::bad_array_new_length`
cannot fire from RPC deserialisation.
