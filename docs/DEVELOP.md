# Development Guide

This guide covers the CARLA Studio codebase structure, key classes, and development workflow.

## Code Structure

```
src/
├── app/
│   ├── carla_studio.cpp     (4000+ lines, monolithic GUI)
│   ├── carla_studio.h
│   └── main.cpp             (Qt application entry point)
│
├── core/
│   ├── StudioAppContext.cpp (event loop, configuration)
│   ├── StudioAppContext.h
│   ├── PlayerSlots.h        (simulation state slots)
│   └── SensorMountKey.h     (sensor calibration data)
│
├── calibration/
│   ├── CalibrationScene.cpp (Qt3D drag-drop editor)
│   ├── CalibrationScene.h
│   └── [...mesh rendering, camera controls...]
│
├── vehicle_import/
│   ├── VehicleSpec.h        (canonical vehicle format)
│   ├── Canonicalizer.cpp    (normalize assets)
│   ├── MeshAnalysis.cpp     (bone, material analysis)
│   ├── MeshGeometry.cpp     (vertex/index buffers)
│   ├── MeshPreviewRenderer.cpp (real-time preview)
│   ├── VehicleImportPage.cpp (GUI wizard steps)
│   ├── ImporterClient.cpp   (TCP communication)
│   ├── EditorProcess.cpp    (Unreal subprocess)
│   ├── KitBundler.cpp       (package vehicle format)
│   ├── PluginSetup.cpp      (Unreal integration)
│   ├── NameSanitizer.cpp    (naming conventions)
│   ├── ObjSanitizer.cpp     (OBJ format cleanup)
│   ├── BPAutopicker.cpp     (Blueprint detection)
│   ├── CalibrationCli.cpp   (offline sensor setup)
│   ├── ImportMode.h         (mode constants)
│   └── ImportTestCli.cpp    (validation tool)
│
├── setup_wizard/
│   ├── PrereqProbe.cpp      (Git/Ninja/CMake/UE detection)
│   ├── PrereqProbe.h
│   ├── SetupWizardDialog.cpp (first-run dialog)
│   └── SetupWizardDialog.h
│
├── integrations/
│   ├── HuggingFace.cpp      (dataset discovery)
│   └── HuggingFace.h
│
└── legacy/
    └── [historical UI components]

resources/
├── carla-studio.qrc         (resource manifest)
├── examples/
│   ├── xodr/                (OpenDRIVE examples)
│   └── scenarios/           (JSON scenario templates)
├── integrations/
│   ├── launch_carla.sh
│   └── [integration scripts...]
├── linux/
│   ├── carla-studio.desktop (desktop entry)
│   └── icons/               (16-512px PNG icons)
├── macos/
│   ├── carla-studio.icns    (macOS bundle icon)
│   └── Info.plist
├── windows/
│   ├── carla-studio.ico     (taskbar icon)
│   └── carla-studio.rc      (manifest, resources)
└── plugin_patches/
    └── [Unreal plugin integration patches...]
```

## Key Classes

### StudioAppContext
Central application state and event loop.
```cpp
class StudioAppContext {
  void PollEventQueue();        // Main simulation loop
  void SpawnVehicle(...);       // Vehicle spawning
  void ConfigureSensor(...);    // Sensor attachment
  void RecordScenario(...);     // Timeline recording
};
```
**Responsibilities**: World management, actor lifecycle, sensor state.

### CalibrationScene (Qt3D)
Real-time 3D sensor placement editor with drag-drop targets.
```cpp
class CalibrationScene : QEntity {
  void AttachSensor(...);       // Add sensor entity to scene
  void UpdateSensorTransform(); // Drag-drop transformation
  void PersistJSON(...);        // Save to JSON config
};
```
**Responsibilities**: 3D visualization, transformation, persistence.

### VehicleImportPage
Multi-step wizard for vehicle asset conversion.
```cpp
class VehicleImportPage : QWizardPage {
  void SelectAsset(...);        // Choose FBX
  void PreviewMesh(...);        // Render in 3D
  void StartImport(...);        // Trigger Unreal conversion
};
```
**Flow**: Asset Selection → Preview → Canonicalization → Bundle → Install.

### ImporterClient
TCP communication with Unreal Editor subprocess.
```cpp
class ImporterClient {
  void ConnectToEditor(...);    // TCP 127.0.0.1:9999
  void SendRequest(JSON);       // Serialized command
  JSON ReceiveResponse();       // Unreal output
};
```
**Protocol**: JSON over TCP (127.0.0.1 loopback only).

## Adding New Features

### Adding a New Tab

1. **Extend carla_studio.h:**
   ```cpp
   class CarlaStudio : QMainWindow {
     private:
       QWidget* CreateMyTab();
       void OnMyTabAction();
   };
   ```

2. **Implement in carla_studio.cpp:**
   ```cpp
   QWidget* CarlaStudio::CreateMyTab() {
     auto widget = new QWidget();
     // Layout and controls...
     return widget;
   }
   ```

3. **Register tab:**
   ```cpp
   auto tab = CreateMyTab();
   ui->tabWidget->addTab(tab, "My Tab");
   connect(this, &CarlaStudio::SimulationTick, tab, &QWidget::update);
   ```

### Adding a New Sensor Type

1. **Extend src/core/SensorMountKey.h:**
   ```cpp
   struct SensorMountKey {
     enum Type { Camera, LiDAR, Radar, IMU, MyNewSensor };
   };
   ```

2. **Add calibration UI** in CalibrationScene:
   ```cpp
   void CalibrationScene::AttachMyNewSensor(...) {
     // Create entity, add mesh, set properties
   }
   ```

3. **Hook spawning** in StudioAppContext:
   ```cpp
   void StudioAppContext::SpawnSensor(SensorMountKey key, ...) {
     if (key.type == SensorMountKey::MyNewSensor) {
       // Spawn via CARLA API
     }
   }
   ```

## Building & Testing

### Local Build
```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
cmake --build . -j4
./bin/carla-studio
```

### Debug Build
```bash
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-g -O0"
gdb ./bin/carla-studio
```

### Running Tests
```bash
# Calibration scene unit tests
./build/bin/carla-studio-tests --filter="CalibrationScene*"

# Vehicle import end-to-end
./build/bin/carla-studio-vehicle-import-test --config=test-vehicle.json
```

## Theme System

CARLA Studio supports 7 themes defined in `resources/themes/`:
- **Qt Default** - Native OS theme
- **System Auto** - OS light/dark auto-switch
- **System Light** - Forced OS light theme
- **System Dark** - Forced OS dark theme
- **Glass Light** - Translucent light theme
- **Glass Dark** - Translucent dark theme
- **Nord** - Nord color palette

Apply theme:
```cpp
QApplication::setStyle("Fusion");
QPalette palette;
// Set colors...
qApp->setPalette(palette);
```

## Architecture Notes

### Monolithic Design
carla_studio.cpp (~4000 lines) intentionally keeps UI logic in one file for:
- Rapid prototyping and debugging
- Clear signal/slot connections
- Easier deployment and packaging

### No MVC Separation
Data and UI are tightly coupled for performance (e.g., sensor preview updates at 60fps).

### Threading Model
- **Main thread**: Qt event loop, UI updates
- **Simulator thread**: Poll CARLA world state
- **Network thread**: TCP client for ImporterClient

Use `QThread::currentThread()` to verify context.

### Memory Management
- All QObjects managed by Qt parent system (no explicit `delete`)
- Use smart pointers (`std::unique_ptr`) for C++ objects
- Avoid raw pointers when possible

## Performance Considerations

### Sensor Preview
- Rendered at 60 FPS max (throttled by display refresh)
- Use frustum culling for complex geometries
- Batch 3D draw calls when possible

### World State Polling
- PollEventQueue() called every 16ms (~60 FPS)
- Lazy load actor data (don't fetch all actors every frame)
- Cache waypoint grids to avoid recomputation

### JSON Serialization
- Pre-allocate buffers for known scenario sizes
- Use streaming parser for large datasets (>100MB)

## Common Patterns

### Emitting Signals from Background Thread
```cpp
// BAD: Direct call from worker thread
void WorkerThread::run() {
  Q_EMIT MySignal();  // ❌ UI corruption
}

// GOOD: Use Qt::QueuedConnection
connect(worker, &Worker::MySignal, this, &MainWindow::OnMySignal,
        Qt::QueuedConnection);
```

### Handling Null Pointers
```cpp
// Always check CARLA API returns
if (auto waypoint = actor->GetLocation()) {
  // Use waypoint
} else {
  qWarning() << "Invalid waypoint";
}
```

### Keyboard Input
```cpp
void CarlaStudio::keyPressEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Space) {
    TogglePauseSimulation();
  }
}
```

## Versioning

- Current: **v0.9.0** (Qt 6.4+, sensor calibration, vehicle import)
- Previous: **v0.8.0** (initial Qt UI, basic scenario builder)
- Roadmap: ROS2 integration, neural network inference UI

## Crash Handling

Enable core dumps for debugging:
```bash
ulimit -c unlimited
./build/bin/carla-studio
# Crashes generate core.XXXXX
gdb ./build/bin/carla-studio ./core.XXXXX
```

---

Questions? Open an issue or see [README.md](README.md) for quick start.
