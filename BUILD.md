# Building CARLA Studio

This guide covers building CARLA Studio in standalone or integrated modes.

## Prerequisites

### System Requirements
- **Ubuntu 24.04+**: GCC 9+, Qt 6.4+, CMake 3.21+
- **macOS 12+**: Clang 14+, Qt 6.4+, CMake 3.21+
- **Windows 10+**: MSVC 2019+, Qt 6.4+, CMake 3.21+

### Required Libraries

#### Qt Framework (Primary)
```bash
# Ubuntu
sudo apt install -y qt6-base-dev qt6-3d-dev qt6-charts-dev

# macOS (Homebrew)
brew install qt cmake ninja boost

# Windows (Qt Installer)
# Download from https://www.qt.io/download
# Install: Qt 6.4+ with Desktop component
```

#### Build Tools
```bash
# Ubuntu
sudo apt install -y cmake ninja-build build-essential

# macOS
brew install cmake ninja

# Windows (MSVC 2019 or later)
# Install from https://visualstudio.microsoft.com/
```

#### Boost Libraries
```bash
# Ubuntu
sudo apt install -y libboost-dev libboost-system-dev

# macOS
brew install boost

# Windows (vcpkg)
vcpkg install boost:x64-windows
```

## Standalone Build

### Step 1: Clone Repository
```bash
git clone https://github.com/yourusername/carla-studio.git
cd carla-studio
```

### Step 2: Configure CMake
```bash
# Using Makefile (recommended)
make CARLA_DIR=/path/to/carla

# Or manual CMake
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j4
```

### Step 3: Run
```bash
./build/bin/carla-studio
```

## Integrated Build

### Step 1: Link into CARLA
```bash
cd /tmp/carla-studio-clean
make integrate CARLA_DIR=/path/to/carla
```

This creates `/path/to/carla/Apps/CarlaStudio → carla-studio/` symlink.

### Step 2: Build Main CARLA
```bash
cd /path/to/carla
mkdir -p Build && cd Build
cmake .. -DBUILD_CARLA_STUDIO=ON -DCMAKE_BUILD_TYPE=Release
cmake --build . --target carla-studio -j4
```

### Step 3: Run
```bash
./Build/bin/carla-studio
```

## Qt Version Selection

### Automatic Detection
CMakeLists.txt auto-detects Qt version in this order:
1. Qt 6 (preferred) — modern, better performance
2. Qt 5.15+ (fallback) — legacy support

### Force Qt Version
```bash
# Use Qt5
cmake .. -DQt5_FOUND=ON -DQt6_FOUND=OFF

# Use Qt6
cmake .. -DQt6_FOUND=ON -DQt5_FOUND=OFF
```

## Optional Features

### Vehicle Import (Assimp)
```bash
# Auto-fetch during build
cmake .. -DCARLA_STUDIO_FETCH_ASSIMP=ON
```

### LibCarla Integration
For full CARLA C++ API access:
```bash
cmake .. \
  -DCMAKE_PREFIX_PATH="/path/to/carla/Build/install" \
  -DBUILD_CARLA_CLIENT=ON
```

### Joystick/Gamepad Support
Auto-detected if system has SDL2:
```bash
sudo apt install -y libsdl2-dev  # Ubuntu
brew install sdl2                 # macOS
```

## Build Issues & Solutions

### CMake Errors

**Error**: `Qt6 not found`
```bash
# Check Qt installation
qmake6 --version
# or
cmake .. -DCMAKE_PREFIX_PATH=$(brew --prefix qt)
```

**Error**: `3D modules missing`
```bash
# Ubuntu: additional Qt6 packages
sudo apt install -y qt6-3d-dev qt6-charts-dev qt6-positioning-dev

# macOS: Qt6 bundle includes 3D by default
```

### Linker Errors

**Error**: `undefined reference to OpenGL`
```bash
# Ubuntu
sudo apt install -y libopengl-dev mesa-common-dev

# Check OpenGL version
glxinfo | grep "OpenGL version"
```

**Error**: `Assimp not found`
```bash
# Use CMake fetch option
cmake .. -DCARLA_STUDIO_FETCH_ASSIMP=ON

# Or install system package
sudo apt install -y libassimp-dev  # Ubuntu
brew install assimp                # macOS
```

### LibCarla Linking

If integration build fails with LibCarla:
```bash
# Rebuild LibCarla first
cd /path/to/carla/Build
cmake --build . --target carla-client -j4

# Then retry Studio build
cmake .. -DCMAKE_PREFIX_PATH="/path/to/carla/Build/install"
```

## Runtime Issues

### Sensor Preview Blank
- Requires OpenGL 3.3+ capable GPU
- On Linux: check `glxinfo -B | grep "OpenGL version"`
- On WSL2: enable GPU acceleration via `config` file

### Crashes on Startup
- Enable debug symbols: `cmake .. -DCMAKE_BUILD_TYPE=Debug`
- Run with logs: `./carla-studio 2>&1 | tee debug.log`
- Check dependencies: `ldd ./carla-studio` (Linux)

### Theme Issues
If GUI elements look corrupted:
```bash
# Reset QT settings
rm -rf ~/.config/QtProject/

# Force theme
./carla-studio --style=fusion
```

## Performance Optimization

### Build for Release
```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -march=native"
cmake --build . -j$(nproc)
```

### Parallel Compilation
```bash
cmake --build . -j$(nproc)
```

## CI/CD Integration

### GitHub Actions
```yaml
- name: Build CARLA Studio
  run: |
    make CARLA_DIR=/tmp/carla
```

### Jenkins
```groovy
stage('Build Studio') {
  steps {
    sh 'make CARLA_DIR=${CARLA_HOME}'
  }
}
```

## Verification

### Quick Test
```bash
./build/bin/carla-studio --version
./build/bin/carla-studio --help
```

### Full Test
```bash
# Build with tests enabled
cmake .. -DBUILD_TESTS=ON
cmake --build . --target carla-studio-tests -j4
./build/bin/carla-studio-tests
```

---

For development and code structure, see [DEVELOP.md](DEVELOP.md).
