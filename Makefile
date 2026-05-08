.PHONY: all gui suite clean help _build _deploy _install_desktop

JOBS  ?= $(shell nproc 2>/dev/null || sysctl -n hw.logicalcpu 2>/dev/null || echo 4)
BUILD := $(CURDIR)/app

# ---------------------------------------------------------------------------
# Platform + architecture
# ---------------------------------------------------------------------------
_OS   := $(shell uname -s 2>/dev/null || echo Windows_NT)
_ARCH := $(shell uname -m 2>/dev/null || echo x86_64)

ifeq ($(_OS), Linux)
  _PLATFORM := linux
  _EXT      :=
  _APP_EXT  :=
else ifeq ($(_OS), Darwin)
  _PLATFORM := macos
  _EXT      :=
  _APP_EXT  := .app
else
  _PLATFORM := windows
  _EXT      := .exe
  _APP_EXT  := .exe
endif

_STUDIO := carla-studio-$(_PLATFORM)-$(_ARCH)$(_APP_EXT)
_CLI    := test-suite_carla-studio-cli-$(_PLATFORM)-$(_ARCH)$(_EXT)
_GUI_TS := test-suite_carla-studio-gui-$(_PLATFORM)-$(_ARCH)$(_EXT)

# ---------------------------------------------------------------------------
# Optional CARLA source linkage
# ---------------------------------------------------------------------------
ifdef CARLA_DIR
_CARLA := -DCARLA_DIR="$(CARLA_DIR)"
else
_CARLA :=
endif

# ---------------------------------------------------------------------------
# cmake configure
# ---------------------------------------------------------------------------
$(BUILD)/CMakeCache.txt:
	cmake -S src -B "$(BUILD)" -DCMAKE_BUILD_TYPE=Release $(_CARLA)

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
_build: $(BUILD)/CMakeCache.txt
	cmake --build "$(BUILD)" \
	  --target carla-studio \
	  --target test-suite_carla-studio-gui \
	  --target test-suite_carla-studio-cli \
	  -j$(JOBS)

# ---------------------------------------------------------------------------
# Deploy - bundle Qt libs and rename to platform binary
# ---------------------------------------------------------------------------
ifeq ($(_PLATFORM), macos)
_deploy: _build
	@command -v macdeployqt >/dev/null 2>&1 \
	  && macdeployqt "$(BUILD)/carla-studio.app" \
	  || echo "[!] macdeployqt not found - Qt libs not bundled"
	cp -r "$(BUILD)/carla-studio.app"            "$(BUILD)/$(_STUDIO)"
	cp    "$(BUILD)/test-suite_carla-studio-cli" "$(BUILD)/$(_CLI)"
	cp    "$(BUILD)/test-suite_carla-studio-gui" "$(BUILD)/$(_GUI_TS)"

else ifeq ($(_PLATFORM), windows)
_deploy: _build
	@command -v windeployqt >/dev/null 2>&1 \
	  && windeployqt "$(BUILD)/carla-studio.exe" \
	  || echo "[!] windeployqt not found - Qt libs not bundled"
	cp "$(BUILD)/carla-studio.exe"                "$(BUILD)/$(_STUDIO)"
	cp "$(BUILD)/test-suite_carla-studio-cli.exe" "$(BUILD)/$(_CLI)"
	cp "$(BUILD)/test-suite_carla-studio-gui.exe" "$(BUILD)/$(_GUI_TS)"

else
_deploy: _build
	cp "$(BUILD)/carla-studio"                "$(BUILD)/$(_STUDIO)"
	cp "$(BUILD)/test-suite_carla-studio-cli" "$(BUILD)/$(_CLI)"
	cp "$(BUILD)/test-suite_carla-studio-gui" "$(BUILD)/$(_GUI_TS)"
endif

# ---------------------------------------------------------------------------
# Targets
# ---------------------------------------------------------------------------
ifeq ($(_PLATFORM), linux)
_install_desktop:
	@mkdir -p "$(HOME)/.local/share/applications"
	@mkdir -p "$(HOME)/.local/share/icons/hicolor/scalable/apps"
	@mkdir -p "$(HOME)/.local/share/icons/hicolor/1024x1024/apps"
	@sed "s|Exec=carla-studio.*|Exec=$(BUILD)/$(_STUDIO)|" \
	  src/icons/linux/carla-studio.desktop \
	  > "$(HOME)/.local/share/applications/carla-studio.desktop"
	@cp src/icons/linux/hicolor/scalable/apps/carla-studio.svg \
	  "$(HOME)/.local/share/icons/hicolor/scalable/apps/carla-studio.svg"
	@cp src/icons/linux/hicolor/1024x1024/apps/carla-studio.png \
	  "$(HOME)/.local/share/icons/hicolor/1024x1024/apps/carla-studio.png"
	@update-icon-caches "$(HOME)/.local/share/icons/hicolor" 2>/dev/null || true
	@echo "[+] ~/.local/share/applications/carla-studio.desktop"
else
_install_desktop:
endif

ifeq ($(_PLATFORM), macos)
_APP_KEEP := -not -name "carla-studio$(_APP_EXT)"
else
_APP_KEEP :=
endif

all: _deploy _install_desktop
ifdef KEEP_TESTS
	@find "$(BUILD)" -mindepth 1 \
	  -not -name "$(_STUDIO)"  $(_APP_KEEP) \
	  -not -name "$(_CLI)"     -not -name "test-suite_carla-studio-cli$(_EXT)" \
	  -not -name "$(_GUI_TS)"  -not -name "test-suite_carla-studio-gui$(_EXT)" \
	  -delete
	@echo ""
	@echo "[+] app/$(_STUDIO)"
	@echo "[+] app/$(_CLI)"
	@echo "[+] app/$(_GUI_TS)"
else
	@find "$(BUILD)" -mindepth 1 \
	  -not -name "$(_STUDIO)"  $(_APP_KEEP) \
	  -delete
	@echo ""
	@echo "[+] app/$(_STUDIO)"
endif

gui: $(BUILD)/CMakeCache.txt
	cmake --build "$(BUILD)" --target carla-studio -j$(JOBS)

suite: $(BUILD)/CMakeCache.txt
	cmake --build "$(BUILD)" --target test-suite_carla-studio-cli -j$(JOBS)
	"$(BUILD)/test-suite_carla-studio-cli$(_EXT)" --update-documentation

clean:
	rm -rf "$(BUILD)"
ifeq ($(_PLATFORM), linux)
	@rm -f "$(HOME)/.local/share/applications/carla-studio.desktop"
	@rm -f "$(HOME)/.local/share/icons/hicolor/scalable/apps/carla-studio.svg"
	@rm -f "$(HOME)/.local/share/icons/hicolor/1024x1024/apps/carla-studio.png"
	@update-icon-caches "$(HOME)/.local/share/icons/hicolor" 2>/dev/null || true
endif

help:
	@echo "Usage: make [target] [CARLA_DIR=/path/to/carla/source]"
	@echo ""
	@echo "Targets:"
	@echo "  all              build + package (app binary only)"
	@echo "                   → app/$(_STUDIO)"
	@echo "  all KEEP_TESTS=1 build + package including test suites"
	@echo "                   → app/$(_STUDIO)"
	@echo "                   → app/$(_CLI)"
	@echo "                   → app/$(_GUI_TS)"
	@echo "  gui              carla-studio binary only (no packaging)"
	@echo "  suite            build + run cli test suite"
	@echo "  clean            remove app/ and uninstall .desktop entry (Linux)"
	@echo ""
	@echo "  CARLA_DIR=<path>  link against CARLA source tree"
	@echo "  JOBS=N            parallel build jobs (default: nproc)"

.DEFAULT_GOAL := help
