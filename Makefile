.PHONY: all app test clean help _build _deploy _install_desktop

JOBS  ?= $(shell nproc 2>/dev/null || sysctl -n hw.logicalcpu 2>/dev/null || echo 4)
BUILD := $(CURDIR)/app

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
_CLI    := carla-studio-test-suite-$(_PLATFORM)-$(_ARCH)$(_EXT)

ifdef CARLA_DIR
_CARLA := -DCARLA_DIR="$(CARLA_DIR)"
else
_CARLA :=
endif

$(BUILD)/CMakeCache.txt:
	cmake -S src -B "$(BUILD)" -DCMAKE_BUILD_TYPE=Release $(_CARLA)

_build: $(BUILD)/CMakeCache.txt
	cmake --build "$(BUILD)" --target carla-studio -j$(JOBS)

ifeq ($(_PLATFORM), macos)
_deploy: _build
	@command -v macdeployqt >/dev/null 2>&1 \
	  && macdeployqt "$(BUILD)/carla-studio.app" \
	  || echo "[!] macdeployqt not found - Qt libs not bundled"
	cp -r "$(BUILD)/carla-studio.app" "$(BUILD)/$(_STUDIO)"

else ifeq ($(_PLATFORM), windows)
_deploy: _build
	@command -v windeployqt >/dev/null 2>&1 \
	  && windeployqt "$(BUILD)/carla-studio.exe" \
	  || echo "[!] windeployqt not found - Qt libs not bundled"
	cp "$(BUILD)/carla-studio.exe" "$(BUILD)/$(_STUDIO)"

else
_deploy: _build
	cp "$(BUILD)/carla-studio" "$(BUILD)/$(_STUDIO)"
endif

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

.DEFAULT_GOAL := _release
_release: _deploy _install_desktop
	@find "$(BUILD)" -maxdepth 1 -mindepth 1 \
	  -not -name "$(_STUDIO)" \
	  -not -name "carla-studio$(_APP_EXT)" \
	  -not -name "cfg" \
	  -exec rm -rf {} + 2>/dev/null || true
	@echo ""
	@echo "[+] app/$(_STUDIO)"
	@echo "[+] app/carla-studio$(_APP_EXT)"

all: $(BUILD)/CMakeCache.txt
	cmake --build "$(BUILD)" --target carla-studio -j$(JOBS)
	cmake --build "$(BUILD)" --target test-suite   -j$(JOBS)
	cp "$(BUILD)/carla-studio$(_EXT)"                "$(BUILD)/$(_STUDIO)"
	cp "$(BUILD)/carla-studio-test-suite$(_EXT)" "$(BUILD)/$(_CLI)"
	@find "$(BUILD)" -maxdepth 1 -mindepth 1 \
	  -not -name "$(_STUDIO)" \
	  -not -name "carla-studio$(_APP_EXT)" \
	  -not -name "$(_CLI)" \
	  -not -name "cfg" \
	  -exec rm -rf {} + 2>/dev/null || true
	@echo ""
	@echo "[+] app/$(_STUDIO)"
	@echo "[+] app/$(_CLI)"

app: $(BUILD)/CMakeCache.txt
	cmake --build "$(BUILD)" --target carla-studio -j$(JOBS)

test: $(BUILD)/CMakeCache.txt
	cmake --build "$(BUILD)" --target carla-studio -j$(JOBS)
	cmake --build "$(BUILD)" --target test-suite   -j$(JOBS)
	"$(BUILD)/carla-studio-test-suite$(_EXT)" --update-documentation

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
	@echo "  make             build + keep only carla-studio binary (default)"
	@echo "                   → app/$(_STUDIO)"
	@echo "                   → app/carla-studio$(_APP_EXT)"
	@echo "  make all         build everything + keep all binaries in app/"
	@echo "                   → app/$(_STUDIO)"
	@echo "                   → app/$(_CLI)"
	@echo "  make app         build carla-studio only (no rename/prune)"
	@echo "  make test        build + run full test suite"
	@echo "  make clean       remove app/ and uninstall .desktop entry (Linux)"
	@echo ""
	@echo "  CARLA_DIR=<path>  link against CARLA source tree"
	@echo "  JOBS=N            parallel build jobs (default: all cores via nproc)"
