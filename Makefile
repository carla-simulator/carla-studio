.PHONY: help clean integrate gui suite all

JOBS ?= $(shell nproc 2>/dev/null || echo 4)

help:
	@echo "CARLA Studio Build"
	@echo ""
	@echo "Standalone (builds against LibCarla):"
	@echo "  make CARLA_DIR=<path>              - Build GUI + CLI + tests"
	@echo ""
	@echo "Integrated (after 'make integrate'):"
	@echo "  make gui   CARLA_DIR=<path>        - carla-studio (GUI + vehicle-import CLI)"
	@echo "  make suite CARLA_DIR=<path>        - run full test suite (+ --update-documentation)"
	@echo "  make all   CARLA_DIR=<path>        - build all targets"
	@echo ""
	@echo "Other:"
	@echo "  make integrate CARLA_DIR=<path>    - Symlink repo into CARLA workspace"
	@echo "  make clean                         - Remove local build/ directory"
	@echo ""
	@echo "Options:"
	@echo "  JOBS=N   parallel jobs (default: nproc)"
	@echo ""

# ---------------------------------------------------------------------------
# Validation helpers
# ---------------------------------------------------------------------------

check-carla-dir:
	@if [ -z "$(CARLA_DIR)" ]; then \
		echo "Error: CARLA_DIR must be specified"; \
		echo "Usage: make <target> CARLA_DIR=/path/to/carla"; \
		exit 1; \
	fi
	@if [ ! -f "$(CARLA_DIR)/CMakeLists.txt" ]; then \
		echo "Error: $(CARLA_DIR) is not a valid CARLA source directory"; \
		exit 1; \
	fi

check-build-dir: check-carla-dir
	@if [ ! -f "$(CARLA_DIR)/Build/CMakeCache.txt" ]; then \
		echo "Error: $(CARLA_DIR)/Build not configured yet."; \
		echo "Run: cd $(CARLA_DIR)/Build && cmake .. -DBUILD_CARLA_STUDIO=ON"; \
		exit 1; \
	fi

# ---------------------------------------------------------------------------
# Integrated build targets
# ---------------------------------------------------------------------------

gui: check-build-dir
	cmake --build "$(CARLA_DIR)/Build" --target carla-studio -j$(JOBS)

suite: check-build-dir
	cmake --build "$(CARLA_DIR)/Build" --target test-suite_carla-studio-cli -j$(JOBS)
	"$(CARLA_DIR)/Build/Apps/CarlaStudio/test-suite_carla-studio-cli" --update-documentation

all: check-build-dir
	cmake --build "$(CARLA_DIR)/Build" \
	  --target carla-studio \
	  --target test-suite_carla-studio-gui \
	  --target test-suite_carla-studio-cli \
	  -j$(JOBS)

# ---------------------------------------------------------------------------
# Standalone build (no prior integration needed)
# ---------------------------------------------------------------------------

.DEFAULT_GOAL := help

ifeq ($(MAKECMDGOALS),)
else ifneq ($(filter-out help clean integrate check-carla-dir check-build-dir gui suite all,$(MAKECMDGOALS)),)
$(MAKECMDGOALS): check-carla-dir
	@echo "[*] Building CARLA Studio standalone..."
	@mkdir -p build
	@cd build && \
		cmake ../app \
		  -DCMAKE_BUILD_TYPE=Release \
		  -DCMAKE_PREFIX_PATH="$(CARLA_DIR)/Build/install;/opt/carla" \
		  -DBUILD_CARLA_CLIENT=ON \
		  -DCARLA_STUDIO_FETCH_ASSIMP=ON && \
		cmake --build . -j$(JOBS)
	@echo ""
	@echo "[+] Build complete!"
	@echo "[+] Binaries in $$(pwd)/build/Apps/CarlaStudio/"
endif

# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------

integrate: check-carla-dir
	@echo "[*] Setting up integration with CARLA..."
	@if [ -L "$(CARLA_DIR)/Apps/CarlaStudio" ]; then \
		echo "[!] Removing existing symlink at $(CARLA_DIR)/Apps/CarlaStudio"; \
		rm -f "$(CARLA_DIR)/Apps/CarlaStudio"; \
	fi
	@ln -s "$$(pwd)" "$(CARLA_DIR)/Apps/CarlaStudio"
	@echo "[+] Symlink: $(CARLA_DIR)/Apps/CarlaStudio -> $$(pwd)"
	@echo ""
	@echo "[*] Next steps:"
	@echo "    cd $(CARLA_DIR)/Build"
	@echo "    cmake .. -DBUILD_CARLA_STUDIO=ON -DCMAKE_BUILD_TYPE=Release"
	@echo "    make all CARLA_DIR=$(CARLA_DIR)"
	@echo ""

clean:
	@echo "[*] Cleaning build artifacts..."
	@rm -rf build
	@echo "[+] Cleaned"
