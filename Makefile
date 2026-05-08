.PHONY: help clean integrate

help:
	@echo "CARLA Studio Build"
	@echo ""
	@echo "Usage:"
	@echo "  make CARLA_DIR=<path>            - Standalone build with LibCarla"
	@echo "  make integrate CARLA_DIR=<path>  - Symlink & integrate with main CARLA repo"
	@echo "  make clean                       - Clean build artifacts"
	@echo ""

# Validate CARLA_DIR is provided
check-carla-dir:
	@if [ -z "$(CARLA_DIR)" ]; then \
		echo "Error: CARLA_DIR must be specified"; \
		echo "Usage: make CARLA_DIR=/path/to/carla"; \
		exit 1; \
	fi
	@if [ ! -f "$(CARLA_DIR)/CMakeLists.txt" ]; then \
		echo "Error: $(CARLA_DIR) is not a valid CARLA source directory"; \
		echo "Expected CMakeLists.txt not found"; \
		exit 1; \
	fi
	@echo "[✓] CARLA_DIR validated: $(CARLA_DIR)"

# Default target (if make is called without CARLA_DIR)
.DEFAULT_GOAL := help

# Standalone build: compile against external carla-client
ifeq ($(MAKECMDGOALS),)
else ifneq ($(filter-out help clean integrate check-carla-dir,$(MAKECMDGOALS)),)
# User is calling make with CARLA_DIR for standalone build
$(MAKECMDGOALS): check-carla-dir
	@echo "[*] Building CARLA Studio standalone..."
	@mkdir -p build
	@cd build && \
		cmake .. \
		  -DCMAKE_BUILD_TYPE=Release \
		  -DCMAKE_PREFIX_PATH="$(CARLA_DIR)/Build/install;/opt/carla" \
		  -DBUILD_CARLA_CLIENT=ON \
		  -DCARLA_STUDIO_FETCH_ASSIMP=ON && \
		cmake --build . -j4
	@echo ""
	@echo "[+] Build complete!"
	@echo "[+] Output: $$(pwd)/build/bin/carla-studio"
	@echo "[+] Run with: ./build/bin/carla-studio"
endif

# Integrate mode: symlink into main repo
integrate: check-carla-dir
	@echo "[*] Setting up integration with CARLA..."
	@if [ -L "$(CARLA_DIR)/Apps/CarlaStudio" ]; then \
		echo "[!] Symlink already exists at $(CARLA_DIR)/Apps/CarlaStudio"; \
		echo "[!] Removing..."; \
		rm -f "$(CARLA_DIR)/Apps/CarlaStudio"; \
	fi
	@echo "[*] Creating symlink: $(CARLA_DIR)/Apps/CarlaStudio -> $$(pwd)"
	@ln -s "$$(pwd)" "$(CARLA_DIR)/Apps/CarlaStudio"
	@echo "[+] Integration complete!"
	@echo ""
	@echo "[*] Next steps:"
	@echo "    cd $(CARLA_DIR)"
	@echo "    mkdir -p Build && cd Build"
	@echo "    cmake .. -DBUILD_CARLA_STUDIO=ON -DCMAKE_BUILD_TYPE=Release"
	@echo "    cmake --build . --target carla-studio -j4"
	@echo ""

# Clean build artifacts
clean:
	@echo "[*] Cleaning build artifacts..."
	@rm -rf build
	@echo "[+] Cleaned"
