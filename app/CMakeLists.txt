if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  cmake_minimum_required (VERSION 3.16)
  project (carla_studio_app LANGUAGES CXX)
  get_filename_component (CARLA_WORKSPACE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
endif ()

# -----------------------------------------------------------------------------
# CARLA Studio versioning (semantic - bump per the rules in CHANGELOG.md):
#   MAJOR (X.0.0) for incompatible API / breaking overhauls
#   MINOR (0.Y.0) for backward-compatible new features
#   PATCH (0.0.Z) for backward-compatible bug fixes
# Version is exported as a compile-time #define so the GUI can show it in the
# top-right of the menu bar without needing a separate header. The value here
# is the source of truth - bump in lockstep with the matching CHANGELOG.md
# entry sitting next to this file.
# -----------------------------------------------------------------------------
set (CARLA_STUDIO_VERSION "1.1.0")

if (NOT COMMAND carla_message)
  function (carla_message msg)
    message (STATUS "CARLA: ${msg}")
  endfunction ()
endif ()

if (NOT COMMAND carla_warning)
  function (carla_warning msg)
    message (WARNING "${msg}")
  endfunction ()
endif ()

if (NOT COMMAND carla_add_executable)
  function (carla_add_executable target _description)
    add_executable (${target} ${ARGN})
  endfunction ()
endif ()

option (CARLA_STUDIO_FETCH_ASSIMP
        "Fetch + build Assimp at configure time for multi-format mesh ingest in CarlaStudio."
        OFF)

if (CARLA_STUDIO_FETCH_ASSIMP)
  include (FetchContent)
  set (ASSIMP_BUILD_TESTS         OFF CACHE BOOL "" FORCE)
  set (ASSIMP_BUILD_ASSIMP_TOOLS  OFF CACHE BOOL "" FORCE)
  set (ASSIMP_INSTALL             OFF CACHE BOOL "" FORCE)
  set (ASSIMP_BUILD_ZLIB          ON  CACHE BOOL "" FORCE)
  set (BUILD_SHARED_LIBS          OFF CACHE BOOL "" FORCE)
  FetchContent_Declare (
    assimp
    GIT_REPOSITORY https://github.com/assimp/assimp.git
    GIT_TAG        v5.4.3
    GIT_SHALLOW    TRUE
  )
  FetchContent_MakeAvailable (assimp)
  carla_message ("CARLA Studio: Assimp fetched and configured (multi-format mesh ingest enabled).")
endif ()

set (CARLA_STUDIO_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set (CARLA_STUDIO_RESOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/icons")
set (CARLA_STUDIO_APP_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/carla_studio.cpp")
set (CARLA_STUDIO_APP_TEST_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/tests/carla_studio_app_test.cpp")
set (CARLA_STUDIO_QRC "${CMAKE_CURRENT_SOURCE_DIR}/icons/carla-studio.qrc")

find_package (Qt6 COMPONENTS Widgets Core Concurrent Network QUIET)
if (Qt6_FOUND)
  set (CARLA_STUDIO_QT_VERSION 6)
  set (CARLA_STUDIO_QT_WIDGETS_TARGET Qt6::Widgets)
  set (CARLA_STUDIO_QT_CORE_TARGET Qt6::Core)
  set (CARLA_STUDIO_QT_CONCURRENT_TARGET Qt6::Concurrent)
  set (CARLA_STUDIO_QT_NETWORK_TARGET Qt6::Network)
  set (CARLA_STUDIO_QT_GUI_TARGET Qt6::Gui)
  find_package (Qt6 COMPONENTS Gamepad Test QUIET)
  find_package (Qt6 COMPONENTS 3DCore 3DRender 3DInput 3DExtras QUIET)
  if (TARGET Qt6::3DCore AND TARGET Qt6::3DRender
      AND TARGET Qt6::3DInput AND TARGET Qt6::3DExtras)
    set (CARLA_STUDIO_QT3D_AVAILABLE TRUE)
    set (CARLA_STUDIO_QT_3D_TARGETS
      Qt6::3DCore Qt6::3DRender Qt6::3DInput Qt6::3DExtras)
  endif ()
else ()
  find_package (Qt5 COMPONENTS Widgets Core Concurrent Network QUIET)
  if (NOT Qt5_FOUND)
    message (WARNING
      "CARLA Studio: Qt6 (Widgets/Core/Concurrent/Network) and Qt5 (Widgets/Core/Concurrent/Network) "
      "were both not found. Skipping CarlaStudio build. Install Qt to enable.")
    return ()
  endif ()
  set (CARLA_STUDIO_QT_VERSION 5)
  set (CARLA_STUDIO_QT_WIDGETS_TARGET Qt5::Widgets)
  set (CARLA_STUDIO_QT_CORE_TARGET Qt5::Core)
  set (CARLA_STUDIO_QT_CONCURRENT_TARGET Qt5::Concurrent)
  set (CARLA_STUDIO_QT_NETWORK_TARGET Qt5::Network)
  set (CARLA_STUDIO_QT_GUI_TARGET Qt5::Gui)
  find_package (Qt5 COMPONENTS Gamepad Test QUIET)
  find_package (Qt5 COMPONENTS 3DCore 3DRender 3DInput 3DExtras QUIET)
  if (TARGET Qt5::3DCore AND TARGET Qt5::3DRender
      AND TARGET Qt5::3DInput AND TARGET Qt5::3DExtras)
    set (CARLA_STUDIO_QT3D_AVAILABLE TRUE)
    set (CARLA_STUDIO_QT_3D_TARGETS
      Qt5::3DCore Qt5::3DRender Qt5::3DInput Qt5::3DExtras)
  endif ()
endif ()

carla_message ("CARLA Building CarlaStudio application (top-layer app module)")

add_library (
  carla-studio-core STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/core/studio_app_context.cpp
)

target_include_directories (
  carla-studio-core PUBLIC
  ${CARLA_STUDIO_SOURCE_DIR}
)

target_link_libraries (
  carla-studio-core PUBLIC
  ${CARLA_STUDIO_QT_CORE_TARGET}
)

# Cross-cutting helpers: hardware probes (ResourceFit), storage volume
# enumeration, license/resource extraction. Pure-Qt, no LibCarla coupling,
# no external-system glue - that lives under integrations/ instead.
add_library (
  carla-studio-utils STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/utils/resource_fit.cpp
)

target_include_directories (
  carla-studio-utils PUBLIC
  ${CARLA_STUDIO_SOURCE_DIR}
)

target_link_libraries (
  carla-studio-utils PUBLIC
  ${CARLA_STUDIO_QT_CORE_TARGET}
)

# External-system glue: Hugging Face auth/API, Brev CLI wrapper, uv env
# manager, third-party tool registry, JSON-driven launch dispatcher. Each
# module here knows about ONE external system and stays narrowly scoped.
# Generic helpers belong in carla-studio-utils.
add_library (
  carla-studio-integ STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/integrations/hf/hugging_face.cpp
)

target_include_directories (
  carla-studio-integ PUBLIC
  ${CARLA_STUDIO_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/integrations/hf
)

target_link_libraries (
  carla-studio-integ PUBLIC
  ${CARLA_STUDIO_QT_CORE_TARGET}
  carla-studio-utils
)

add_library (
  carla-studio-setup-wizard STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/setup/prereq_probe.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/setup/setup_wizard_dialog.cpp
)

set_target_properties (
  carla-studio-setup-wizard PROPERTIES
  AUTOMOC ON
)

target_include_directories (
  carla-studio-setup-wizard PUBLIC
  ${CARLA_STUDIO_SOURCE_DIR}
)

target_link_libraries (
  carla-studio-setup-wizard PUBLIC
  ${CARLA_STUDIO_QT_WIDGETS_TARGET}
)

add_library (
  carla-studio-vehicle-import STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/import_mode.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/kit_bundler.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/merged_spec_builder.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/import_pipeline.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/canonicalizer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/cook_step.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/mesh_aabb.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/mesh_analysis.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/mesh_geometry.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/name_sanitizer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/spec_applicator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/vehicle_spec.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/obj_sanitizer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/importer_client.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/editor_process.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/bp_autopicker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/vehicle_import_page.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/vehicle_import_container.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/vehicle_preview_page.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/vehicle_preview_window.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/view_gizmo.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/mesh_preview_renderer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/carla_spawn.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/plugin_setup.cpp
)

set_target_properties (
  carla-studio-vehicle-import PROPERTIES
  AUTOMOC ON
)

target_include_directories (
  carla-studio-vehicle-import PUBLIC
  ${CARLA_STUDIO_SOURCE_DIR}
)

target_link_libraries (
  carla-studio-vehicle-import PUBLIC
  ${CARLA_STUDIO_QT_WIDGETS_TARGET}
  ${CARLA_STUDIO_QT_CONCURRENT_TARGET}
  ${CARLA_STUDIO_QT_NETWORK_TARGET}
)

# Qt3D for the in-Studio vehicle preview viewer (VehiclePreviewPage). The
# viewer renders the source mesh on a calibration grid before any Unreal
# import - orientation, scale, and wheel-marker checks happen here.
if (CARLA_STUDIO_QT_VERSION EQUAL 6)
  find_package (Qt6 COMPONENTS 3DCore 3DRender 3DInput 3DExtras QUIET)
  if (TARGET Qt6::3DCore AND TARGET Qt6::3DRender AND TARGET Qt6::3DInput AND TARGET Qt6::3DExtras)
    target_link_libraries (carla-studio-vehicle-import PUBLIC
      Qt6::3DCore Qt6::3DRender Qt6::3DInput Qt6::3DExtras)
    target_compile_definitions (carla-studio-vehicle-import PUBLIC CARLA_STUDIO_WITH_QT3D)
    carla_message ("Qt6 3D modules linked; vehicle preview Qt3D viewer enabled.")
  else ()
    carla_message ("Qt6 3D modules not found; vehicle preview viewer will be a stub.")
  endif ()
else ()
  carla_message ("Qt5 path: vehicle preview viewer not built (Qt5 3D not wired).")
endif ()

if (TARGET carla-client)
  target_link_libraries (carla-studio-vehicle-import PUBLIC carla-client)
  target_compile_definitions (carla-studio-vehicle-import PUBLIC CARLA_STUDIO_WITH_LIBCARLA)
endif ()

if (CARLA_STUDIO_FETCH_ASSIMP AND TARGET assimp)
  target_link_libraries (carla-studio-vehicle-import PUBLIC assimp)
  target_compile_definitions (carla-studio-vehicle-import PUBLIC CARLA_STUDIO_WITH_ASSIMP)
endif ()

# Platform-specific icon resources that live alongside the executable sources.
set (CARLA_STUDIO_PLATFORM_SOURCES "")

if (WIN32)
  # Windows: compile the .rc referencing carla-studio.ico into the executable.
  set (CARLA_STUDIO_WIN32_RC "${CARLA_STUDIO_RESOURCE_DIR}/windows/carla-studio.rc")
  list (APPEND CARLA_STUDIO_PLATFORM_SOURCES ${CARLA_STUDIO_WIN32_RC})
endif ()

if (APPLE)
  # macOS: bundle the .icns inside the .app; set_source_files_properties
  # routes the file to Contents/Resources and MACOSX_BUNDLE_ICON_FILE gives it
  # to the bundle's Info.plist.
  set (CARLA_STUDIO_MACOS_ICNS "${CARLA_STUDIO_RESOURCE_DIR}/macos/carla-studio.icns")
  set_source_files_properties (
    ${CARLA_STUDIO_MACOS_ICNS} PROPERTIES
    MACOSX_PACKAGE_LOCATION "Resources"
  )
  list (APPEND CARLA_STUDIO_PLATFORM_SOURCES ${CARLA_STUDIO_MACOS_ICNS})
endif ()

set (CARLA_STUDIO_CALIBRATION_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/calibration/calibration_scene.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/calibration/calibration_scene.h
)
if (NOT CARLA_STUDIO_QT3D_AVAILABLE)
  set (CARLA_STUDIO_CALIBRATION_SOURCES "")
endif ()

carla_add_executable (
  carla-studio
  "Build the CARLA Studio main application."
  WIN32            # MACOSX_BUNDLE is applied via property below (keeps CMake < 3.16 compat path consistent)
  ${CARLA_STUDIO_APP_SOURCE}
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/import_test_cli.cpp
  ${CARLA_STUDIO_QRC}
  ${CARLA_STUDIO_PLATFORM_SOURCES}
  ${CARLA_STUDIO_CALIBRATION_SOURCES}
)

set_target_properties (
  carla-studio PROPERTIES
  AUTOMOC ON
  AUTORCC ON
)

if (APPLE)
  set_target_properties (
    carla-studio PROPERTIES
    MACOSX_BUNDLE TRUE
    MACOSX_BUNDLE_BUNDLE_NAME "CARLA Studio"
    MACOSX_BUNDLE_GUI_IDENTIFIER "org.carla-simulator.CarlaStudio"
    MACOSX_BUNDLE_ICON_FILE "carla-studio.icns"
  )
endif ()

target_link_libraries (
  carla-studio PUBLIC
  ${CARLA_STUDIO_QT_WIDGETS_TARGET}
  carla-studio-core
  carla-studio-utils
  carla-studio-integ
  carla-studio-vehicle-import
  carla-studio-setup-wizard
)

# Export version as a compile-time string. Quoted-stringify so the source can
# use it directly as a string literal (`CARLA_STUDIO_VERSION_STRING`).
target_compile_definitions (
  carla-studio PRIVATE
  "CARLA_STUDIO_VERSION_STRING=\"${CARLA_STUDIO_VERSION}\""
)

# Calibration CLI: pre-import inspector. Loads a vehicle mesh, renders top /
# side / front PNGs on a 1m calibration grid, prints a JSON summary. Same
# rendering core as the Studio Preview tab. Used from the terminal for
# regression-free orientation and size-class checks before touching Unreal.
carla_add_executable (
  carla-studio-vehicle-preview
  "Calibration CLI for vehicle mesh inspection."
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/calibration_cli.cpp
)
target_link_libraries (
  carla-studio-vehicle-preview PRIVATE
  carla-studio-vehicle-import
  ${CARLA_STUDIO_QT_CORE_TARGET}
  ${CARLA_STUDIO_QT_WIDGETS_TARGET}
)


# Standalone cleanup utility - finds and SIGKILLs stale CARLA Studio /
# UnrealEditor / CarlaUnreal-Linux-Shipping / CarlaUnreal.sh processes
# left behind by a crash or interrupted run. Uses pgrep extended-regex
# patterns; idempotent (exit 0 even if nothing matched).
carla_add_executable (
  carla-studio-cli_cleanup
  "Standalone cleanup CLI - kills stale CARLA Studio / UE / shipping processes."
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/cleanup_cli.cpp
)
target_link_libraries (
  carla-studio-cli_cleanup PRIVATE
  ${CARLA_STUDIO_QT_CORE_TARGET}
)


if (CARLA_STUDIO_QT_VERSION EQUAL 6)
  if (TARGET Qt6::Gamepad)
    target_link_libraries (carla-studio PUBLIC Qt6::Gamepad)
    target_compile_definitions (carla-studio PRIVATE CARLA_QT_GAMEPAD_ENABLED)
  endif ()
else ()
  if (TARGET Qt5::Gamepad)
    target_link_libraries (carla-studio PUBLIC Qt5::Gamepad)
    target_compile_definitions (carla-studio PRIVATE CARLA_QT_GAMEPAD_ENABLED)
  endif ()
endif ()

if (TARGET carla-client)
  target_link_libraries (carla-studio PUBLIC carla-client)
  target_compile_definitions (carla-studio PRIVATE CARLA_STUDIO_WITH_LIBCARLA)
else ()
  carla_warning ("carla-client target is unavailable; configure from the CARLA workspace root with BUILD_CARLA_CLIENT=ON and BUILD_CARLA_STUDIO=ON for full LibCarla integration.")
endif ()

if (CARLA_STUDIO_QT3D_AVAILABLE)
  target_link_libraries (carla-studio PUBLIC ${CARLA_STUDIO_QT_3D_TARGETS})
  target_compile_definitions (carla-studio PRIVATE CARLA_STUDIO_WITH_QT3D)
else ()
  carla_warning ("Qt3D modules not found; Calibration View 3D scene will be disabled. Install qt6-3d-dev / libqt63dcore6-dev (or Qt5 equivalents) to enable.")
endif ()

# Linux install rules: hicolor icons + .desktop entry so the app appears in menus.
if (UNIX AND NOT APPLE)
  include (GNUInstallDirs)
  set (CARLA_STUDIO_LINUX_ICON_DIR "${CARLA_STUDIO_RESOURCE_DIR}/linux/hicolor")
  install (
    FILES "${CARLA_STUDIO_LINUX_ICON_DIR}/1024x1024/apps/carla-studio.png"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/1024x1024/apps"
    OPTIONAL
  )
  install (
    FILES "${CARLA_STUDIO_LINUX_ICON_DIR}/scalable/apps/carla-studio.svg"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps"
    OPTIONAL
  )
  install (
    FILES "${CARLA_STUDIO_RESOURCE_DIR}/linux/carla-studio.desktop"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/applications"
    OPTIONAL
  )
  install (TARGETS carla-studio RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif ()

if (APPLE)
  include (GNUInstallDirs)
  install (TARGETS carla-studio BUNDLE DESTINATION ".")
endif ()

if (WIN32)
  include (GNUInstallDirs)
  install (TARGETS carla-studio RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif ()

# License + third-party manifest. Both are bundled as Qt resources for the
# in-app "Help → License" dialog AND installed to the standard share/doc
# location so distro packagers can collect them with `dh_installdocs` or
# equivalent. The Qt resource path is the source of truth at runtime.
include (GNUInstallDirs)
install (
  FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE"
    "${CMAKE_CURRENT_SOURCE_DIR}/../docs/THIRD_PARTY.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/../CITATION.cff"
  DESTINATION "${CMAKE_INSTALL_DOCDIR}"
  OPTIONAL
)

set (CARLA_STUDIO_QT_TEST_TARGET "")
if (CARLA_STUDIO_QT_VERSION EQUAL 6)
  find_package (Qt6 COMPONENTS Test QUIET)
  if (TARGET Qt6::Test)
    set (CARLA_STUDIO_QT_TEST_TARGET Qt6::Test)
  endif ()
else ()
  find_package (Qt5 COMPONENTS Test QUIET)
  if (TARGET Qt5::Test)
    set (CARLA_STUDIO_QT_TEST_TARGET Qt5::Test)
  endif ()
endif ()
carla_message ("CARLA Studio Qt Test target: ${CARLA_STUDIO_QT_TEST_TARGET}")

include (CTest)

if (BUILD_TESTING AND UNIX)
  add_test (
    NAME carla-studio-ue4-quick-launch-kill
    COMMAND /usr/bin/env bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/quick_runtime_checks.sh ue4
  )
  add_test (
    NAME carla-studio-ue5-quick-launch-kill
    COMMAND /usr/bin/env bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/quick_runtime_checks.sh ue5
  )
  add_test (
    NAME carla-studio-thirdparty-checks
    COMMAND /usr/bin/env bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/quick_runtime_checks.sh thirdparty
  )
endif ()

if (CARLA_STUDIO_QT_TEST_TARGET)
  add_executable (
    carla-studio-tests
    ${CARLA_STUDIO_APP_TEST_SOURCE}
  )

  set_target_properties (
    carla-studio-tests PROPERTIES
    AUTOMOC ON
  )

  target_link_libraries (
    carla-studio-tests PRIVATE
    ${CARLA_STUDIO_QT_TEST_TARGET}
    ${CARLA_STUDIO_QT_WIDGETS_TARGET}
    carla-studio-core
    carla-studio-utils
    carla-studio-vehicle-import
  )

  target_compile_definitions (
    carla-studio-tests PRIVATE
    "CARLA_STUDIO_SOURCE_PATH=\"${CARLA_STUDIO_APP_SOURCE}\""
  )

  if (BUILD_TESTING)
    add_test (NAME carla-studio-tests COMMAND carla-studio-tests)
  endif ()

  add_executable (
    test-suite_carla-studio-gui
    ${CARLA_STUDIO_SOURCE_DIR}/tests/studio_gui_matrix.cpp
  )

  set_target_properties (
    test-suite_carla-studio-gui PROPERTIES
    AUTOMOC ON
  )

  target_link_libraries (
    test-suite_carla-studio-gui PRIVATE
    ${CARLA_STUDIO_QT_TEST_TARGET}
    ${CARLA_STUDIO_QT_WIDGETS_TARGET}
    ${CARLA_STUDIO_QT_NETWORK_TARGET}
    carla-studio-core
    carla-studio-utils
    carla-studio-vehicle-import
  )
  if (CARLA_STUDIO_QT3D_AVAILABLE)
    target_sources (
      test-suite_carla-studio-gui PRIVATE
      ${CARLA_STUDIO_CALIBRATION_SOURCES}
    )
    target_link_libraries (
      test-suite_carla-studio-gui PRIVATE
      ${CARLA_STUDIO_QT_3D_TARGETS}
    )
    target_compile_definitions (
      test-suite_carla-studio-gui PRIVATE
      CARLA_STUDIO_WITH_QT3D
    )
  endif ()

  target_compile_definitions (
    test-suite_carla-studio-gui PRIVATE
    "CARLA_STUDIO_SOURCE_PATH=\"${CARLA_STUDIO_APP_SOURCE}\""
    "CARLA_STUDIO_REPO_ROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/..\""
    "CARLA_STUDIO_TESTS_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/tests\""
    "CARLA_STUDIO_BINARY_PATH=\"$<TARGET_FILE:carla-studio>\""
  )

  if (BUILD_TESTING)
    add_test (NAME test-suite_carla-studio-gui
              COMMAND test-suite_carla-studio-gui)
  endif ()

  carla_add_executable (
    test-suite_carla-studio-cli
    "Test suite runner - CLI pipeline tests + GUI matrix dispatch + screenshot capture."
    ${CMAKE_CURRENT_SOURCE_DIR}/vehicle/import/run_test_suite_cli.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/cli_pipeline_test.cpp
  )
  set_target_properties (test-suite_carla-studio-cli PROPERTIES AUTOMOC ON)
  target_link_libraries (
    test-suite_carla-studio-cli PRIVATE
    ${CARLA_STUDIO_QT_GUI_TARGET}
    ${CARLA_STUDIO_QT_CORE_TARGET}
    ${CARLA_STUDIO_QT_TEST_TARGET}
    carla-studio-vehicle-import
  )
  target_compile_definitions (
    test-suite_carla-studio-cli PRIVATE
    "CARLA_STUDIO_REPO_ROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/..\""
    "CARLA_STUDIO_BINARY_PATH=\"$<TARGET_FILE:carla-studio>\""
  )
  add_dependencies (test-suite_carla-studio-cli carla-studio test-suite_carla-studio-gui)

endif ()
