// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include "vehicle/import/mesh_analysis.h"
#include "vehicle/import/vehicle_spec.h"

#include <QString>

#include <array>
#include <functional>

namespace carla_studio::vehicle_import {

// Knobs the user can adjust for an import (the * fields in the GUI / the
// stdin questionnaire). Pre-fill via classifyByName + presetForSizeClass.
struct ImportPipelineKnobs {
  float mass             = 1500.f;
  float maxSteerAngle    = 70.f;
  float maxBrakeTorque   = 1500.f;
  float suspMaxRaise     = 10.f;
  float suspMaxDrop      = 10.f;
  float suspDamping      = 0.65f;
  SizeClass sizeClass    = SizeClass::Sedan;
};

struct ImportPipelineInput {
  QString meshPath;          // body OBJ/FBX/...
  QString tiresPath;         // optional separate tires OBJ
  QString vehicleName;       // sanitized identifier; if empty, derived from meshPath
  ImportPipelineKnobs knobs;

  QString carlaSrcRoot;          // CARLA_src checkout root
  QString carlaShippingRoot;     // extracted shipping CARLA root
  QString editorBinary;          // UnrealEditor binary path
  QString uprojectPath;          // CarlaUnreal.uproject

  int     editorPort = 18584;    // VehicleImporter RPC port
  int     carlaRpcPort = 3000;   // shipping CARLA RPC port

  bool    runDrive       = false;     // spawn + drive after deploy
  bool    visibleCarla   = true;      // shipping with full RHI vs -nullrhi
  bool    manualDrive    = false;     // doDir 4-direction matrix
  int     autoDriveSeconds = 60;      // SAE-L5 autopilot duration when !manualDrive

  bool    zipCookedOutput = false;    // tar.gz the cooked subdir into outputDir
  QString outputDir;                  // where to write the .tar.gz (default $CWD)

  QString workspaceDir;               // .cs_import scratch dir (default $CWD/.cs_import)
};

struct ImportPipelineResult {
  bool    imported = false;
  bool    deployed = false;
  bool    spawned  = false;
  QString assetPath;                  // /Game/.../BP_<name>
  QString cookedTarGz;                // path of the .tar.gz when zipCookedOutput
  QString errorDetail;
};

// Callback bag: same pipeline, different I/O surfaces.
//
//   - `log`       progress lines for the user
//   - `askYesNo`  blocking yes/no prompt (defaults true if no UI)
//   - `editKnobs` blocking edit pass over the knobs (GUI: dialog form;
//                 CLI: stdin questionnaire). If null, knobs are taken as-is.
//   - `openCalibration`  blocking calibration view (GUI: tab; CLI: subprocess)
//   - `confirmStartImport` final "ready to import" gate; default proceed
struct ImportPipelineCallbacks {
  std::function<void(const QString &)>            log;
  std::function<bool(const QString &question)>    askYesNo;
  std::function<void(ImportPipelineKnobs &k)>     editKnobs;
  std::function<void(const QString &meshPath)>    openCalibration;
  std::function<bool()>                           confirmStartImport;
};

// Runs the whole pipeline (preflight → analyze → merge → import → register
// → cook → deploy → optional spawn+drive → optional zip). Returns when done
// or on first failure. Identical wiring for GUI + CLI.
ImportPipelineResult runImportPipeline(const ImportPipelineInput &in,
                                       const ImportPipelineCallbacks &cb);

// Single calibration entry point used by both GUI and CLI: opens the same
// VehiclePreviewPage (Qt3D orbit camera) widget. `parent` non-null embeds it
// as a child widget; null pops it as a top-level QDialog (modal exec).
// Blocking: returns when the user closes the calibration window.
//
// In a QCoreApplication-only process (CLI without a QApplication) the function
// short-circuits to running `carla-studio-vehicle-preview --interactive` as a
// subprocess; that binary itself uses VehiclePreviewWindow → identical widget.
void openCalibrationWindow(const QString &meshPath,
                           class QWidget *parent = nullptr);

// Same as above, but renders wheel-anchor markers at the supplied (cm)
// positions so the user can verify the spec's wheel placement against
// the loaded mesh body.
void openCalibrationWindowWithWheels(const QString &meshPath,
                                     const std::array<float, 12> &wheelsCmXYZ,
                                     class QWidget *parent = nullptr);

}  // namespace carla_studio::vehicle_import
