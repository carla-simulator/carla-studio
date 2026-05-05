// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include "vehicle_import/VehicleSpec.h"

#include <QString>

namespace carla_studio::vehicle_import {

enum class IngestionMode {
  Combined  = 0,
  BodyPlusTires = 1,
  TireSwap  = 2,
};

struct WheelTemplate {
  bool  valid = false;
  float radius = 0;
  float width  = 0;
};

struct MergeResult {
  bool         ok = false;
  IngestionMode mode = IngestionMode::Combined;
  VehicleSpec   spec;
  QString       reason;
};

struct ObjMergeResult {
  bool    ok = false;
  QString outputPath;
  QString reason;
  int     bodyVertexCount = 0;
  int     tireVertexCount = 0;
  int     mergedVertexCount = 0;
  int     tireSourceCount = 0;
  int     tireCopiesPlaced = 0;
};

WheelTemplate wheelTemplateFromAnalysis(const MeshAnalysisResult &r);

MergeResult mergeBodyAndWheels(const VehicleSpec &bodySpec,
                               const WheelTemplate &wheelsTpl,
                               bool wheelsFileProvided);

ObjMergeResult mergeBodyAndTires(const QString &bodyObjPath,
                                 const QString &tiresObjPath,
                                 const QString &outDir,
                                 const std::array<WheelSpec, 4> &wheelAnchors);

}  // namespace carla_studio::vehicle_import
