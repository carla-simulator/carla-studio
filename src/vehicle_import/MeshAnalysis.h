// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include "vehicle_import/MeshGeometry.h"

#include <QString>
#include <array>
#include <vector>

namespace carla_studio::vehicle_import {

enum class SizeClass {
  Unknown    = 0,
  TinyRobot  = 1,
  Compact    = 2,
  Sedan      = 3,
  Suv        = 4,
  Truck      = 5,
  Bus        = 6,
};

struct WheelCandidate {
  float cx = 0, cy = 0, cz = 0;
  float radius = 0;
  float width  = 0;
  int   clusterId = -1;
};

struct Cluster {
  int   firstVertex = 0;
  int   vertexCount = 0;
  float xMin = 0, xMax = 0, yMin = 0, yMax = 0, zMin = 0, zMax = 0;
};

struct MeshAnalysisResult {
  bool                   ok = false;
  std::vector<Cluster>   clusters;
  std::vector<WheelCandidate> wheels;
  bool      hasFourWheels = false;
  SizeClass sizeClass     = SizeClass::Unknown;
  float chassisXMin = 0, chassisXMax = 0;
  float chassisYMin = 0, chassisYMax = 0;
  float chassisZMin = 0, chassisZMax = 0;
  float overallXMin = 0, overallXMax = 0;
  float overallYMin = 0, overallYMax = 0;
  float overallZMin = 0, overallZMax = 0;
  bool        smallVehicleNeedsWheelShape = false;
};

MeshAnalysisResult analyzeMesh(const MeshGeometry &g, float scaleToCm);

QString sizeClassName(SizeClass s);

}  // namespace carla_studio::vehicle_import
