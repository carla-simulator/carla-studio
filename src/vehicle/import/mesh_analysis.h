// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include "vehicle/import/mesh_geometry.h"

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
  int   cluster_id = -1;
};

struct Cluster {
  int   first_vertex = 0;
  int   vertex_count = 0;
  float x_min = 0, x_max = 0, y_min = 0, y_max = 0, z_min = 0, z_max = 0;
};

struct MeshAnalysisResult {
  bool                   ok = false;
  std::vector<Cluster>   clusters;
  std::vector<WheelCandidate> wheels;
  bool      has_four_wheels = false;
  SizeClass size_class     = SizeClass::Unknown;
  float chassis_x_min = 0, chassis_x_max = 0;
  float chassis_y_min = 0, chassis_y_max = 0;
  float chassis_z_min = 0, chassis_z_max = 0;
  float overall_x_min = 0, overall_x_max = 0;
  float overall_y_min = 0, overall_y_max = 0;
  float overall_z_min = 0, overall_z_max = 0;
  bool        small_vehicle_needs_wheel_shape = false;
};

MeshAnalysisResult analyze_mesh(const MeshGeometry &g, float scale_to_cm);

QString size_class_name(SizeClass s);

SizeClass classify_by_name(const QString &filename_stem);

}
