// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include "vehicle/import/vehicle_spec.h"

#include <QString>
#include <array>

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
  QString output_path;
  QString reason;
  int     body_vertex_count = 0;
  int     tire_vertex_count = 0;
  int     merged_vertex_count = 0;
  int     tire_source_count = 0;
  int     tire_copies_placed = 0;
};

WheelTemplate wheel_template_from_analysis(const MeshAnalysisResult &r);

WheelTemplate wheel_template_from_tires_obj(const QString &tires_obj_path,
                                        float scale_to_cm = 1.0f);

struct IpgHeader {
  bool   has_scale = false;
  double sx = 1.0, sy = 1.0, sz = 1.0;
  bool   has_wheels = false;
  std::array<double, 12> wheels { 0,0,0, 0,0,0, 0,0,0, 0,0,0 };
};

IpgHeader parse_ipg_header(const QString &obj_path);

MergeResult merge_body_and_wheels(const VehicleSpec &body_spec,
                               const WheelTemplate &wheels_tpl,
                               bool wheels_file_provided);

ObjMergeResult merge_body_and_tires(const QString &body_obj_path,
                                 const QString &tires_obj_path,
                                 const QString &out_dir,
                                 const std::array<WheelSpec, 4> &wheel_anchors);

}
