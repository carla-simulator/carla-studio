// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include "vehicle/import/mesh_analysis.h"

#include <QJsonObject>
#include <QString>
#include <array>

namespace carla_studio::vehicle_import {

struct SizePreset {
  float mass             = 1500.0f;
  float max_steer_angle    = 70.0f;
  float max_brake_torque   = 1500.0f;
  float susp_max_raise     = 10.0f;
  float susp_max_drop      = 10.0f;
  float susp_damping      = 0.65f;
  QString torque_curve_tag = QStringLiteral("sedan");
};

SizePreset preset_for_size_class(SizeClass s);

struct WheelSpec {
  float x = 0, y = 0, z = 0;
  float radius = 0;
  float width  = 0;
  float max_steer_angle  = 0;
  float max_brake_torque = 0;
  float susp_max_raise   = 0;
  float susp_max_drop    = 0;
};

struct VehicleSpec {
  QString name;
  QString mesh_path;
  QString content_path = QStringLiteral("/Game/Carla/Static/Vehicles/4Wheeled");
  QString base_vehicle_bp;

  SizeClass size_class = SizeClass::Unknown;
  float mass          = 0;
  float susp_damping   = 0;
  QString torque_curve_tag;

  float scale_to_cm   = 1.0f;
  int   up_axis      = 2;
  int   forward_axis = 0;

  float adjust_yaw_deg  = 0.0f;
  float adjust_mirror_x = 1.0f;
  float adjust_mirror_y = 1.0f;

  std::array<WheelSpec, 4> wheels {};

  float chassis_x_min = 0, chassis_x_max = 0;
  float chassis_y_min = 0, chassis_y_max = 0;
  float chassis_z_min = 0, chassis_z_max = 0;
  float overall_x_min = 0, overall_x_max = 0;
  float overall_y_min = 0, overall_y_max = 0;
  float overall_z_min = 0, overall_z_max = 0;

  bool small_vehicle_needs_wheel_shape = false;
  bool has_four_wheels = false;
  bool detected_from_mesh = false;
};

VehicleSpec build_spec_from_analysis(const MeshAnalysisResult &r,
                                  const QString &name,
                                  const QString &mesh_path,
                                  float scale_to_cm,
                                  int   up_axis,
                                  int   forward_axis);

QJsonObject spec_to_json(const VehicleSpec &s);

struct SpecVerification {
  bool        ok = true;
  QStringList warnings;
  QStringList errors;
  bool wheels_above_chaos_floor = false;
  bool wheel_anchors_clear_road = false;
  bool four_wheels_present     = false;
  bool chassis_extent_positive = false;
  bool wheel_anchors_x_sane     = false;
  bool wheel_anchors_y_sane     = false;
};

SpecVerification verify_chaos_vehicle_spec(const VehicleSpec &s);

}
