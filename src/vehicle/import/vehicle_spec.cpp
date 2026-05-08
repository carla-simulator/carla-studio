// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/vehicle_spec.h"

#include <QJsonArray>

namespace carla_studio::vehicle_import {

SizePreset preset_for_size_class(SizeClass s) {
  SizePreset p;
  switch (s) {
    case SizeClass::TinyRobot:
      p.mass = 50.0f;     p.max_steer_angle = 35.0f; p.max_brake_torque = 200.0f;
      p.susp_max_raise = 4.0f;  p.susp_max_drop = 4.0f;  p.susp_damping = 0.45f;
      p.torque_curve_tag = QStringLiteral("tiny-robot"); break;
    case SizeClass::Compact:
      p.mass = 1300.0f;   p.max_steer_angle = 65.0f; p.max_brake_torque = 1100.0f;
      p.susp_max_raise = 8.0f;  p.susp_max_drop = 8.0f;  p.susp_damping = 0.55f;
      p.torque_curve_tag = QStringLiteral("compact"); break;
    case SizeClass::Sedan:
      p.mass = 1600.0f;   p.max_steer_angle = 70.0f; p.max_brake_torque = 1500.0f;
      p.susp_max_raise = 10.0f; p.susp_max_drop = 10.0f; p.susp_damping = 0.65f;
      p.torque_curve_tag = QStringLiteral("sedan"); break;
    case SizeClass::Suv:
      p.mass = 2400.0f;   p.max_steer_angle = 70.0f; p.max_brake_torque = 2000.0f;
      p.susp_max_raise = 12.0f; p.susp_max_drop = 12.0f; p.susp_damping = 0.70f;
      p.torque_curve_tag = QStringLiteral("suv"); break;
    case SizeClass::Truck:
      p.mass = 3000.0f;   p.max_steer_angle = 45.0f; p.max_brake_torque = 4000.0f;
      p.susp_max_raise = 15.0f; p.susp_max_drop = 15.0f; p.susp_damping = 0.75f;
      p.torque_curve_tag = QStringLiteral("truck"); break;
    case SizeClass::Bus:
      p.mass = 12000.0f;  p.max_steer_angle = 40.0f; p.max_brake_torque = 6500.0f;
      p.susp_max_raise = 18.0f; p.susp_max_drop = 18.0f; p.susp_damping = 0.78f;
      p.torque_curve_tag = QStringLiteral("bus"); break;
    default: break;
  }
  return p;
}

VehicleSpec build_spec_from_analysis(const MeshAnalysisResult &r,
                                  const QString &name,
                                  const QString &mesh_path,
                                  float scale_to_cm,
                                  int   up_axis,
                                  int   forward_axis) {
  VehicleSpec s;
  s.name        = name;
  s.mesh_path    = mesh_path;
  s.scale_to_cm   = scale_to_cm;
  s.up_axis      = up_axis;
  s.forward_axis = forward_axis;
  s.size_class   = r.size_class;
  const SizePreset p = preset_for_size_class(r.size_class);
  s.mass             = p.mass;
  s.susp_damping      = p.susp_damping;
  s.torque_curve_tag   = p.torque_curve_tag;
  const float xExt = r.chassis_x_max - r.chassis_x_min;
  const float yExt = r.chassis_y_max - r.chassis_y_min;
  const bool swap_xy = (yExt > xExt * 1.25f);
  s.adjust_yaw_deg = swap_xy ? 90.0f : 0.0f;

  s.chassis_x_min = swap_xy ? r.chassis_y_min : r.chassis_x_min;
  s.chassis_x_max = swap_xy ? r.chassis_y_max : r.chassis_x_max;
  s.chassis_y_min = swap_xy ? r.chassis_x_min : r.chassis_y_min;
  s.chassis_y_max = swap_xy ? r.chassis_x_max : r.chassis_y_max;
  s.chassis_z_min = r.chassis_z_min; s.chassis_z_max = r.chassis_z_max;

  s.overall_x_min = swap_xy ? r.overall_y_min : r.overall_x_min;
  s.overall_x_max = swap_xy ? r.overall_y_max : r.overall_x_max;
  s.overall_y_min = swap_xy ? r.overall_x_min : r.overall_y_min;
  s.overall_y_max = swap_xy ? r.overall_x_max : r.overall_y_max;
  s.overall_z_min = r.overall_z_min; s.overall_z_max = r.overall_z_max;
  s.small_vehicle_needs_wheel_shape = r.small_vehicle_needs_wheel_shape;
  s.has_four_wheels    = r.has_four_wheels;
  s.detected_from_mesh = r.ok;
  if (r.has_four_wheels) {
    for (size_t i = 0; i < 4 && i < r.wheels.size(); ++i) {
      const WheelCandidate &w = r.wheels[i];
      WheelSpec &ws = s.wheels[i];
      ws.x = swap_xy ? w.cy : w.cx;
      ws.y = swap_xy ? w.cx : w.cy;
      ws.z = w.cz;
      ws.radius = w.radius;
      ws.width  = w.width;
      const bool isFront = i < 2;
      ws.max_steer_angle  = isFront ? p.max_steer_angle : 0.0f;
      ws.max_brake_torque = p.max_brake_torque;
      ws.susp_max_raise   = p.susp_max_raise;
      ws.susp_max_drop    = p.susp_max_drop;
    }
  }
  return s;
}

namespace {

QJsonObject wheel_to_json(const WheelSpec &w) {
  QJsonObject o;
  o["x"] = w.x; o["y"] = w.y; o["z"] = w.z;
  o["radius"] = w.radius;
  o["width"]  = w.width;
  o["max_steer_angle"]  = w.max_steer_angle;
  o["max_brake_torque"] = w.max_brake_torque;
  o["susp_max_raise"]   = w.susp_max_raise;
  o["susp_max_drop"]    = w.susp_max_drop;
  return o;
}

}

QJsonObject spec_to_json(const VehicleSpec &s) {
  QJsonObject o;
  o["vehicle_name"]    = s.name;
  o["mesh_path"]       = s.mesh_path;
  o["content_path"]    = s.content_path;
  o["base_vehicle_bp"] = s.base_vehicle_bp;
  o["mass"]            = s.mass;
  o["susp_damping"]    = s.susp_damping;
  o["torque_curve_tag"] = s.torque_curve_tag;
  o["size_class"]      = size_class_name(s.size_class);
  o["small_vehicle_needs_wheelshape"] = s.small_vehicle_needs_wheel_shape;
  o["has_four_wheels"] = s.has_four_wheels;
  o["detected_from_mesh"] = s.detected_from_mesh;
  o["source_scale_to_cm"]  = s.scale_to_cm;
  o["source_up_axis"]      = s.up_axis;
  o["source_forward_axis"] = s.forward_axis;
  o["adjust_yaw_deg"]      = s.adjust_yaw_deg;
  o["adjust_mirror_x"]     = s.adjust_mirror_x;
  o["adjust_mirror_y"]     = s.adjust_mirror_y;
  o["wheel_fl"] = wheel_to_json(s.wheels[0]);
  o["wheel_fr"] = wheel_to_json(s.wheels[1]);
  o["wheel_rl"] = wheel_to_json(s.wheels[2]);
  o["wheel_rr"] = wheel_to_json(s.wheels[3]);
  QJsonObject chassis;
  chassis["x_min"] = s.chassis_x_min; chassis["x_max"] = s.chassis_x_max;
  chassis["y_min"] = s.chassis_y_min; chassis["y_max"] = s.chassis_y_max;
  chassis["z_min"] = s.chassis_z_min; chassis["z_max"] = s.chassis_z_max;
  o["chassis_aabb_cm"] = chassis;
  QJsonObject overall;
  overall["x_min"] = s.overall_x_min; overall["x_max"] = s.overall_x_max;
  overall["y_min"] = s.overall_y_min; overall["y_max"] = s.overall_y_max;
  overall["z_min"] = s.overall_z_min; overall["z_max"] = s.overall_z_max;
  o["overall_aabb_cm"] = overall;
  return o;
}

SpecVerification verify_chaos_vehicle_spec(const VehicleSpec &s) {
  SpecVerification v;
  constexpr float kChaosWheelFloorCm = 18.0f;
  constexpr float kWheelClearRoadCm  = 1.0f;
  constexpr float kMaxReasonableWheelCm = 80.0f;
  const float chassisX = s.chassis_x_max - s.chassis_x_min;
  const float chassisY = s.chassis_y_max - s.chassis_y_min;
  const float chassisZ = s.chassis_z_max - s.chassis_z_min;

  v.chassis_extent_positive = (chassisX > 1.0f && chassisY > 1.0f && chassisZ > 1.0f);
  if (!v.chassis_extent_positive)
    v.errors << QString("chassis extent collapsed: %1 x %2 x %3 cm")
                  .arg(chassisX).arg(chassisY).arg(chassisZ);

  v.four_wheels_present = s.has_four_wheels && s.wheels.size() == 4;
  if (!v.four_wheels_present)
    v.errors << "spec does not declare 4 wheels";

  bool allAboveFloor = true, allClearRoad = true;
  bool xSane = true;
  for (size_t i = 0; i < s.wheels.size(); ++i) {
    const WheelSpec &w = s.wheels[i];
    if (w.radius < kChaosWheelFloorCm) {
      v.warnings << QString("wheel[%1] radius %2cm < %3cm Chaos floor "
                            "(small_vehicle_needs_wheelshape should be set)")
                       .arg(i).arg(w.radius).arg(kChaosWheelFloorCm);
      allAboveFloor = false;
    }
    if (w.radius > kMaxReasonableWheelCm)
      v.warnings << QString("wheel[%1] radius %2cm exceeds %3cm sanity ceiling")
                       .arg(i).arg(w.radius).arg(kMaxReasonableWheelCm);
    if (w.z < (w.radius - kWheelClearRoadCm)) {
      v.errors << QString("wheel[%1] z=%2 below radius=%3 (chassis will sink "
                          "into road on spawn)").arg(i).arg(w.z).arg(w.radius);
      allClearRoad = false;
    }
    if (w.x < s.chassis_x_min - 5.0f || w.x > s.chassis_x_max + 5.0f) {
      xSane = false;
      v.warnings << QString("wheel[%1] x=%2 outside chassis x range [%3..%4]")
                       .arg(i).arg(w.x).arg(s.chassis_x_min).arg(s.chassis_x_max);
    }
    if (std::abs(w.y) < 5.0f)
      v.warnings << QString("wheel[%1] y=%2 near centreline (lateral too small)")
                       .arg(i).arg(w.y);
  }
  v.wheels_above_chaos_floor = allAboveFloor;
  v.wheel_anchors_clear_road = allClearRoad;
  v.wheel_anchors_x_sane     = xSane;
  v.wheel_anchors_y_sane     = true;
  for (size_t i = 0; i < s.wheels.size(); ++i)
    if (std::abs(s.wheels[i].y) < 5.0f) { v.wheel_anchors_y_sane = false; break; }

  v.ok = v.errors.isEmpty();
  return v;
}

}
