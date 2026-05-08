// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_import/VehicleSpec.h"

#include <QJsonArray>

namespace carla_studio::vehicle_import {

SizePreset presetForSizeClass(SizeClass s) {
  SizePreset p;
  switch (s) {
    case SizeClass::TinyRobot:
      p.mass = 50.0f;     p.maxSteerAngle = 35.0f; p.maxBrakeTorque = 200.0f;
      p.suspMaxRaise = 4.0f;  p.suspMaxDrop = 4.0f;  p.suspDamping = 0.45f;
      p.torqueCurveTag = QStringLiteral("tiny-robot"); break;
    case SizeClass::Compact:
      p.mass = 950.0f;    p.maxSteerAngle = 65.0f; p.maxBrakeTorque = 1100.0f;
      p.suspMaxRaise = 8.0f;  p.suspMaxDrop = 8.0f;  p.suspDamping = 0.55f;
      p.torqueCurveTag = QStringLiteral("compact"); break;
    case SizeClass::Sedan:
      p.mass = 1500.0f;   p.maxSteerAngle = 70.0f; p.maxBrakeTorque = 1500.0f;
      p.suspMaxRaise = 10.0f; p.suspMaxDrop = 10.0f; p.suspDamping = 0.65f;
      p.torqueCurveTag = QStringLiteral("sedan"); break;
    case SizeClass::Suv:
      p.mass = 2200.0f;   p.maxSteerAngle = 70.0f; p.maxBrakeTorque = 1900.0f;
      p.suspMaxRaise = 12.0f; p.suspMaxDrop = 12.0f; p.suspDamping = 0.70f;
      p.torqueCurveTag = QStringLiteral("suv"); break;
    case SizeClass::Truck:
      p.mass = 8000.0f;   p.maxSteerAngle = 45.0f; p.maxBrakeTorque = 5000.0f;
      p.suspMaxRaise = 15.0f; p.suspMaxDrop = 15.0f; p.suspDamping = 0.75f;
      p.torqueCurveTag = QStringLiteral("truck"); break;
    case SizeClass::Bus:
      p.mass = 12000.0f;  p.maxSteerAngle = 40.0f; p.maxBrakeTorque = 6500.0f;
      p.suspMaxRaise = 18.0f; p.suspMaxDrop = 18.0f; p.suspDamping = 0.78f;
      p.torqueCurveTag = QStringLiteral("bus"); break;
    default: break;
  }
  return p;
}

VehicleSpec buildSpecFromAnalysis(const MeshAnalysisResult &r,
                                  const QString &name,
                                  const QString &meshPath,
                                  float scaleToCm,
                                  int   upAxis,
                                  int   forwardAxis) {
  VehicleSpec s;
  s.name        = name;
  s.meshPath    = meshPath;
  s.scaleToCm   = scaleToCm;
  s.upAxis      = upAxis;
  s.forwardAxis = forwardAxis;
  s.sizeClass   = r.sizeClass;
  const SizePreset p = presetForSizeClass(r.sizeClass);
  s.mass             = p.mass;
  s.suspDamping      = p.suspDamping;
  s.torqueCurveTag   = p.torqueCurveTag;
  s.chassisXMin = r.chassisXMin; s.chassisXMax = r.chassisXMax;
  s.chassisYMin = r.chassisYMin; s.chassisYMax = r.chassisYMax;
  s.chassisZMin = r.chassisZMin; s.chassisZMax = r.chassisZMax;
  s.overallXMin = r.overallXMin; s.overallXMax = r.overallXMax;
  s.overallYMin = r.overallYMin; s.overallYMax = r.overallYMax;
  s.overallZMin = r.overallZMin; s.overallZMax = r.overallZMax;
  s.smallVehicleNeedsWheelShape = r.smallVehicleNeedsWheelShape;
  s.hasFourWheels    = r.hasFourWheels;
  s.detectedFromMesh = r.ok;
  if (r.hasFourWheels) {
    for (int i = 0; i < 4 && i < static_cast<int>(r.wheels.size()); ++i) {
      const WheelCandidate &w = r.wheels[i];
      WheelSpec &ws = s.wheels[i];
      ws.x = w.cx; ws.y = w.cy; ws.z = w.cz;
      ws.radius = w.radius;
      ws.width  = w.width;
      const bool isFront = i < 2;
      ws.maxSteerAngle  = isFront ? p.maxSteerAngle : 0.0f;
      ws.maxBrakeTorque = p.maxBrakeTorque;
      ws.suspMaxRaise   = p.suspMaxRaise;
      ws.suspMaxDrop    = p.suspMaxDrop;
    }
  }
  return s;
}

namespace {

QJsonObject wheelToJson(const WheelSpec &w) {
  QJsonObject o;
  o["x"] = w.x; o["y"] = w.y; o["z"] = w.z;
  o["radius"] = w.radius;
  o["width"]  = w.width;
  o["max_steer_angle"]  = w.maxSteerAngle;
  o["max_brake_torque"] = w.maxBrakeTorque;
  o["susp_max_raise"]   = w.suspMaxRaise;
  o["susp_max_drop"]    = w.suspMaxDrop;
  return o;
}

}  // namespace

QJsonObject specToJson(const VehicleSpec &s) {
  QJsonObject o;
  o["vehicle_name"]    = s.name;
  o["mesh_path"]       = s.meshPath;
  o["content_path"]    = s.contentPath;
  o["base_vehicle_bp"] = s.baseVehicleBP;
  o["mass"]            = s.mass;
  o["susp_damping"]    = s.suspDamping;
  o["torque_curve_tag"] = s.torqueCurveTag;
  o["size_class"]      = sizeClassName(s.sizeClass);
  o["small_vehicle_needs_wheelshape"] = s.smallVehicleNeedsWheelShape;
  o["has_four_wheels"] = s.hasFourWheels;
  o["detected_from_mesh"] = s.detectedFromMesh;
  o["source_scale_to_cm"]  = s.scaleToCm;
  o["source_up_axis"]      = s.upAxis;
  o["source_forward_axis"] = s.forwardAxis;
  o["adjust_yaw_deg"]      = s.adjustYawDeg;
  o["adjust_mirror_x"]     = s.adjustMirrorX;
  o["adjust_mirror_y"]     = s.adjustMirrorY;
  o["wheel_fl"] = wheelToJson(s.wheels[0]);
  o["wheel_fr"] = wheelToJson(s.wheels[1]);
  o["wheel_rl"] = wheelToJson(s.wheels[2]);
  o["wheel_rr"] = wheelToJson(s.wheels[3]);
  QJsonObject chassis;
  chassis["x_min"] = s.chassisXMin; chassis["x_max"] = s.chassisXMax;
  chassis["y_min"] = s.chassisYMin; chassis["y_max"] = s.chassisYMax;
  chassis["z_min"] = s.chassisZMin; chassis["z_max"] = s.chassisZMax;
  o["chassis_aabb_cm"] = chassis;
  QJsonObject overall;
  overall["x_min"] = s.overallXMin; overall["x_max"] = s.overallXMax;
  overall["y_min"] = s.overallYMin; overall["y_max"] = s.overallYMax;
  overall["z_min"] = s.overallZMin; overall["z_max"] = s.overallZMax;
  o["overall_aabb_cm"] = overall;
  return o;
}

}  // namespace carla_studio::vehicle_import
