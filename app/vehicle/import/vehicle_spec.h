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
  float maxSteerAngle    = 70.0f;
  float maxBrakeTorque   = 1500.0f;
  float suspMaxRaise     = 10.0f;
  float suspMaxDrop      = 10.0f;
  float suspDamping      = 0.65f;
  QString torqueCurveTag = QStringLiteral("sedan");
};

SizePreset presetForSizeClass(SizeClass s);

struct WheelSpec {
  float x = 0, y = 0, z = 0;
  float radius = 0;
  float width  = 0;
  float maxSteerAngle  = 0;
  float maxBrakeTorque = 0;
  float suspMaxRaise   = 0;
  float suspMaxDrop    = 0;
};

struct VehicleSpec {
  QString name;
  QString meshPath;
  QString contentPath = QStringLiteral("/Game/Carla/Static/Vehicles/4Wheeled");
  QString baseVehicleBP;

  SizeClass sizeClass = SizeClass::Unknown;
  float mass          = 0;
  float suspDamping   = 0;
  QString torqueCurveTag;

  float scaleToCm   = 1.0f;
  int   upAxis      = 2;
  int   forwardAxis = 0;

  float adjustYawDeg  = 0.0f;
  float adjustMirrorX = 1.0f;
  float adjustMirrorY = 1.0f;

  std::array<WheelSpec, 4> wheels {};

  float chassisXMin = 0, chassisXMax = 0;
  float chassisYMin = 0, chassisYMax = 0;
  float chassisZMin = 0, chassisZMax = 0;
  float overallXMin = 0, overallXMax = 0;
  float overallYMin = 0, overallYMax = 0;
  float overallZMin = 0, overallZMax = 0;

  bool smallVehicleNeedsWheelShape = false;
  bool hasFourWheels = false;
  bool detectedFromMesh = false;
};

VehicleSpec buildSpecFromAnalysis(const MeshAnalysisResult &r,
                                  const QString &name,
                                  const QString &meshPath,
                                  float scaleToCm,
                                  int   upAxis,
                                  int   forwardAxis);

QJsonObject specToJson(const VehicleSpec &s);

struct SpecVerification {
  bool        ok = true;
  QStringList warnings;
  QStringList errors;
  bool wheelsAboveChaosFloor = false;
  bool wheelAnchorsClearRoad = false;
  bool fourWheelsPresent     = false;
  bool chassisExtentPositive = false;
  bool wheelAnchorsXSane     = false;
  bool wheelAnchorsYSane     = false;
};

SpecVerification verifyChaosVehicleSpec(const VehicleSpec &s);

}  // namespace carla_studio::vehicle_import
