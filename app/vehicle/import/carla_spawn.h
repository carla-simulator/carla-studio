// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QString>

namespace carla_studio::vehicle_import {

struct VehicleRegistration {
  QString uprojectPath;
  QString shippingCarlaRoot;
  QString bpAssetPath;
  QString make;
  QString model;
  QString editorBinary;
  int     numberOfWheels = 4;
};

struct RegisterResult {
  bool    ok = false;
  QString configFilePath;
  QString detail;
  bool    alreadyPresent = false;
};

struct DeployResult {
  bool    ok = false;
  int     filesCopied = 0;
  int     cookedFiles = 0;
  QString destDir;
  QString configFilePath;
  QString detail;
};

RegisterResult registerVehicleInJson(const VehicleRegistration &reg);
DeployResult   deployVehicleToShippingCarla(const VehicleRegistration &reg);

struct SpawnResult {
  enum class Kind { Spawned, BlueprintNotFound, NoSimulator, Failed };
  Kind    kind = Kind::Failed;
  QString detail;
};

SpawnResult spawnInRunningCarla(const QString &make, const QString &model,
                                const QString &host = QStringLiteral("localhost"),
                                int port = 2000);

struct DriveTestResult {
  bool    spawned       = false;
  bool    drivesForward = false;
  bool    bboxIsCustom  = false;
  bool    destroyClean  = false;
  float   displacementM = 0.f;
  float   bboxExtentX   = 0.f;
  float   bboxExtentY   = 0.f;
  float   bboxExtentZ   = 0.f;
  QString detail;
};

DriveTestResult driveTestVehicle(const QString &make, const QString &model,
                                  const QString &host = QStringLiteral("localhost"),
                                  int port = 2000);

}  // namespace carla_studio::vehicle_import
