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
  QString uproject_path;
  QString shipping_carla_root;
  QString bp_asset_path;
  QString make;
  QString model;
  QString editor_binary;
  int     number_of_wheels = 4;
};

struct RegisterResult {
  bool    ok = false;
  QString config_file_path;
  QString detail;
  bool    already_present = false;
};

struct DeployResult {
  bool    ok = false;
  int     files_copied = 0;
  int     cooked_files = 0;
  QString dest_dir;
  QString config_file_path;
  QString detail;
};

RegisterResult register_vehicle_in_json(const VehicleRegistration &reg);
DeployResult   deploy_vehicle_to_shipping_carla(const VehicleRegistration &reg);

struct SpawnResult {
  enum class Kind { Spawned, BlueprintNotFound, NoSimulator, Failed };
  Kind    kind = Kind::Failed;
  QString detail;
};

SpawnResult spawn_in_running_carla(const QString &make, const QString &model,
                                const QString &host = QStringLiteral("localhost"),
                                int port = 2000);

struct DriveTestResult {
  bool    spawned       = false;
  bool    drives_forward = false;
  bool    bbox_is_custom  = false;
  bool    destroy_clean  = false;
  float   displacement_m = 0.f;
  float   bbox_extent_x   = 0.f;
  float   bbox_extent_y   = 0.f;
  float   bbox_extent_z   = 0.f;
  QString detail;
};

DriveTestResult drive_test_vehicle(const QString &make, const QString &model,
                                  const QString &host = QStringLiteral("localhost"),
                                  int port = 2000);

}
