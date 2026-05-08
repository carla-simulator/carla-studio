// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include "vehicle/import/mesh_analysis.h"
#include "vehicle/import/vehicle_spec.h"

#include <QString>

#include <array>
#include <functional>

namespace carla_studio::vehicle_import {



struct ImportPipelineKnobs {
  float mass             = 1500.f;
  float max_steer_angle    = 70.f;
  float max_brake_torque   = 1500.f;
  float susp_max_raise     = 10.f;
  float susp_max_drop      = 10.f;
  float susp_damping      = 0.65f;
  SizeClass size_class    = SizeClass::Sedan;
};

struct ImportPipelineInput {
  QString mesh_path;
  QString tires_path;
  QString vehicle_name;
  ImportPipelineKnobs knobs;

  QString carla_src_root;
  QString carla_shipping_root;
  QString editor_binary;
  QString uproject_path;

  int     editor_port = 18584;
  int     carla_rpc_port = 3000;

  bool    run_drive       = false;
  bool    visible_carla   = true;
  bool    manual_drive    = false;
  int     auto_drive_seconds = 60;

  bool    zip_cooked_output = false;
  QString output_dir;

  QString workspace_dir;
};

struct ImportPipelineResult {
  bool    imported = false;
  bool    deployed = false;
  bool    spawned  = false;
  QString asset_path;
  QString cooked_tar_gz;
  QString error_detail;
};









struct ImportPipelineCallbacks {
  std::function<void(const QString &)>            log;
  std::function<bool(const QString &question)>    ask_yes_no;
  std::function<void(ImportPipelineKnobs &k)>     edit_knobs;
  std::function<void(const QString &mesh_path)>    open_calibration;
  std::function<bool()>                           confirm_start_import;
};




ImportPipelineResult run_import_pipeline(const ImportPipelineInput &in,
                                       const ImportPipelineCallbacks &cb);









void open_calibration_window(const QString &mesh_path,
                           class QWidget *parent = nullptr);




void open_calibration_window_with_wheels(const QString &mesh_path,
                                     const std::array<float, 12> &wheels_cm_xyz,
                                     class QWidget *parent = nullptr);

}
