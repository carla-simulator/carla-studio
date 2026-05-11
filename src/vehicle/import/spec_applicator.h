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
#include <QStringList>

namespace carla_studio::vehicle_import {

struct ApplyResult {
  bool        ok = false;
  QString     bp_asset_path;
  QStringList persisted_assets;
  QStringList warnings;
  QString     detail;
};

struct ValidationReport {
  bool   ok = false;
  bool   chassis_overlaps_ground = false;
  bool   wheels_touch_ground = false;
  bool   bp_class_resolves   = false;
  bool   collision_sensor_fires = false;
  QString detail;
};

ApplyResult apply_spec_to_editor(const VehicleSpec &spec, const QString &editor_rpc_endpoint);

ValidationReport validate_imported_vehicle(const QString &bp_asset_path,
                                         const QString &editor_rpc_endpoint);

}
