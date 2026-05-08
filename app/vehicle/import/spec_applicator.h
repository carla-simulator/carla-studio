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
  QString     bpAssetPath;
  QStringList persistedAssets;
  QStringList warnings;
  QString     detail;
};

struct ValidationReport {
  bool   ok = false;
  bool   chassisOverlapsGround = false;
  bool   wheelsTouchGround = false;
  bool   bpClassResolves   = false;
  bool   collisionSensorFires = false;
  QString detail;
};

ApplyResult applySpecToEditor(const VehicleSpec &spec, const QString &editorRpcEndpoint);

ValidationReport validateImportedVehicle(const QString &bpAssetPath,
                                         const QString &editorRpcEndpoint);

}  // namespace carla_studio::vehicle_import
