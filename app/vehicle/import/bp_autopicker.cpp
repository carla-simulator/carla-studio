// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/bp_autopicker.h"

#include <QString>
#include <cmath>

namespace carla_studio::vehicle_import {

QString pickClosestBaseVehicleBP(float /*lengthCm*/) {
  return QStringLiteral("/Game/Carla/Blueprints/USDImportTemplates/BaseUSDImportVehicle");
}

}  // namespace carla_studio::vehicle_import
