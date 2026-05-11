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

struct KitBundleResult {
  bool ok = false;
  QString output_dir;
  QStringList written_files;
  QString detail;
};

KitBundleResult write_vehicle_kit(const VehicleSpec &spec,
                                const QString     &output_dir);

QString render_kit_readme(const VehicleSpec &spec);

}
