// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QJsonObject>
#include <QString>

namespace carla_studio::vehicle_import {

inline constexpr int kImporterPort = 18583;
int importer_port();

bool       probe_importer_port();
QString    send_json(const QJsonObject &spec);
QJsonObject build_spawn_spec(const QString &asset_path,
                          double x = 0.0, double y = 0.0, double z = 100.0,
                          double yaw = 0.0);

}
