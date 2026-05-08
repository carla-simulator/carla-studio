// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QString>
#include <QStringList>

namespace carla_studio::vehicle_import {

struct CookRequest {
  QString uprojectPath;
  QString editorBinary;
  QString contentSubpath;
  QString vehicleName;
  int     timeoutMs = 600000;
};

struct CookResult {
  bool        ok = false;
  QString     cookedRoot;
  QStringList cookedFiles;
  QString     stdoutTail;
  QString     stderrTail;
  QString     detail;
};

CookResult cookVehicleAssets(const CookRequest &req);

QString resolveCookedRoot(const QString &uprojectPath, const QString &contentSubpath);

}  // namespace carla_studio::vehicle_import
