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

enum class ImportMode {
  Lite = 0,
  Full = 1,
};

struct ImportModeStatus {
  ImportMode mode = ImportMode::Lite;
  bool forcedLite = false;
  bool hasUproject = false;
  bool hasEngine   = false;
  bool pluginAligned = false;
  QString uprojectPath;
  QString enginePath;
  QString reason;
};

ImportModeStatus determineImportMode(const QString &uprojectPath,
                                     const QString &enginePath,
                                     bool forceLite);

QString describeImportMode(const ImportModeStatus &s);

}  // namespace carla_studio::vehicle_import
