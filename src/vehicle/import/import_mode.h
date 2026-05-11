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
  bool forced_lite = false;
  bool has_uproject = false;
  bool has_engine   = false;
  bool plugin_aligned = false;
  QString uproject_path;
  QString engine_path;
  QString reason;
};

ImportModeStatus determine_import_mode(const QString &uproject_path,
                                     const QString &engine_path,
                                     bool force_lite);

QString describe_import_mode(const ImportModeStatus &s);

}
