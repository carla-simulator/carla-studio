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
  QString uproject_path;
  QString editor_binary;
  QString content_subpath;
  QString vehicle_name;
  int     timeout_ms = 600000;
};

struct CookResult {
  bool        ok = false;
  QString     cooked_root;
  QStringList cooked_files;
  QString     stdout_tail;
  QString     stderr_tail;
  QString     detail;
};

CookResult cook_vehicle_assets(const CookRequest &req);

QString resolve_cooked_root(const QString &uproject_path, const QString &content_subpath);

}
