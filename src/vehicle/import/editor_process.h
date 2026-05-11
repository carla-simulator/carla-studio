// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QString>
#include <QtGlobal>

namespace carla_studio::vehicle_import {

struct EditorProcessInfo {
  qint64 pid           = 0;
  bool   exists        = false;
  bool   is_headless    = false;
  qint64 start_unix_time = 0;
};

EditorProcessInfo find_editor_for_uproject(const QString &uproject_path);

qint64 plugin_so_mtime_unix(const QString &uproject_path);

bool   editor_is_stale(const EditorProcessInfo &info,
                     const QString          &uproject_path);

bool   kill_editor(qint64 pid, int timeout_ms = 5000);

}
