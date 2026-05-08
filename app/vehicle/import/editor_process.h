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
  bool   isHeadless    = false;
  qint64 startUnixTime = 0;
};

EditorProcessInfo findEditorForUproject(const QString &uprojectPath);

qint64 pluginSoMtimeUnix(const QString &uprojectPath);

bool   editorIsStale(const EditorProcessInfo &info,
                     const QString          &uprojectPath);

bool   killEditor(qint64 pid, int timeoutMs = 5000);

}  // namespace carla_studio::vehicle_import
