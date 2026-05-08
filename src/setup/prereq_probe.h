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

namespace carla_studio::setup_wizard {

struct PrereqResult {
  QString tool;
  QString detail;
  bool    ok = false;
};

PrereqResult probe_git();
PrereqResult probe_git_lfs();
PrereqResult probe_ninja();
PrereqResult probe_cmake();
QString      run_shell(const QString &cmd, int timeout_ms = 1500);
bool         path_has_unreal_editor(const QString &ue_path);

}
