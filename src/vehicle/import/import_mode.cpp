// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/import_mode.h"

#include <QFile>
#include <QFileInfo>
#include <QSettings>

namespace carla_studio::vehicle_import {

namespace {

bool pluginSoExists(const QString &uproject_path) {
  if (uproject_path.isEmpty()) return false;
  const QString uprojectDir = QFileInfo(uproject_path).absolutePath();
  const QString soPath = uprojectDir + "/Plugins/CarlaTools/Binaries/Linux/libUnrealEditor-CarlaTools.so";
  return QFileInfo(soPath).isFile();
}

bool engineEditorExists(const QString &engine_path) {
  if (engine_path.isEmpty()) return false;
  const QString bin = engine_path + "/Engine/Binaries/Linux/UnrealEditor";
  return QFileInfo(bin).isFile() || QFileInfo(bin + ".exe").isFile();
}

}

ImportModeStatus determine_import_mode(const QString &uproject_path,
                                     const QString &engine_path,
                                     bool force_lite) {
  ImportModeStatus s;
  s.uproject_path  = uproject_path;
  s.engine_path    = engine_path;
  s.has_uproject   = !uproject_path.isEmpty() && QFileInfo(uproject_path).isFile();
  s.has_engine     = engineEditorExists(engine_path);
  s.plugin_aligned = s.has_uproject && pluginSoExists(uproject_path);
  s.forced_lite    = force_lite;
  if (force_lite) {
    s.mode   = ImportMode::Lite;
    s.reason = QStringLiteral("forced Lite via Cfg");
    return s;
  }
  if (s.has_uproject && s.has_engine && s.plugin_aligned) {
    s.mode   = ImportMode::Full;
    s.reason = QStringLiteral("CARLA source + Unreal engine + CarlaTools plugin all present");
    return s;
  }
  s.mode = ImportMode::Lite;
  QStringList missing;
  if (!s.has_uproject)   missing << "CARLA_SRC_ROOT (uproject)";
  if (!s.has_engine)     missing << "CARLA_UNREAL_ENGINE_PATH (UnrealEditor)";
  if (!s.plugin_aligned) missing << "CarlaTools plugin .so";
  s.reason = QStringLiteral("Lite mode - missing: ") + missing.join(", ");
  return s;
}

QString describe_import_mode(const ImportModeStatus &s) {
  if (s.mode == ImportMode::Full)
    return QStringLiteral("Full mode - Studio will run the UE plugin to emit assets, deploy, and visualize.");
  return QStringLiteral("Lite mode - Studio will produce a vehicle kit (canonical mesh + spec.json + README) "
                        "you can finish in Unreal Editor manually. ") + s.reason + QStringLiteral(".");
}

}
