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

bool pluginSoExists(const QString &uprojectPath) {
  if (uprojectPath.isEmpty()) return false;
  const QString uprojectDir = QFileInfo(uprojectPath).absolutePath();
  const QString soPath = uprojectDir + "/Plugins/CarlaTools/Binaries/Linux/libUnrealEditor-CarlaTools.so";
  return QFileInfo(soPath).isFile();
}

bool engineEditorExists(const QString &enginePath) {
  if (enginePath.isEmpty()) return false;
  const QString bin = enginePath + "/Engine/Binaries/Linux/UnrealEditor";
  return QFileInfo(bin).isFile() || QFileInfo(bin + ".exe").isFile();
}

}  // namespace

ImportModeStatus determineImportMode(const QString &uprojectPath,
                                     const QString &enginePath,
                                     bool forceLite) {
  ImportModeStatus s;
  s.uprojectPath  = uprojectPath;
  s.enginePath    = enginePath;
  s.hasUproject   = !uprojectPath.isEmpty() && QFileInfo(uprojectPath).isFile();
  s.hasEngine     = engineEditorExists(enginePath);
  s.pluginAligned = s.hasUproject && pluginSoExists(uprojectPath);
  s.forcedLite    = forceLite;
  if (forceLite) {
    s.mode   = ImportMode::Lite;
    s.reason = QStringLiteral("forced Lite via Cfg");
    return s;
  }
  if (s.hasUproject && s.hasEngine && s.pluginAligned) {
    s.mode   = ImportMode::Full;
    s.reason = QStringLiteral("CARLA source + Unreal engine + CarlaTools plugin all present");
    return s;
  }
  s.mode = ImportMode::Lite;
  QStringList missing;
  if (!s.hasUproject)   missing << "CARLA_SRC_ROOT (uproject)";
  if (!s.hasEngine)     missing << "CARLA_UNREAL_ENGINE_PATH (UnrealEditor)";
  if (!s.pluginAligned) missing << "CarlaTools plugin .so";
  s.reason = QStringLiteral("Lite mode - missing: ") + missing.join(", ");
  return s;
}

QString describeImportMode(const ImportModeStatus &s) {
  if (s.mode == ImportMode::Full)
    return QStringLiteral("Full mode - Studio will run the UE plugin to emit assets, deploy, and visualize.");
  return QStringLiteral("Lite mode - Studio will produce a vehicle kit (canonical mesh + spec.json + README) "
                        "you can finish in Unreal Editor manually. ") + s.reason + QStringLiteral(".");
}

}  // namespace carla_studio::vehicle_import
