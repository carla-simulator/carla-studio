// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/plugin_setup.h"

#include <QByteArray>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>

namespace carla_studio::vehicle_import {

namespace {

QByteArray readResource(const QString &alias) {
  QFile rf(":/plugins/" + alias);
  if (!rf.open(QIODevice::ReadOnly)) return {};
  return rf.readAll();
}

QByteArray sha(const QByteArray &b) {
  return QCryptographicHash::hash(b, QCryptographicHash::Sha256).toHex();
}

bool writeIfDifferent(const QString &targetPath, const QByteArray &content,
                      bool *wroteOut, const PreflightLogSink &log) {
  QDir().mkpath(QFileInfo(targetPath).absolutePath());
  QFile cur(targetPath);
  if (cur.open(QIODevice::ReadOnly) && sha(cur.readAll()) == sha(content)) {
    cur.close();
    return true;
  }
  cur.close();
  QFile w(targetPath);
  if (!w.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    if (log) log(QString("    write failed: %1").arg(targetPath));
    return false;
  }
  if (w.write(content) != content.size()) return false;
  w.close();
  if (wroteOut) *wroteOut = true;
  if (log) log(QString("    wrote %1").arg(targetPath));
  return true;
}

bool soHasListener(const QString &soPath) {
  if (!QFileInfo(soPath).isFile()) return false;
  QProcess p;
  p.start("nm", { "-D", soPath });
  if (!p.waitForFinished(15000)) return false;
  return p.readAllStandardOutput().contains("UVehicleImporter11StartServer");
}

QString pluginDir(const QString &uproject) {
  return QFileInfo(uproject).absolutePath() + "/Plugins/CarlaTools";
}

QString pluginSoPath(const QString &uproject) {
  return pluginDir(uproject) + "/Binaries/Linux/libUnrealEditor-CarlaTools.so";
}

QString manifestBuildId(const QString &manifestPath) {
  QFile f(manifestPath);
  if (!f.open(QIODevice::ReadOnly)) return {};
  return QJsonDocument::fromJson(f.readAll()).object().value("BuildId").toString();
}

bool buildIdsAligned(const QString &uproject, QString *report) {
  const QString uprojectDir = QFileInfo(uproject).absolutePath();
  const QString carla_unreal_id = manifestBuildId(uprojectDir + "/Binaries/Linux/UnrealEditor.modules");
  const QString carla_id       = manifestBuildId(uprojectDir + "/Plugins/Carla/Binaries/Linux/UnrealEditor.modules");
  const QString carla_tools_id  = manifestBuildId(uprojectDir + "/Plugins/CarlaTools/Binaries/Linux/UnrealEditor.modules");
  if (report) *report = QString("CarlaUnreal=%1 Carla=%2 CarlaTools=%3")
                          .arg(carla_unreal_id.left(8), carla_id.left(8), carla_tools_id.left(8));
  if (carla_unreal_id.isEmpty()) return false;
  return carla_id == carla_unreal_id && carla_tools_id == carla_unreal_id;
}

}

PreflightOutcome run_importer_preflight(const QString             &uproject,
                                      const QString             &engine_path,
                                      const QString             &editor_binary,
                                      const PreflightLogSink    &log,
                                      const PreflightProgressFn &progress) {
  PreflightOutcome out;
  out.uproject_path = uproject;
  out.editor_binary = editor_binary;

  if (uproject.isEmpty() || !QFileInfo(uproject).isFile()) {
    out.reason = "Vehicle .uproject not resolved.";
    return out;
  }
  if (engine_path.isEmpty()) {
    out.reason = "UE Engine path not resolved.";
    return out;
  }

  if (progress) progress(2, "checking importer source…");
  if (log) log("Preflight: ensuring importer source files are present in CarlaTools plugin.");

  const QString plug = pluginDir(uproject);
  const QString hPath  = plug + "/Source/CarlaTools/Public/VehicleImporter.h";
  const QString cppPath = plug + "/Source/CarlaTools/Private/VehicleImporter.cpp";
  const QString modPath = plug + "/Source/CarlaTools/Private/CarlaTools.cpp";

  const QByteArray hRes   = readResource("CarlaTools/VehicleImporter.h");
  const QByteArray cppRes = readResource("CarlaTools/VehicleImporter.cpp");
  const QByteArray modRes = readResource("CarlaTools/CarlaTools.cpp");
  if (hRes.isEmpty() || cppRes.isEmpty() || modRes.isEmpty()) {
    out.reason = "Bundled importer source missing from Studio resources (build issue).";
    return out;
  }

  if (!writeIfDifferent(hPath,   hRes,   &out.source_written, log) ||
      !writeIfDifferent(cppPath, cppRes, &out.source_written, log)) {
    out.reason = "Could not write importer source files into the CarlaTools plugin.";
    return out;
  }

  bool modulePatched = false;
  if (!writeIfDifferent(modPath, modRes, &modulePatched, log)) {
    out.reason = "Could not patch CarlaTools.cpp (StartupModule hook).";
    return out;
  }
  out.module_hook_patched = modulePatched;

  const QString soPath = pluginSoPath(uproject);
  QString buildIdReport;
  const bool buildIdsOk = buildIdsAligned(uproject, &buildIdReport);
  const bool soOk       = soHasListener(soPath);
  if (log) log(QString("Preflight: BuildIds %1 - %2").arg(buildIdsOk ? "aligned" : "MISMATCHED", buildIdReport));
  if (log) log(QString("Preflight: listener symbol in CarlaTools.so - %1").arg(soOk ? "present" : "MISSING"));

  if (out.source_written || out.module_hook_patched) {
    if (log) log("Preflight: importer source files were updated on disk. The currently-installed "
                 "CarlaTools.so may be stale - if Import hangs, run the rebuild manually:");
    if (log) log("    " + engine_path + "/Engine/Build/BatchFiles/Linux/Build.sh "
                 "CarlaUnrealEditor Linux Development \"" + uproject + "\" -waitmutex");
  }
  if (!buildIdsOk) {
    if (log) log("Preflight: BuildIds are mismatched - UE may refuse to load one or more plugins. "
                 "If the editor exits during launch, run the rebuild command above to realign.");
  }
  if (!soOk) {
    if (log) log("Preflight: importer listener (UVehicleImporter::StartServer) is not in the .so. "
                 "Import will time out. Run the rebuild command above first.");
  }

  if (progress) progress(100, "preflight done - proceeding");
  out.result = PreflightOutcome::Result::ReadyExisting;
  return out;
}

}
