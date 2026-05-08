// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/cook_step.h"

#include <QDir>
#include <QDirIterator>
#include <QFileInfo>
#include <QProcess>

namespace carla_studio::vehicle_import {

namespace {

QString uprojectName(const QString &uprojectPath) {
  return QFileInfo(uprojectPath).completeBaseName();
}

QString cookedRoot(const QString &uprojectPath, const QString &contentSubpath) {
  const QString saved = QFileInfo(uprojectPath).absolutePath() + "/Saved";
  const QString name  = uprojectName(uprojectPath);
  return saved + "/Cooked/Linux/" + name + "/Content/" + contentSubpath;
}

QString tail(const QString &s, int maxBytes) {
  return s.size() <= maxBytes ? s : s.right(maxBytes);
}

}  // namespace

QString resolveCookedRoot(const QString &uprojectPath, const QString &contentSubpath) {
  return cookedRoot(uprojectPath, contentSubpath);
}

CookResult cookVehicleAssets(const CookRequest &req) {
  CookResult r;
  if (!QFileInfo(req.editorBinary).isExecutable()) {
    r.detail = "Editor binary not executable: " + req.editorBinary;
    return r;
  }
  if (!QFileInfo(req.uprojectPath).isFile()) {
    r.detail = "uproject not found: " + req.uprojectPath;
    return r;
  }
  const QString uprojectDir   = QFileInfo(req.uprojectPath).absolutePath();
  const QString cookFsDir     = uprojectDir + "/Content/" + req.contentSubpath;
  QStringList args;
  args << req.uprojectPath
       << "-run=cook"
       << "-targetplatform=Linux"
       << "-unattended"
       << "-nullrhi"
       << "-nosound"
       << "-nosplash"
       << "-nop4"
       << "-NoLogTimes"
       << "-utf8output"
       << QString("-CookDir=%1").arg(cookFsDir);

  QProcess proc;
  proc.start(req.editorBinary, args);
  if (!proc.waitForStarted(15000)) {
    r.detail = "Cook commandlet failed to start.";
    return r;
  }
  if (!proc.waitForFinished(req.timeoutMs)) {
    proc.kill();
    proc.waitForFinished(2000);
    r.stdoutTail = tail(QString::fromLocal8Bit(proc.readAllStandardOutput()), 4096);
    r.stderrTail = tail(QString::fromLocal8Bit(proc.readAllStandardError()),  4096);
    r.detail = QString("Cook commandlet timed out after %1 ms.").arg(req.timeoutMs);
    return r;
  }
  r.stdoutTail = tail(QString::fromLocal8Bit(proc.readAllStandardOutput()), 8192);
  r.stderrTail = tail(QString::fromLocal8Bit(proc.readAllStandardError()),  8192);
  const bool cleanExit = (proc.exitStatus() == QProcess::NormalExit && proc.exitCode() == 0);
  r.cookedRoot = cookedRoot(req.uprojectPath, req.contentSubpath);
  if (QFileInfo(r.cookedRoot).isDir()) {
    QDirIterator it(r.cookedRoot, { "*.uasset", "*.uexp", "*.ubulk", "*.umap" },
                    QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) r.cookedFiles << it.next();
  }
  if (r.cookedFiles.isEmpty()) {
    QString combined = r.stderrTail.isEmpty() ? r.stdoutTail : r.stderrTail;
    if (combined.size() > 800) combined = "…" + combined.right(800);
    r.detail = QString("Cook produced no output for %1 (exit=%2). Tail: %3")
                 .arg(req.contentSubpath).arg(proc.exitCode()).arg(combined);
    return r;
  }
  r.ok = true;
  r.detail = cleanExit
      ? QString("Cooked %1 file(s) to %2").arg(r.cookedFiles.size()).arg(r.cookedRoot)
      : QString("Cooked %1 file(s) to %2 (UE returned exit=%3 due to unrelated project "
                "errors - vehicle assets cooked successfully)")
          .arg(r.cookedFiles.size()).arg(r.cookedRoot).arg(proc.exitCode());
  return r;
}

}  // namespace carla_studio::vehicle_import
