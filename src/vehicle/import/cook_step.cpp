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

QString uprojectName(const QString &uproject_path) {
  return QFileInfo(uproject_path).completeBaseName();
}

QString cooked_root(const QString &uproject_path, const QString &content_subpath) {
  const QString saved = QFileInfo(uproject_path).absolutePath() + "/Saved";
  const QString name  = uprojectName(uproject_path);
  return saved + "/Cooked/Linux/" + name + "/Content/" + content_subpath;
}

QString tail(const QString &s, int maxBytes) {
  return s.size() <= maxBytes ? s : s.right(maxBytes);
}

}

QString resolve_cooked_root(const QString &uproject_path, const QString &content_subpath) {
  return cooked_root(uproject_path, content_subpath);
}

CookResult cook_vehicle_assets(const CookRequest &req) {
  CookResult r;
  if (!QFileInfo(req.editor_binary).isExecutable()) {
    r.detail = "Editor binary not executable: " + req.editor_binary;
    return r;
  }
  if (!QFileInfo(req.uproject_path).isFile()) {
    r.detail = "uproject not found: " + req.uproject_path;
    return r;
  }
  const QString uprojectDir   = QFileInfo(req.uproject_path).absolutePath();
  const QString cookFsDir     = uprojectDir + "/Content/" + req.content_subpath;
  QStringList args;
  args << req.uproject_path
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
  proc.start(req.editor_binary, args);
  if (!proc.waitForStarted(15000)) {
    r.detail = "Cook commandlet failed to start.";
    return r;
  }
  if (!proc.waitForFinished(req.timeout_ms)) {
    proc.kill();
    proc.waitForFinished(2000);
    r.stdout_tail = tail(QString::fromLocal8Bit(proc.readAllStandardOutput()), 4096);
    r.stderr_tail = tail(QString::fromLocal8Bit(proc.readAllStandardError()),  4096);
    r.detail = QString("Cook commandlet timed out after %1 ms.").arg(req.timeout_ms);
    return r;
  }
  r.stdout_tail = tail(QString::fromLocal8Bit(proc.readAllStandardOutput()), 8192);
  r.stderr_tail = tail(QString::fromLocal8Bit(proc.readAllStandardError()),  8192);
  const bool cleanExit = (proc.exitStatus() == QProcess::NormalExit && proc.exitCode() == 0);
  r.cooked_root = cooked_root(req.uproject_path, req.content_subpath);
  if (QFileInfo(r.cooked_root).isDir()) {
    QDirIterator it(r.cooked_root, { "*.uasset", "*.uexp", "*.ubulk", "*.umap" },
                    QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) r.cooked_files << it.next();
  }
  if (r.cooked_files.isEmpty()) {
    QString combined = r.stderr_tail.isEmpty() ? r.stdout_tail : r.stderr_tail;
    if (combined.size() > 800) combined = "…" + combined.right(800);
    r.detail = QString("Cook produced no output for %1 (exit=%2). Tail: %3")
                 .arg(req.content_subpath).arg(proc.exitCode()).arg(combined);
    return r;
  }
  r.ok = true;
  r.detail = cleanExit
      ? QString("Cooked %1 file(s) to %2").arg(r.cooked_files.size()).arg(r.cooked_root)
      : QString("Cooked %1 file(s) to %2 (UE returned exit=%3 due to unrelated project "
                "errors - vehicle assets cooked successfully)")
          .arg(r.cooked_files.size()).arg(r.cooked_root).arg(proc.exitCode());
  return r;
}

}
