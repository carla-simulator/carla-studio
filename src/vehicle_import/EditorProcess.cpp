// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_import/EditorProcess.h"

#include <QByteArray>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QList>

#include <signal.h>
#include <sys/types.h>
#include <unistd.h>

namespace carla_studio::vehicle_import {

namespace {

QByteArray readFileBytes(const QString &path) {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) return {};
  return f.readAll();
}

qint64 bootTimeUnix() {
  const QByteArray data = readFileBytes("/proc/stat");
  for (const QByteArray &line : data.split('\n')) {
    if (line.startsWith("btime ")) {
      return line.mid(6).trimmed().toLongLong();
    }
  }
  return 0;
}

qint64 procStartUnix(qint64 pid) {
  const QByteArray data = readFileBytes(QString("/proc/%1/stat").arg(pid));
  if (data.isEmpty()) return 0;
  const int rparen = data.lastIndexOf(')');
  if (rparen < 0 || rparen + 2 >= data.size()) return 0;
  const QList<QByteArray> rest = data.mid(rparen + 2).split(' ');
  if (rest.size() <= 19) return 0;
  bool ok = false;
  const qulonglong ticks = rest[19].toULongLong(&ok);
  if (!ok) return 0;
  const long hz = sysconf(_SC_CLK_TCK);
  if (hz <= 0) return 0;
  const qint64 boot = bootTimeUnix();
  if (boot == 0) return 0;
  return boot + (qint64)(ticks / (qulonglong)hz);
}

QByteArray procCmdline(qint64 pid) {
  return readFileBytes(QString("/proc/%1/cmdline").arg(pid));
}

bool isAlive(qint64 pid) {
  return pid > 0 && QFileInfo(QString("/proc/%1").arg(pid)).exists();
}

}  // namespace

EditorProcessInfo findEditorForUproject(const QString &uprojectPath) {
  EditorProcessInfo out;
  if (uprojectPath.isEmpty()) return out;

  const QString uprojectAbs = QFileInfo(uprojectPath).absoluteFilePath();
  const QByteArray uprojectName = QFileInfo(uprojectAbs).fileName().toUtf8();

  QDir proc("/proc");
  const QStringList entries = proc.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
  for (const QString &e : entries) {
    bool ok = false;
    const qint64 pid = e.toLongLong(&ok);
    if (!ok || pid <= 0) continue;

    const QByteArray cmdline = procCmdline(pid);
    if (cmdline.isEmpty()) continue;
    if (cmdline.indexOf("UnrealEditor") < 0) continue;
    if (cmdline.indexOf(uprojectName) < 0)   continue;

    out.pid           = pid;
    out.exists        = true;
    out.isHeadless    = (cmdline.indexOf("-unattended") >= 0);
    out.startUnixTime = procStartUnix(pid);
    return out;
  }
  return out;
}

qint64 pluginSoMtimeUnix(const QString &uprojectPath) {
  if (uprojectPath.isEmpty()) return 0;
  const QString uprojectDir = QFileInfo(uprojectPath).absolutePath();
  const QString soPath = uprojectDir
    + "/Plugins/CarlaTools/Binaries/Linux/libUnrealEditor-CarlaTools.so";
  const QFileInfo fi(soPath);
  if (!fi.exists()) return 0;
  return fi.lastModified().toSecsSinceEpoch();
}

bool editorIsStale(const EditorProcessInfo &info,
                   const QString          &uprojectPath) {
  if (!info.exists || info.startUnixTime == 0) return false;
  const qint64 soMtime = pluginSoMtimeUnix(uprojectPath);
  if (soMtime == 0) return false;
  return soMtime > info.startUnixTime;
}

bool killEditor(qint64 pid, int timeoutMs) {
  if (pid <= 0) return true;
  if (!isAlive(pid)) return true;

  ::kill((pid_t)pid, SIGTERM);

  const int stepMs = 100;
  int waited = 0;
  while (waited < timeoutMs) {
    usleep((useconds_t)stepMs * 1000);
    waited += stepMs;
    if (!isAlive(pid)) return true;
  }
  ::kill((pid_t)pid, SIGKILL);
  waited = 0;
  while (waited < 2000) {
    usleep((useconds_t)stepMs * 1000);
    waited += stepMs;
    if (!isAlive(pid)) return true;
  }
  return !isAlive(pid);
}

}  // namespace carla_studio::vehicle_import
