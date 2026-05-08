// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/editor_process.h"

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
  const qsizetype rparen = data.lastIndexOf(')');
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

}

EditorProcessInfo find_editor_for_uproject(const QString &uproject_path) {
  EditorProcessInfo out;
  if (uproject_path.isEmpty()) return out;

  const QString uprojectAbs = QFileInfo(uproject_path).absoluteFilePath();
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
    out.is_headless    = (cmdline.indexOf("-unattended") >= 0);
    out.start_unix_time = procStartUnix(pid);
    return out;
  }
  return out;
}

qint64 plugin_so_mtime_unix(const QString &uproject_path) {
  if (uproject_path.isEmpty()) return 0;
  const QString uprojectDir = QFileInfo(uproject_path).absolutePath();
  const QString soPath = uprojectDir
    + "/Plugins/CarlaTools/Binaries/Linux/libUnrealEditor-CarlaTools.so";
  const QFileInfo fi(soPath);
  if (!fi.exists()) return 0;
  return fi.lastModified().toSecsSinceEpoch();
}

bool editor_is_stale(const EditorProcessInfo &info,
                   const QString          &uproject_path) {
  if (!info.exists || info.start_unix_time == 0) return false;
  const qint64 soMtime = plugin_so_mtime_unix(uproject_path);
  if (soMtime == 0) return false;
  return soMtime > info.start_unix_time;
}

bool kill_editor(qint64 pid, int timeout_ms) {
  if (pid <= 0) return true;
  if (!isAlive(pid)) return true;

  ::kill((pid_t)pid, SIGTERM);

  const int stepMs = 100;
  int waited = 0;
  while (waited < timeout_ms) {
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

}
