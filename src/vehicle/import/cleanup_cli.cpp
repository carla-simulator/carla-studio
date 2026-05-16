// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include <QCoreApplication>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QSet>

#include <signal.h>
#include <sys/types.h>
#include <unistd.h>

namespace {

struct PatternHit {
  pid_t   pid = 0;
  QString cmdline;
  QString matchedBy;
};

QList<PatternHit> findByRegex(const QString &posixExtRegex)
{
  QList<PatternHit> hits;
  QProcess p;
  p.start("/bin/sh", QStringList() << "-c"
            << QString("pgrep -af -- '%1'")
                 .arg(QString(posixExtRegex).replace('\'', "'\\''")));
  p.waitForFinished(3000);
  const QString stdoutText = QString::fromLocal8Bit(p.readAllStandardOutput());
  for (const QString &line : stdoutText.split('\n', Qt::SkipEmptyParts)) {
    const qsizetype sp = line.indexOf(' ');
    if (sp <= 0) continue;
    bool ok = false;
    const int pidI = line.left(static_cast<int>(sp)).toInt(&ok);
    if (!ok || pidI <= 1) continue;
    PatternHit h;
    h.pid       = static_cast<pid_t>(pidI);
    h.cmdline   = line.mid(static_cast<int>(sp) + 1).trimmed();
    h.matchedBy = posixExtRegex;
    hits.append(h);
  }
  return hits;
}

}

int carla_cli_cleanup_main(int argc, char **argv)
{
  QCoreApplication app(argc, argv);
  QTextStream out(stdout);

  static const QStringList patterns = {
    // CARLA simulator processes
    QStringLiteral("CarlaUnreal-Linux-Shipping"),
    QStringLiteral("(^|/)CarlaUnreal\\.sh"),
    QStringLiteral("(^|/)CarlaUE[45]\\.sh"),
    QStringLiteral("UnrealEditor[^ ]*[ ].*CarlaUnreal\\.uproject"),
    QStringLiteral("UnrealEditor[^ ]*[ ].*-run=cook"),
    // CARLA Studio main binary
    QStringLiteral("(^|/)carla-studio($| )"),
    // CARLA Studio CLI sub-binaries
    QStringLiteral("(^|/)carla-studio-cli_setup($| )"),
    QStringLiteral("(^|/)carla-studio-cli_start($| )"),
    QStringLiteral("(^|/)carla-studio-cli_stop($| )"),
    QStringLiteral("(^|/)carla-studio-cli_maps($| )"),
    QStringLiteral("(^|/)carla-studio-cli_sensor($| )"),
    QStringLiteral("(^|/)carla-studio-cli_actuate($| )"),
    QStringLiteral("(^|/)carla-studio-cli_healthcheck($| )"),
    QStringLiteral("(^|/)carla-studio-cli_vehicle-import($| )"),
    QStringLiteral("(^|/)carla-studio-vehicle-preview($| )"),
    // CARLA Studio test suite binaries
    QStringLiteral("(^|/)carla-studio-test-suite($| )"),
    QStringLiteral("(^|/)carla-studio-vehicle-import-test($| )"),
    QStringLiteral("(^|/)carla-studio-gui-matrix-test($| )"),
    // wget/tar spawned by setup CLI for CARLA downloads
    QStringLiteral("wget[^ ].*carla-releases"),
    QStringLiteral("wget[^ ].*tiny\\.carla\\.org"),
    QStringLiteral("tar[^ ]*.*CARLA_.*\\.tar\\.gz"),
    // git clone/pull spawned by maps CLI (community maps) or setup CLI (source build)
    QStringLiteral("git[^ ]*[ ]clone[^ ]*.*mcity"),
    QStringLiteral("git[^ ]*[ ]clone[^ ]*.*apollo"),
    QStringLiteral("git[^ ]*[ ]clone[^ ]*.*carla-simulator"),
    QStringLiteral("git[^ ]*[ ]submodule.*recursive"),
  };

  const pid_t myPid     = getpid();
  const pid_t myParent  = getppid();

  QSet<pid_t> seen;
  QList<PatternHit> all;
  for (const QString &pat : patterns) {
    for (const PatternHit &h : findByRegex(pat)) {
      if (seen.contains(h.pid)) continue;
      seen.insert(h.pid);
      all.append(h);
    }
  }

  QList<PatternHit> killable;
  for (const PatternHit &h : all) {
    if (h.pid == myPid)    continue;
    if (h.pid == myParent) continue;
    if (h.cmdline.contains(QStringLiteral("carla-studio-cli_cleanup"))) continue;
    if (h.cmdline.contains(QStringLiteral("pgrep -af"))) continue;
    if (h.cmdline.startsWith(QStringLiteral("/bin/sh -c"))) continue;
    killable.append(h);
  }

  if (killable.isEmpty()) {
    out << "[cleanup] nothing to kill - no stale CARLA Studio processes found.\n";
    return 0;
  }

  out << "[cleanup] killing " << killable.size() << " process(es):\n";
  for (const PatternHit &h : killable) {
    out << QString("  pid %1  [%2]  %3\n")
            .arg(h.pid, 6)
            .arg(h.matchedBy)
            .arg(h.cmdline.left(140));
    ::kill(h.pid, SIGKILL);
  }
  out.flush();

  QProcess::execute("/bin/sh", QStringList() << "-c" << "sleep 1");
  QSet<pid_t> survivors;
  for (const QString &pat : patterns) {
    for (const PatternHit &h : findByRegex(pat)) {
      if (h.pid == myPid || h.pid == myParent) continue;
      if (h.cmdline.contains(QStringLiteral("carla-studio-cli_cleanup"))) continue;
      if (h.cmdline.contains(QStringLiteral("pgrep -af"))) continue;
      if (h.cmdline.startsWith(QStringLiteral("/bin/sh -c"))) continue;
      survivors.insert(h.pid);
    }
  }
  if (!survivors.isEmpty()) {
    out << "[cleanup] " << survivors.size()
        << " process(es) survived SIGKILL - escalating with sudo would be needed.\n";
    for (pid_t p : survivors) out << "  surviving pid " << p << "\n";
    return 1;
  }

  out << "[cleanup] all clear.\n";
  return 0;
}
