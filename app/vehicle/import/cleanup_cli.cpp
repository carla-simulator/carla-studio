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
    const int sp = line.indexOf(' ');
    if (sp <= 0) continue;
    bool ok = false;
    const int pidI = line.left(sp).toInt(&ok);
    if (!ok || pidI <= 1) continue;
    PatternHit h;
    h.pid       = static_cast<pid_t>(pidI);
    h.cmdline   = line.mid(sp + 1).trimmed();
    h.matchedBy = posixExtRegex;
    hits.append(h);
  }
  return hits;
}

}  // namespace

int main(int argc, char **argv)
{
  QCoreApplication app(argc, argv);
  QTextStream out(stdout);

  // POSIX extended regex patterns. pgrep -f matches against the full
  // command line of each process. Patterns are anchored / made specific
  // enough that they don't catch unrelated processes.
  static const QStringList patterns = {
    QStringLiteral("(^|/)carla-studio-cli_vehicle-import($| )"),
    QStringLiteral("(^|/)carla-studio-vehicle-preview($| )"),
    QStringLiteral("(^|/)carla-studio-vehicle-import-test($| )"),
    QStringLiteral("(^|/)carla-studio-gui-matrix-test($| )"),
    QStringLiteral("(^|/)carla-studio($| )"),
    QStringLiteral("UnrealEditor[^ ]*[ ].*CarlaUnreal\\.uproject"),
    QStringLiteral("UnrealEditor[^ ]*[ ].*-run=cook"),
    QStringLiteral("CarlaUnreal-Linux-Shipping"),
    QStringLiteral("(^|/)CarlaUnreal\\.sh"),
    QStringLiteral("(^|/)CarlaUE[45]\\.sh"),
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

  // Filter out self, parent shell, the cleanup binary itself, AND any
  // /bin/sh helpers that are running pgrep on our behalf - those have the
  // search pattern as a substring of their own command line, so they show
  // up in pgrep results.
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

  // Give the kernel a moment to reap, then re-scan to see what survived.
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
