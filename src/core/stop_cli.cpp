// Copyright (C) 2026 Abdul, Hashim.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QThread>

#include <csignal>
#include <sys/types.h>

namespace {

QTextStream &out() { static QTextStream s(stdout); return s; }
static void log(const QString &line) { out() << "[stop] " << line << "\n"; out().flush(); }

static QString pidFile() {
    return QDir::homePath() + "/.config/carla-studio/carla-pids.txt";
}

static QList<qint64> readTrackedPids() {
    QList<qint64> pids;
    QFile f(pidFile());
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return pids;
    for (const QString &line : QString::fromLocal8Bit(f.readAll()).split('\n', Qt::SkipEmptyParts)) {
        bool ok = false;
        const qint64 p = line.trimmed().toLongLong(&ok);
        if (ok && p > 1) pids << p;
    }
    return pids;
}

static bool pid_alive(qint64 pid) {
    return ::kill(static_cast<pid_t>(pid), 0) == 0;
}

static QList<qint64> pgrepPids(const QString &pattern) {
    QList<qint64> pids;
    QProcess p;
    p.start("pgrep", {"-f", pattern});
    p.waitForFinished(3000);
    for (const QString &tok : QString::fromLocal8Bit(p.readAllStandardOutput()).split('\n', Qt::SkipEmptyParts)) {
        bool ok = false;
        const qint64 pid = tok.trimmed().toLongLong(&ok);
        if (ok && pid > 1) pids << pid;
    }
    return pids;
}

static void killGroup(const QString &pattern, int sig) {
    for (qint64 pid : pgrepPids(pattern)) {
        ::kill(static_cast<pid_t>(pid), sig);
        log(QString("  signal %1 → PID %2 [%3]").arg(sig).arg(pid).arg(pattern));
    }
}

static void stopStack() {
    log("Stopping Docker CARLA containers...");
    QProcess::execute("/bin/sh", {"-c",
        "docker stop carla-mcity-server 2>/dev/null; docker rm carla-mcity-server 2>/dev/null; "
        "docker ps -q --filter ancestor=carlasim/carla | xargs -r docker stop 2>/dev/null || true"});

    log("Stopping TeraSim...");
    killGroup("'terasim|TeraSim'", SIGTERM);

    log("Stopping ROS2 / visualization...");
    killGroup("'ros2|rviz|carla_av_ros2|carla_sensor_ros2|carla_cosim_ros2|gnss_decoder_fallback|planning_simulator_fallback'", SIGTERM);
    QProcess::execute("/bin/sh", {"-c", "ros2 daemon stop 2>/dev/null || true"});

    log("Stopping SUMO / scenario runners...");
    killGroup("'sumo-gui|sumo |libsumo|traci|terasim_examples|_example\\.py'", SIGTERM);

    log("Removing Docker network...");
    QProcess::execute("/bin/sh", {"-c", "docker network rm carla-net 2>/dev/null || true"});

    log("Stack stopped.");
}

}

int carla_cli_stop_main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();

    bool full = false;
    for (const QString &a : args)
        if (a == "--full" || a == "-f") full = true;

    QList<qint64> pids = readTrackedPids();
    for (qint64 p : pgrepPids("'CarlaUE[45]-Linux|UnrealEditor.*CarlaUnreal|CarlaUnreal-Linux-Shipping|CarlaUnreal\\.sh|CarlaUE[45]\\.sh'")) {
        if (!pids.contains(p)) pids << p;
    }

    if (pids.isEmpty()) {
        log("No tracked CARLA processes found — nothing to stop.");
    } else {
        log(QString("Sending SIGTERM to %1 process(es)...").arg(pids.size()));
        for (qint64 pid : pids) {
            if (pid_alive(pid)) {
                ::kill(static_cast<pid_t>(pid), SIGTERM);
                log(QString("  SIGTERM → PID %1").arg(pid));
            }
        }

        const int grace_steps = 30;
        const int step_ms = 2000;
        for (int i = 0; i < grace_steps; ++i) {
            QThread::msleep(static_cast<unsigned long>(step_ms));
            bool any_alive = false;
            for (qint64 pid : pids) {
                if (pid_alive(pid)) { any_alive = true; break; }
            }
            if (!any_alive) break;
        }

        for (qint64 pid : pids) {
            if (pid_alive(pid)) {
                ::kill(static_cast<pid_t>(pid), SIGKILL);
                log(QString("  SIGKILL → PID %1").arg(pid));
            }
        }

        for (qint64 pid : pgrepPids("'CarlaUE4|CarlaUE5|CarlaUnreal'")) {
            ::kill(static_cast<pid_t>(pid), SIGKILL);
            log(QString("  SIGKILL (sweep) → PID %1").arg(pid));
        }

        QFile::remove(pidFile());
        log("CARLA stopped.");
    }

    if (full) stopStack();
    return 0;
}
