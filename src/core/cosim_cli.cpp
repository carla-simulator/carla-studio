// Copyright (C) 2026 Abdul, Hashim.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include <QCoreApplication>
#include <QDir>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QThread>

namespace {

QTextStream &out() { static QTextStream s(stdout); return s; }
static void log(const QString &line) { out() << "[cosim] " << line << "\n"; out().flush(); }

static bool pythonImport(const QStringList &modules) {
    const QString expr = "import " + modules.join(", ");
    QProcess p;
    p.start("python3", {"-c", expr});
    p.waitForFinished(8000);
    return p.exitCode() == 0;
}

static bool portOpen(const QString &host, int port) {
    QProcess nc;
    nc.start("/bin/sh", {"-c", QString("nc -z -w2 %1 %2").arg(host).arg(port)});
    nc.waitForFinished(4000);
    return nc.exitCode() == 0;
}

}

int carla_cli_cosim_main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();

    QString directory;
    for (int i = 1; i < args.size(); ++i) {
        if ((args[i] == "--directory" || args[i] == "-d") && i + 1 < args.size())
            directory = args[++i];
    }

    if (!pythonImport({"carla", "terasim_cosim"})) {
        log("Co-simulation dependencies unavailable on this host");
        return 0;
    }

    QProcess redisCheck;
    redisCheck.start("/bin/sh", {"-c", "pgrep -x redis-server"});
    redisCheck.waitForFinished(2000);
    if (redisCheck.exitCode() != 0) {
        QProcess redisCheck2;
        redisCheck2.start("/bin/sh", {"-c", "command -v redis-server"});
        redisCheck2.waitForFinished(2000);
        if (redisCheck2.exitCode() == 0) {
            QProcess::startDetached("/bin/bash", {"-c", "nohup redis-server &"});
            log("Redis server started");
        } else {
            log("Redis not present; co-simulation may not work");
        }
    }

    log("Waiting for CARLA readiness");
    bool ready = false;
    for (int s = 0; s < 90; ++s) {
        if (portOpen("127.0.0.1", 2000)) { ready = true; break; }
        QThread::sleep(1);
    }
    if (!ready) {
        log("CARLA not ready after 90s; skipping co-simulation");
        return 0;
    }

    const QString examplesDir = directory.isEmpty()
        ? QDir::currentPath()
        : (directory + "/examples/carla_examples");

    log("Starting co-simulation");
    QProcess proc;
    proc.setWorkingDirectory(examplesDir);
    proc.start("python3", {"carla_cosim_ros2.py"});
    proc.waitForFinished(-1);
    log(proc.exitCode() == 0 ? "Co-simulation finished" : "Co-simulation stopped");
    return 0;
}
