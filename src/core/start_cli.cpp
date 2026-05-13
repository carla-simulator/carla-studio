// Copyright (C) 2026 Abdul, Hashim.
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// carla-studio-cli_start — headless CARLA simulator launcher
//
// Usage: carla-studio start [--map <MapName>] [--port <rpc-port>] [--directory <carla-root>]

#include <QCoreApplication>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QStandardPaths>
#include <QTextStream>

namespace {

QTextStream &out() { static QTextStream s(stdout); return s; }
QTextStream &err() { static QTextStream s(stderr); return s; }

static void log(const QString &line) { out() << "[start] " << line << "\n"; out().flush(); }
static void logErr(const QString &line) { err() << "[start] ERROR: " << line << "\n"; err().flush(); }

static QString pidFile() {
    const QString cfg = QDir::homePath() + "/.config/carla-studio";
    QDir().mkpath(cfg);
    return cfg + "/carla-pids.txt";
}

static void writePid(qint64 pid) {
    QFile f(pidFile());
    if (f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream(&f) << pid << "\n";
    }
}

static QString findCarlaRoot(const QString &hint) {
    if (!hint.isEmpty() && QFileInfo(hint).isDir()) return hint;
    const QString env = QString::fromLocal8Bit(qgetenv("CARLA_ROOT")).trimmed();
    if (!env.isEmpty() && QFileInfo(env).isDir()) return env;
    return {};
}

static QString findShippingBinary(const QString &carla_root) {
    static const QStringList kBinaries = {
        "CarlaUnreal-Linux-Shipping",
        "CarlaUnreal.sh",
        "CarlaUE5.sh",
        "CarlaUE4.sh",
        "LinuxNoEditor/CarlaUE4.sh",
    };
    // Check flat layout first (0.9.x / most releases).
    for (const QString &rel : kBinaries) {
        const QString p = carla_root + "/" + rel;
        if (QFileInfo(p).isFile()) return p;
    }
    // Some releases (e.g. 0.10.0) extract into a single top-level subdirectory.
    QDirIterator it(carla_root, QDir::Dirs | QDir::NoDotAndDotDot, QDirIterator::NoIteratorFlags);
    while (it.hasNext()) {
        const QString sub = it.next();
        for (const QString &rel : kBinaries) {
            const QString p = sub + "/" + rel;
            if (QFileInfo(p).isFile()) return p;
        }
    }
    return {};
}

}

int carla_cli_start_main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();

    QString mapName;
    int rpc_port = 2000;
    QString directory;
    bool showProgress = false;

    for (int i = 1; i < args.size(); ++i) {
        const QString a = args[i];
        if (a == "--progress") { showProgress = true; }
        else if ((a == "--map" || a == "-m") && i + 1 < args.size())
            mapName = args[++i];
        else if ((a == "--port" || a == "-p") && i + 1 < args.size())
            rpc_port = args[++i].toInt();
        else if ((a == "--directory" || a == "-d") && i + 1 < args.size())
            directory = args[++i];
    }

    const QString carla_root = findCarlaRoot(directory);
    if (carla_root.isEmpty()) {
        logErr("Cannot find CARLA root. Set CARLA_ROOT or use --directory <path>.");
        return 1;
    }

    const QString binary = findShippingBinary(carla_root);
    if (binary.isEmpty()) {
        logErr("No CARLA shipping binary found in: " + carla_root);
        logErr("Expected: CarlaUnreal-Linux-Shipping, CarlaUnreal.sh, CarlaUE5.sh, or CarlaUE4.sh");
        return 1;
    }

    QStringList launch_args;
    launch_args << QString("-carla-rpc-port=%1").arg(rpc_port)
                << QString("-carla-streaming-port=%1").arg(rpc_port + 1)
                << "-RenderOffScreen";
    if (!mapName.isEmpty())
        launch_args << QString("-map=%1").arg(mapName);

    const QString cmd = QString("'%1' %2 >/tmp/carla_studio_launch.log 2>&1 & echo $!")
                          .arg(binary)
                          .arg(launch_args.join(' '));

    log(QString("Launching: %1 %2").arg(binary).arg(launch_args.join(' ')));
    if (!mapName.isEmpty()) log("Map: " + mapName);

    QProcess shell;
    shell.start("/bin/bash", QStringList() << "-c" << cmd);
    shell.waitForFinished(15000);
    const QString pidText = QString::fromLocal8Bit(shell.readAllStandardOutput()).trimmed();
    bool ok = false;
    const qint64 pid = pidText.toLongLong(&ok);
    if (!ok || pid <= 0) {
        logErr("Failed to launch CARLA — no PID returned.");
        logErr("Check /tmp/carla_studio_launch.log for details.");
        return 1;
    }

    writePid(pid);
    log(QString("CARLA started (PID %1). Log: /tmp/carla_studio_launch.log").arg(pid));
    log("RPC port: " + QString::number(rpc_port));

    if (showProgress) {
        log(QString("Waiting for CARLA on port %1 (max 120 s)...").arg(rpc_port));
        const int kMaxWait = 120;
        bool ready = false;
        for (int s = 3; s <= kMaxWait; s += 3) {
            QProcess nc;
            nc.start("/bin/sh", QStringList() << "-c"
                     << QString("nc -z -w2 127.0.0.1 %1").arg(rpc_port));
            nc.waitForFinished(5000);
            if (nc.exitCode() == 0) { ready = true; break; }
            out() << QString("\r[start] waiting... (%1/%2 s)   ").arg(s).arg(kMaxWait);
            out().flush();
        }
        out() << "\n";
        if (ready)
            log("CARLA is ready on port " + QString::number(rpc_port));
        else
            log("Port " + QString::number(rpc_port) + " not reachable after "
                + QString::number(kMaxWait) + " s — check /tmp/carla_studio_launch.log");
    }

    return 0;
}
