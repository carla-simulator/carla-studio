// Copyright (C) 2026 Abdul, Hashim.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include <QCoreApplication>
#include <QDir>
#include <QProcess>
#include <QProcessEnvironment>
#include <QString>
#include <QStringList>
#include <QTextStream>

namespace {
QTextStream &out() { static QTextStream s(stdout); return s; }
static void log(const QString &line) { out() << "[preview] " << line << "\n"; out().flush(); }
}

int carla_cli_preview_main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();

    QString directory;
    for (int i = 1; i < args.size(); ++i) {
        if ((args[i] == "--directory" || args[i] == "-d") && i + 1 < args.size())
            directory = args[++i];
    }

    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();

    QProcess rosSetup;
    rosSetup.start("/bin/sh", {"-c", ". /opt/ros/*/setup.bash 2>/dev/null && env"});
    rosSetup.waitForFinished(5000);
    if (rosSetup.exitCode() == 0) {
        const QString envOutput = QString::fromLocal8Bit(rosSetup.readAllStandardOutput());
        for (const QString &line : envOutput.split('\n', Qt::SkipEmptyParts)) {
            const qsizetype eq = line.indexOf('=');
            if (eq > 0) env.insert(line.left(eq), line.mid(eq + 1));
        }
    }

    if (!directory.isEmpty()) {
        const QString autowareSetup = directory + "/autoware/install/setup.bash";
        if (QFile::exists(autowareSetup)) {
            QProcess awSetup;
            awSetup.start("/bin/sh", {"-c", ". " + autowareSetup + " 2>/dev/null && env"});
            awSetup.waitForFinished(5000);
            if (awSetup.exitCode() == 0) {
                const QString envOutput = QString::fromLocal8Bit(awSetup.readAllStandardOutput());
                for (const QString &line : envOutput.split('\n', Qt::SkipEmptyParts)) {
                    const qsizetype eq = line.indexOf('=');
                    if (eq > 0) env.insert(line.left(eq), line.mid(eq + 1));
                }
            }
        }
    }

    QProcess ros2Check;
    ros2Check.setProcessEnvironment(env);
    ros2Check.start("ros2", {"pkg", "list"});
    ros2Check.waitForFinished(8000);
    if (ros2Check.exitCode() != 0) {
        log("ROS2 command not available; preview control inactive");
        return 0;
    }

    const QString pkgList = QString::fromLocal8Bit(ros2Check.readAllStandardOutput());
    if (!pkgList.split('\n').contains("preview_control")) {
        log("Preview control package not installed on this host");
        return 0;
    }

    log("Starting preview control");
    QProcess proc;
    proc.setProcessEnvironment(env);
    proc.start("ros2", {"launch", "preview_control", "carla_control.launch.py"});
    proc.waitForFinished(-1);
    log(proc.exitCode() == 0 ? "Preview control finished" : "Preview control stopped");
    return 0;
}
