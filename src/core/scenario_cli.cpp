// Copyright (C) 2026 Abdul, Hashim.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>

namespace {
QTextStream &out() { static QTextStream s(stdout); return s; }
static void log(const QString &line) { out() << "[scenario] " << line << "\n"; out().flush(); }

static bool pythonImport(const QStringList &modules) {
    const QString expr = "import " + modules.join(", ");
    QProcess p;
    p.start("python3", {"-c", expr});
    p.waitForFinished(8000);
    return p.exitCode() == 0;
}
}

int carla_cli_scenario_main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();

    QString scenario = qEnvironmentVariable("TERASIM_SCENARIO", "default_sumo");
    QString directory;
    for (int i = 1; i < args.size(); ++i) {
        if ((args[i] == "--scenario" || args[i] == "-s") && i + 1 < args.size())
            scenario = args[++i];
        else if ((args[i] == "--directory" || args[i] == "-d") && i + 1 < args.size())
            directory = args[++i];
    }

    log(QString("Scenario preparation (%1)").arg(scenario));

    if (!pythonImport({"sumolib", "traci", "terasim"})) {
        log("Scenario dependencies unavailable on this host");
        return 0;
    }

    const QString examplesDir = directory.isEmpty()
        ? QDir::currentPath()
        : (directory + "/examples/terasim_examples");

    QString target = scenario + "_example.py";
    if (!QFileInfo::exists(examplesDir + "/" + target)) {
        log(QString("Scenario %1 not present; using default_sumo_example.py").arg(target));
        target = "default_sumo_example.py";
    }

    log("Running scenario " + target);
    QProcess proc;
    proc.setWorkingDirectory(examplesDir);
    proc.start("python3", {target});
    proc.waitForFinished(-1);
    log(proc.exitCode() == 0 ? "Scenario finished" : "Scenario stopped");
    return 0;
}
