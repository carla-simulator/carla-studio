// Copyright (C) 2026 Abdul, Hashim.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTemporaryFile>
#include <QTextStream>
#include <QThread>

namespace {

QTextStream &out() { static QTextStream s(stdout); return s; }
static void log(const QString &line) { out() << "[stack] " << line << "\n"; out().flush(); }

static QString applyTokens(QString cmd,
                            const QString &parentDir,
                            const QString &scriptDir,
                            const QString &carlaDir,
                            const QString &carlaPort,
                            const QString &mapName,
                            const QString &pythonBin,
                            const QString &terasimScenario,
                            const QString &vehicleType)
{
    const QString self = QCoreApplication::applicationFilePath();
    cmd.replace("__PARENT_DIR__",       parentDir);
    cmd.replace("__SCRIPT_DIR__",       scriptDir);
    cmd.replace("__CARLA_DIR__",        carlaDir);
    cmd.replace("__CARLA_PORT__",       carlaPort);
    cmd.replace("__DEFAULT_MAP_NAME__", mapName);
    cmd.replace("__PYTHON_BIN__",       pythonBin);
    cmd.replace("__TERASIM_SCENARIO__", terasimScenario);
    cmd.replace("__CARLA_VEHICLE_TYPE__", vehicleType);
    cmd.replace("carla-studio cosim",    self + " cosim");
    cmd.replace("carla-studio preview",  self + " preview");
    cmd.replace("carla-studio scenario", self + " scenario");
    return cmd;
}

static QString findJson(const QString &carlaDir) {
    const QStringList candidates = {
        QCoreApplication::applicationDirPath() + "/cfg/terminal_launch_map.json",
        QDir::homePath() + "/.config/carla-studio/terminal_launch_map.json",
        carlaDir + "/terminal_launch_map.json",
    };
    for (const QString &p : candidates)
        if (QFileInfo::exists(p)) return p;
    return {};
}

static QString generateTerminatorConfig(const QJsonArray &terminals,
                                         const QJsonArray &tabs,
                                         const QString &parentDir,
                                         const QString &scriptDir,
                                         const QString &carlaDir,
                                         const QString &carlaPort,
                                         const QString &mapName,
                                         const QString &pythonBin,
                                         const QString &terasimScenario,
                                         const QString &vehicleType)
{
    QMap<int, QString> tabLabels;
    for (const QJsonValue &tv : tabs)
        tabLabels[tv.toObject()["id"].toInt()] = tv.toObject()["label"].toString();

    QList<QJsonObject> terms;
    for (const QJsonValue &tv : terminals) {
        const QJsonObject t = tv.toObject();
        if (t["modes"].toObject()["terminator"].toBool()) terms.append(t);
    }

    QList<int> tabIds;
    for (const QJsonObject &t : terms) {
        int tid = t["tab"].toInt();
        if (!tabIds.contains(tid)) tabIds.append(tid);
    }
    std::sort(tabIds.begin(), tabIds.end());

    QStringList labels;
    for (int tid : tabIds) labels << tabLabels.value(tid, QString("Tab %1").arg(tid));

    QString cfg;
    QTextStream s(&cfg);
    s << "[global_config]\n"
      << "  window_state = maximise\n"
      << "  suppress_multiple_term_dialog = True\n"
      << "  tab_position = top\n\n"
      << "[profiles]\n"
      << "  [[default]]\n"
      << "    scrollback_lines = 50000\n"
      << "    login_shell = True\n\n"
      << "[layouts]\n"
      << "  [[CARLA_MCityFullStack]]\n"
      << "    [[[notebook0]]]\n"
      << "      active_page = 0\n"
      << "      labels = " << labels.join(", ") << "\n"
      << "      order = 0\n"
      << "      parent = window0\n"
      << "      type = Notebook\n";

    int tabIndex = 1;
    for (int tabId : tabIds) {
        const QString tabNode  = QString("tab%1").arg(tabIndex);
        const QString leftNode = QString("tab%1_left").arg(tabIndex);
        const QString rightNode = QString("tab%1_right").arg(tabIndex);

        s << QString("    [[[%1]]]\n").arg(tabNode)
          << QString("      order = %1\n").arg(tabIndex - 1)
          << "      parent = notebook0\n"
          << "      position = 632\n"
          << "      type = HPaned\n"
          << QString("    [[[%1]]]\n").arg(leftNode)
          << "      order = 0\n"
          << QString("      parent = %1\n").arg(tabNode)
          << "      position = 354\n"
          << "      type = VPaned\n"
          << QString("    [[[%1]]]\n").arg(rightNode)
          << "      order = 1\n"
          << QString("      parent = %1\n").arg(tabNode)
          << "      position = 354\n"
          << "      type = VPaned\n";

        for (const QJsonObject &t : terms) {
            if (t["tab"].toInt() != tabId) continue;
            const int pane = t["pane"].toInt(1);
            const QString parent = (pane <= 2) ? leftNode : rightNode;
            const int order = (pane == 1 || pane == 3) ? 0 : 1;
            const QString nodeName = QString("terminal_t%1_p%2").arg(tabId).arg(pane);
            QString cmd = applyTokens(t["command"].toString(),
                parentDir, scriptDir, carlaDir, carlaPort, mapName,
                pythonBin, terasimScenario, vehicleType);
            cmd.replace("\\", "\\\\").replace("\"", "\\\"");
            const QString resolved = (cmd == "bash" || cmd == "htop" || cmd == "nvtop")
                ? cmd
                : QString("bash -c \"%1; exec bash\"").arg(cmd);

            s << QString("    [[[%1]]]\n").arg(nodeName)
              << QString("      command = %1\n").arg(resolved)
              << QString("      order = %1\n").arg(order)
              << QString("      parent = %1\n").arg(parent)
              << "      profile = default\n"
              << "      type = Terminal\n"
              << QString("      title = %1\n").arg(t["title"].toString());
        }
        ++tabIndex;
    }

    s << "    [[[window0]]]\n"
      << "      order = 0\n"
      << "      parent = \"\"\n"
      << "      position = 0:25\n"
      << "      size = 1920, 1080\n"
      << "      type = Window\n\n"
      << "[plugins]\n";

    return cfg;
}

}

int carla_cli_stack_main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();

    bool useTerminator = false;
    QString carlaDir = QString::fromLocal8Bit(qgetenv("CARLA_ROOT")).trimmed();
    QString mapName  = "McityMap_Main";
    QString scenario = "default_sumo";
    QString vehicleType = "vehicle.lincoln.mkz_2020";
    QString port = "2000";
    QString workspaceRoot = QDir::currentPath();

    for (int i = 1; i < args.size(); ++i) {
        const QString &a = args[i];
        if (a == "--terminator" || a == "-t")         useTerminator = true;
        else if ((a == "--carla-dir" || a == "-c") && i + 1 < args.size())      carlaDir      = args[++i];
        else if ((a == "--map" || a == "-m") && i + 1 < args.size())            mapName       = args[++i];
        else if ((a == "--scenario" || a == "-s") && i + 1 < args.size())       scenario      = args[++i];
        else if ((a == "--port" || a == "-p") && i + 1 < args.size())           port          = args[++i];
        else if ((a == "--vehicle-type") && i + 1 < args.size())                vehicleType   = args[++i];
        else if ((a == "--workspace-root" || a == "-w") && i + 1 < args.size()) workspaceRoot = args[++i];
    }

    const QString jsonPath = findJson(carlaDir);
    if (jsonPath.isEmpty()) {
        log("terminal_launch_map.json not found. Expected in cfg/ next to binary or ~/.config/carla-studio/");
        return 1;
    }

    QFile f(jsonPath);
    if (!f.open(QIODevice::ReadOnly)) {
        log("Cannot read " + jsonPath);
        return 1;
    }
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    const QJsonArray terminals = doc.object()["terminals"].toArray();
    const QJsonArray tabs      = doc.object()["tabs"].toArray();

    const QString parentDir = workspaceRoot;
    const QString scriptDir = workspaceRoot;
    const QString pythonBin = "python3";
    const QString modeKey   = useTerminator ? "terminator" : "gnome";

    if (useTerminator) {
        QProcess terminatorCheck;
        terminatorCheck.start("/bin/sh", {"-c", "command -v terminator"});
        terminatorCheck.waitForFinished(2000);
        if (terminatorCheck.exitCode() != 0) {
            log("Terminator not found; falling back to gnome-terminal");
            useTerminator = false;
        }
    }

    if (useTerminator) {
        const QString cfgContent = generateTerminatorConfig(terminals, tabs,
            parentDir, scriptDir, carlaDir, port, mapName,
            pythonBin, scenario, vehicleType);

        QTemporaryFile tmpCfg;
        tmpCfg.setAutoRemove(false);
        if (!tmpCfg.open()) { log("Cannot write terminator config"); return 1; }
        tmpCfg.write(cfgContent.toUtf8());
        tmpCfg.close();

        log("Launching Terminator");
        QProcess::startDetached("terminator",
            {"-g", tmpCfg.fileName(), "-l", "CARLA_MCityFullStack"});
        return 0;
    }

    log("Launching GNOME Terminal windows");
    int launched = 0;
    for (const QJsonValue &tv : terminals) {
        const QJsonObject t = tv.toObject();
        if (!t["modes"].toObject()[modeKey].toBool()) continue;

        const QString title = t["title"].toString();
        const QString cmd   = applyTokens(t["command"].toString(),
            parentDir, scriptDir, carlaDir, port, mapName,
            pythonBin, scenario, vehicleType);
        const int sleepMs = t["post_spawn_sleep"].toInt(0) * 1000;

        log("Window: " + title);
        QProcess::startDetached("gnome-terminal",
            {"--title=" + title, "--",
             "/bin/bash", "-c",
             cmd + "; read -p 'Press Enter to close...'"});

        ++launched;
        if (sleepMs > 0) QThread::msleep(static_cast<unsigned long>(sleepMs));
    }

    log(QString("Launched %1 GNOME terminal windows").arg(launched));
    return 0;
}
