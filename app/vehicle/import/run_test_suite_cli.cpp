// SPDX-License-Identifier: AGPL-3.0-or-later
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QGuiApplication>
#include <QProcess>
#include <QRegularExpression>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QTimer>
#include <QtTest/QtTest>

extern void runCliPipelineTests(int argc, char **argv, int &exitCode);

static bool captureScreenshots(const QString &studioBin,
                                const QString &outDir,
                                const QString &display) {
    QDir().mkpath(outDir);
    QProcess p;
    p.setProgram(studioBin);
    p.setArguments({"--capture-screenshots", outDir});
    QStringList env = QProcess::systemEnvironment();
    if (!display.isEmpty()) {
        env.removeIf([](const QString &e){ return e.startsWith("DISPLAY="); });
        env << "DISPLAY=" + display;
    }
    p.setEnvironment(env);
    p.start();
    if (!p.waitForStarted(5000)) {
        QTextStream(stderr) << "[test-suite] carla-studio did not start\n";
        return false;
    }
    if (!p.waitForFinished(30000)) {
        p.kill();
        QTextStream(stderr) << "[test-suite] carla-studio capture timed out\n";
        return false;
    }
    const QStringList pngs = QDir(outDir).entryList({"*.png"}, QDir::Files);
    if (pngs.isEmpty()) {
        QTextStream(stderr) << "[test-suite] no screenshots produced\n";
        return false;
    }
    QTextStream(stdout) << "[test-suite] captured " << pngs.size() << " screenshots -> " << outDir << "\n";
    return true;
}

static void patchUsageMd(const QString &usageMdPath, const QString &screenshotsDir,
                          const QString &repoRoot) {
    QFile f(usageMdPath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream(stderr) << "[test-suite] cannot read " << usageMdPath << "\n";
        return;
    }
    QString src = QString::fromUtf8(f.readAll());
    f.close();

    const QString relDir = QDir(repoRoot).relativeFilePath(screenshotsDir);

    static const QList<QPair<QString, QString>> mapping = {
        { "## GUI",                    "window-full.png"            },
        { "### Connection",            "tab-0-Home.png"             },
        { "### Sensor calibration",    "tab-1-Interfaces.png"       },
        { "### World and actor",       "tab-2-Health_Check.png"     },
        { "### Scenario tools",        "tab-3-Scenario_Builder.png" },
        { "### Vehicle import wizard", "tab-4-Vehicle_Import.png"   },
    };

    for (const auto &[heading, shotFile] : mapping) {
        const QString fullShot = screenshotsDir + "/" + shotFile;
        if (!QFileInfo(fullShot).exists()) continue;
        const QString imgTag = "\n![](" + relDir + "/" + shotFile + ")\n";
        const int pos = src.indexOf(heading);
        if (pos < 0) continue;
        const int eol = src.indexOf('\n', pos);
        if (eol < 0) continue;
        if (src.mid(eol, imgTag.size()) == imgTag) continue;
        src.insert(eol, imgTag);
    }

    if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        QTextStream(stderr) << "[test-suite] cannot write " << usageMdPath << "\n";
        return;
    }
    QTextStream(&f) << src;
    QTextStream(stdout) << "[test-suite] USAGE.md updated\n";
}

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);

    const QStringList args = app.arguments();
    const bool updateDocs = args.contains("--update-documentation");

    const QString display = qEnvironmentVariable("DISPLAY", ":0");
    QTextStream(stdout) << "[test-suite] DISPLAY=" << display << "\n";

    const QString binDir  = QCoreApplication::applicationDirPath();
#ifdef CARLA_STUDIO_REPO_ROOT
    const QString repoRoot = qEnvironmentVariable("CARLA_STUDIO_REPO_ROOT",
                               QStringLiteral(CARLA_STUDIO_REPO_ROOT));
#else
    const QString repoRoot = qEnvironmentVariable("CARLA_STUDIO_REPO_ROOT",
                               binDir + "/../../Apps/CarlaStudio");
#endif

    int exitCode = 0;

    QTextStream(stdout) << "\n=== CLI pipeline tests ===\n";
    runCliPipelineTests(argc, argv, exitCode);

    QTextStream(stdout) << "\n=== GUI matrix tests ===\n";
    QProcess guiProc;
    guiProc.setProgram(binDir + "/test-suite_carla-studio-gui");
    guiProc.setProcessChannelMode(QProcess::ForwardedChannels);
    guiProc.start();
    const bool guiStarted = guiProc.waitForStarted(5000);
    if (!guiStarted) {
        QTextStream(stderr) << "[test-suite] test-suite_carla-studio-gui did not start\n";
        exitCode = 1;
    } else {
        guiProc.waitForFinished(120000);
        if (guiProc.exitCode() != 0) {
            QTextStream(stderr) << "[test-suite] GUI matrix tests FAILED\n";
            exitCode = 1;
        }
    }

    if (updateDocs) {
        const bool guiOk = guiStarted && guiProc.exitCode() == 0;
        if (!guiOk) {
            QTextStream(stdout) << "[test-suite] skipping --update-documentation (GUI tests did not pass)\n";
        } else {
            const QString screenshotsDir = repoRoot + "/docs/usage/screenshots";
            const bool captured = captureScreenshots(binDir + "/carla-studio",
                                                     screenshotsDir, display);
            if (captured) {
                patchUsageMd(repoRoot + "/docs/USAGE.md", screenshotsDir, repoRoot);
            } else {
                exitCode = 1;
            }
        }
    }

    QTextStream(stdout) << "\n[test-suite] " << (exitCode == 0 ? "PASS" : "FAIL") << "\n";
    QTimer::singleShot(0, &app, [exitCode, &app]() { app.exit(exitCode); });
    return app.exec();
}
