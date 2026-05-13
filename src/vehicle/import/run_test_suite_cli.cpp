// SPDX-License-Identifier: AGPL-3.0-or-later
#include <QApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QRegularExpression>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QTimer>
#include <QtTest/QtTest>

extern void runCliPipelineTests(int argc, char **argv, int &exit_code);
extern void runGuiMatrixTests(int argc, char **argv, int &exit_code);

static bool captureScreenshots(const QString &studioBin,
                                const QString &out_dir,
                                const QString &display) {
    QDir().mkpath(out_dir);
    QProcess p;
    p.setProgram(studioBin);
    p.setArguments({"--capture-screenshots", out_dir});
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
    const QStringList pngs = QDir(out_dir).entryList({"*.png"}, QDir::Files);
    if (pngs.isEmpty()) {
        QTextStream(stderr) << "[test-suite] no screenshots produced\n";
        return false;
    }
    QTextStream(stdout) << "[test-suite] captured " << pngs.size() << " screenshots -> " << out_dir << "\n";
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
        const qsizetype pos = src.indexOf(heading);
        if (pos < 0) continue;
        const qsizetype eol = src.indexOf('\n', pos);
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
    QApplication app(argc, argv);

    const QStringList args = app.arguments();
    const bool updateDocs = args.contains("--update-documentation");

    const QString display = qEnvironmentVariable("DISPLAY", ":0");
    QTextStream(stdout) << "[test-suite] DISPLAY=" << display << "\n";

    const QString bin_dir  = QCoreApplication::applicationDirPath();
#ifdef CARLA_STUDIO_REPO_ROOT
    const QString repoRoot = qEnvironmentVariable("CARLA_STUDIO_REPO_ROOT",
                               QStringLiteral(CARLA_STUDIO_REPO_ROOT));
#else
    const QString repoRoot = qEnvironmentVariable("CARLA_STUDIO_REPO_ROOT",
                               bin_dir + "/../../Apps/CarlaStudio");
#endif

    // Allow up to 1 h per test function so live-sim tests (mcity clone ~270 s,
    // CARLA boot ~120 s, source build ~3 h) don't trip the Qt Test watchdog.
    // Must be set BEFORE QTest::qExec() is called — initTestCase() is too late.
    qputenv("QTEST_TIMEOUT", qgetenv("CARLA_LONG_TESTS").isEmpty() ? "3600" : "10800");

    int exit_code = 0;

    QTextStream(stdout) << "\n=== CLI pipeline tests ===\n";
    runCliPipelineTests(argc, argv, exit_code);

    int gui_code = 0;
    QTextStream(stdout) << "\n=== GUI matrix tests ===\n";
    runGuiMatrixTests(argc, argv, gui_code);
    exit_code |= gui_code;

    if (updateDocs) {
        if (gui_code != 0) {
            QTextStream(stdout) << "[test-suite] skipping --update-documentation (GUI tests did not pass)\n";
        } else {
            const QString screenshotsDir = repoRoot + "/docs/usage/tabs";
            const bool captured = captureScreenshots(bin_dir + "/carla-studio",
                                                     screenshotsDir, display);
            if (captured) {
                patchUsageMd(repoRoot + "/docs/USAGE.md", screenshotsDir, repoRoot);
            } else {
                exit_code = 1;
            }
        }
    }

    QTextStream(stdout) << "\n[test-suite] " << (exit_code == 0 ? "PASS" : "FAIL") << "\n";
    QTimer::singleShot(0, &app, [exit_code, &app]() { app.exit(exit_code); });
    return app.exec();
}
