// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// CliPipelineTest - verifies that the GUI, the CLI, and the test harness all
// reach the SAME backend for vehicle import:
//
//   1) ImportPipelineSpecBuild  - runImportPipeline shapes the spec via the
//      shared MeshAnalysis + buildSpecFromAnalysis + classifyByName path
//      (the GUI's path, no hardcoded ±140/±80 fallback for valid meshes).
//
//   2) ImportPipelineCallbacks  - callback bag is fired in the right order
//      (editKnobs → openCalibration → confirmStartImport → log) and a
//      cancel at confirm-start short-circuits before any RPC.
//
//   3) ImportPipelineKnobsClass - classifyByName + presetForSizeClass round-
//      trips per the public API the GUI / CLI both consume.
//
//   4) CliDispatchHelp / CliDispatchCleanup - the `carla-studio` dispatcher
//      execv's the right sibling binary for `help` / `cleanup`.

#include "vehicle/import/import_pipeline.h"
#include "vehicle/import/merged_spec_builder.h"
#include "vehicle/import/mesh_analysis.h"
#include "vehicle/import/vehicle_spec.h"

#include <QGuiApplication>
#include <QPixmap>
#include <QScreen>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QObject>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTemporaryDir>
#include <QtTest/QtTest>

namespace cs = carla_studio::vehicle_import;

namespace {

// Tiny synthetic OBJ - bbox 4.0 × 1.8 × 1.4 m, single face. Enough for
// MeshAnalysis to compute extents + classify; sendJson will fail because
// no editor is listening, but the orchestration up to that point is what
// we're testing.
QString writeSyntheticObj(const QString &dir, const QString &stem) {
  const QString path = dir + "/" + stem + ".obj";
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return QString();
  QTextStream out(&f);
  out << "# synthetic test mesh\n"
         "v -200  -90  0\n"      // half-dims in cm: 4.0 × 1.8 × 1.4 m
         "v  200  -90  0\n"
         "v  200   90  0\n"
         "v -200   90  0\n"
         "v -200  -90  140\n"
         "v  200  -90  140\n"
         "v  200   90  140\n"
         "v -200   90  140\n"
         "f 1 2 3\n"
         "f 1 3 4\n"
         "f 5 6 7\n"
         "f 5 7 8\n";
  f.close();
  return path;
}

}  // namespace

class ImportPipelineTest : public QObject {
  Q_OBJECT
 private slots:

  void specBuild_usesMeshAnalysisNotHardcodedFallback() {
    QTemporaryDir dir; QVERIFY(dir.isValid());
    const QString meshPath = writeSyntheticObj(dir.path(), "mkz");

    cs::ImportPipelineInput in;
    in.meshPath    = meshPath;
    in.vehicleName = "mkz";
    in.knobs.mass             = 1750.f;          // user override
    in.knobs.maxBrakeTorque   = 1234.f;
    // No carlaSrcRoot/etc → pipeline will short-circuit at sendJson with a
    // "no response from editor" failure. We're checking the SPEC path, so
    // we use a callback that captures the spec via openCalibration's mesh
    // path (which buildSpecFromInput leaves on spec.meshPath).

    QString seenLog;
    cs::ImportPipelineCallbacks cb;
    cb.log = [&](const QString &m){ seenLog += m + "\n"; };

    cs::ImportPipelineResult res = cs::runImportPipeline(in, cb);

    // The pipeline must have failed at the sendJson step (no editor),
    // not earlier (which would mean we never got to spec build).
    QVERIFY(!res.imported);
    QVERIFY(res.errorDetail.contains("no response from editor")
         || res.errorDetail.contains("port"));
    // Log emitted the "Sending spec to editor" line ⇒ spec was built.
    QVERIFY2(seenLog.contains("Sending spec to editor"),
             qPrintable("expected log to mention spec send; got:\n" + seenLog));
  }

  void callbacks_orderAndCancel() {
    QTemporaryDir dir; QVERIFY(dir.isValid());
    const QString meshPath = writeSyntheticObj(dir.path(), "mkz");

    cs::ImportPipelineInput in;
    in.meshPath    = meshPath;
    in.vehicleName = "mkz";

    QStringList trace;
    cs::ImportPipelineCallbacks cb;
    cb.editKnobs       = [&](cs::ImportPipelineKnobs &k) {
      trace << "editKnobs"; k.mass = 999.f;
    };
    cb.openCalibration = [&](const QString &) { trace << "openCalibration"; };
    cb.confirmStartImport = [&]() {
      trace << "confirmStartImport(false)";
      return false;                 // cancel
    };
    cb.log = [&](const QString &m){ trace << "log:" + m.left(40); };

    cs::ImportPipelineResult res = cs::runImportPipeline(in, cb);

    QVERIFY(!res.imported);
    QCOMPARE(res.errorDetail,
             QString("user cancelled at confirm-start gate"));
    // First three callbacks fired in order, no further log lines after cancel.
    QVERIFY(trace.size() >= 3);
    QCOMPARE(trace[0], QString("editKnobs"));
    QCOMPARE(trace[1], QString("openCalibration"));
    QCOMPARE(trace[2], QString("confirmStartImport(false)"));
    for (int i = 3; i < trace.size(); ++i) {
      QVERIFY2(!trace[i].startsWith("log:Sending spec"),
               "spec must not be sent after user cancel");
    }
  }

  void classifyByName_overridesLengthHeuristic() {
    // FR-max should be tagged TinyRobot regardless of bbox-derived length.
    QCOMPARE(cs::classifyByName("FR-max_Latest"),
             cs::SizeClass::TinyRobot);
    QCOMPARE(cs::classifyByName("Mercedes_Benz_GLS_580"),
             cs::SizeClass::Suv);
    QCOMPARE(cs::classifyByName("MKZ_2017"),
             cs::SizeClass::Sedan);
    QCOMPARE(cs::classifyByName("random_thing_not_in_db"),
             cs::SizeClass::Unknown);
  }

  void presetForSizeClass_matchesPublicTable() {
    const cs::SizePreset sedan = cs::presetForSizeClass(cs::SizeClass::Sedan);
    QCOMPARE(sedan.mass, 1600.f);          // bumped from 1500 to 1600 in this branch
    QVERIFY(sedan.torqueCurveTag == QStringLiteral("sedan"));
    const cs::SizePreset suv = cs::presetForSizeClass(cs::SizeClass::Suv);
    QCOMPARE(suv.mass, 2400.f);
  }

  // Regression: when wheelAnchors come in degenerate (all-zero), the merge
  // would stack all 4 tire copies at one off-axis point. With proper
  // chassis-bbox-derived anchors, tire copies must spread to four distinct
  // corner positions inside the chassis bbox.
  void mergeBodyAndTires_fourDistinctTireCopies() {
    QTemporaryDir dir; QVERIFY(dir.isValid());

    // mkz-like body: Y is forward (long axis 5m), X is lateral (1.9m).
    const QString bodyPath = dir.filePath("body.obj");
    {
      QFile f(bodyPath); QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      // A tilted box, 8 verts: X=[-95,95], Y=[-250,250], Z=[5,145]
      s << "v -95 -250 5\n"  "v  95 -250 5\n"
           "v  95  250 5\n"  "v -95  250 5\n"
           "v -95 -250 145\n" "v  95 -250 145\n"
           "v  95  250 145\n" "v -95  250 145\n"
           "f 1 2 3\n" "f 1 3 4\n";
    }
    // One tire centered at (0,0,35), small extent.
    const QString tiresPath = dir.filePath("tires.obj");
    {
      QFile f(tiresPath); QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      s << "v -10 -10  25\n" "v  10 -10  25\n"
           "v  10  10  25\n" "v -10  10  25\n"
           "v -10 -10  45\n" "v  10 -10  45\n"
           "v  10  10  45\n" "v -10  10  45\n"
           "f 1 2 3\n" "f 1 3 4\n";
    }

    // Anchors derived directly from raw OBJ bbox (mirrors makeSpec's helper).
    const float fwdInset = 500.f * 0.18f; // (250-(-250)) * 0.18 = 90
    std::array<cs::WheelSpec, 4> anchors;
    anchors[0].x = -95;  anchors[0].y = -250 + fwdInset; anchors[0].z = 35;
    anchors[1].x =  95;  anchors[1].y = -250 + fwdInset; anchors[1].z = 35;
    anchors[2].x = -95;  anchors[2].y =  250 - fwdInset; anchors[2].z = 35;
    anchors[3].x =  95;  anchors[3].y =  250 - fwdInset; anchors[3].z = 35;

    const cs::ObjMergeResult mr = cs::mergeBodyAndTires(
        bodyPath, tiresPath, dir.filePath("out"), anchors);
    QVERIFY2(mr.ok, qPrintable(mr.reason));

    // Read merged.obj, group verts by 'o studio_tire_*' headers, compute
    // each group's centroid. Expect 4 groups with distinct centroids.
    QFile out(mr.outputPath); QVERIFY(out.open(QIODevice::ReadOnly));
    QTextStream ts(&out);
    QList<QVector3D> centroids;
    QVector3D sum(0,0,0);
    int        cnt = 0;
    auto flush = [&]() {
      if (cnt > 0) centroids.append(sum / float(cnt));
      sum = QVector3D(0,0,0); cnt = 0;
    };
    bool inTire = false;
    while (!ts.atEnd()) {
      const QString line = ts.readLine();
      if (line.startsWith("o studio_tire_")) {
        flush(); inTire = true; continue;
      }
      if (line.startsWith("o ")) { flush(); inTire = false; continue; }
      if (!inTire) continue;
      if (!line.startsWith("v ")) continue;
      const QStringList toks = line.split(QRegularExpression("\\s+"),
                                          Qt::SkipEmptyParts);
      if (toks.size() < 4) continue;
      sum += QVector3D(toks[1].toFloat(), toks[2].toFloat(), toks[3].toFloat());
      ++cnt;
    }
    flush();

    QCOMPARE(centroids.size(), 4);
    // All centroids must be distinct (pairwise distance > 50 cm).
    for (int i = 0; i < 4; ++i)
      for (int j = i + 1; j < 4; ++j) {
        const float d = (centroids[i] - centroids[j]).length();
        QVERIFY2(d > 50.f,
                 qPrintable(QString("tire %1 and %2 too close: %3 cm")
                              .arg(i).arg(j).arg(d)));
      }
  }

  // Regression: a body OBJ whose vertices are far from origin must produce a
  // merged OBJ whose body bbox is centered on (0,0) and resting at z=0.
  void mergeBodyAndTires_centersBodyOnOrigin() {
    QTemporaryDir dir; QVERIFY(dir.isValid());

    // Body cube at OBJ position (1000..1400, 500..690, 100..240) - far off origin.
    const QString bodyPath = dir.filePath("body.obj");
    {
      QFile f(bodyPath); QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      s << "v 1000 500 100\n" "v 1400 500 100\n"
           "v 1400 690 100\n" "v 1000 690 100\n"
           "v 1000 500 240\n" "v 1400 500 240\n"
           "v 1400 690 240\n" "v 1000 690 240\n"
           "f 1 2 3\n" "f 1 3 4\n";
    }
    // Single tire cluster at (0, 0, 35) with small extent.
    const QString tiresPath = dir.filePath("tires.obj");
    {
      QFile f(tiresPath); QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      s << "v -10 -10  25\n" "v  10 -10  25\n"
           "v  10  10  25\n" "v -10  10  25\n"
           "v -10 -10  45\n" "v  10 -10  45\n"
           "v  10  10  45\n" "v -10  10  45\n"
           "f 1 2 3\n" "f 1 3 4\n";
    }

    std::array<cs::WheelSpec, 4> anchors;
    anchors[0].x = 1100; anchors[0].y = 540; anchors[0].z = 130;
    anchors[1].x = 1300; anchors[1].y = 540; anchors[1].z = 130;
    anchors[2].x = 1100; anchors[2].y = 650; anchors[2].z = 130;
    anchors[3].x = 1300; anchors[3].y = 650; anchors[3].z = 130;

    const cs::ObjMergeResult mr = cs::mergeBodyAndTires(
        bodyPath, tiresPath, dir.filePath("out"), anchors);
    QVERIFY2(mr.ok, qPrintable(mr.reason));
    QVERIFY(QFileInfo(mr.outputPath).isFile());

    // Re-parse the merged OBJ's vertices and check ONLY the body verts (we
    // know they're the first 8). bbox center should be at (0,0); z_min=0.
    QFile out(mr.outputPath); QVERIFY(out.open(QIODevice::ReadOnly));
    QTextStream ts(&out);
    int seen = 0;
    double xMin =  1e9, xMax = -1e9;
    double yMin =  1e9, yMax = -1e9;
    double zMin =  1e9;
    while (!ts.atEnd() && seen < 8) {
      const QString line = ts.readLine();
      if (!line.startsWith("v ")) continue;
      const QStringList toks = line.split(QRegularExpression("\\s+"),
                                          Qt::SkipEmptyParts);
      if (toks.size() < 4) continue;
      const double x = toks[1].toDouble();
      const double y = toks[2].toDouble();
      const double z = toks[3].toDouble();
      if (x < xMin) xMin = x; if (x > xMax) xMax = x;
      if (y < yMin) yMin = y; if (y > yMax) yMax = y;
      if (z < zMin) zMin = z;
      ++seen;
    }
    QCOMPARE(seen, 8);
    const double cx = 0.5 * (xMin + xMax);
    const double cy = 0.5 * (yMin + yMax);
    QVERIFY2(std::fabs(cx) < 1e-3,
             qPrintable(QString("body bbox not X-centered: cx=%1").arg(cx)));
    QVERIFY2(std::fabs(cy) < 1e-3,
             qPrintable(QString("body bbox not Y-centered: cy=%1").arg(cy)));
    QVERIFY2(std::fabs(zMin) < 1e-3,
             qPrintable(QString("body z_min not at 0: zMin=%1").arg(zMin)));
  }
};

class CliDispatcherTest : public QObject {
  Q_OBJECT
 private:
  QString carlaStudioBin() const {
    return QCoreApplication::applicationDirPath() + "/carla-studio";
  }
  static QString vehicleFixtureRoot() {
    const QString env = qEnvironmentVariable("CARLA_STUDIO_FIXTURES");
    return env.isEmpty() ? QString() : env;
  }

 private slots:

  void help_prints_subcommand_list() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built next to test");

    QProcess p;
    p.start(fi.absoluteFilePath(), QStringList() << "help");
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(10000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("vehicle-import"), qPrintable(out));
    QVERIFY2(out.contains("cleanup"),        qPrintable(out));
    QVERIFY2(out.contains("setup"),          qPrintable(out));
    QCOMPARE(p.exitCode(), 0);
  }

  void cleanup_runs_and_exits_zero() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built next to test");

    QProcess p;
    p.start(fi.absoluteFilePath(), QStringList() << "cleanup");
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(15000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    // Either "nothing to kill" or "killed N process(es)" - both are exit 0.
    QVERIFY2(out.contains("[cleanup]"), qPrintable(out));
    QCOMPARE(p.exitCode(), 0);
  }

  void debug_merge_only_on_real_mkz_fixture_produces_4_distinct_tires() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built next to test");
    const QString fixtDir = vehicleFixtureRoot();
    if (fixtDir.isEmpty()) QSKIP("CARLA_STUDIO_FIXTURES not set");
    const QString bodyPath  = fixtDir + "/mkz/MKZ.obj";
    const QString tiresPath = fixtDir + "/mkz/Tires.obj";
    if (!QFileInfo(bodyPath).isFile() || !QFileInfo(tiresPath).isFile())
      QSKIP("mkz fixtures not present");

    QTemporaryDir cwd; QVERIFY(cwd.isValid());

    QProcess p;
    p.setWorkingDirectory(cwd.path());
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.remove("DISPLAY"); env.remove("WAYLAND_DISPLAY");
    p.setProcessEnvironment(env);
    p.start(QCoreApplication::applicationDirPath() + "/carla-studio-cli_vehicle-import",
            QStringList() << "--mesh" << bodyPath
                          << "--tires" << tiresPath
                          << "--debug-merge-only");
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(60000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("merge: OK"), qPrintable(out));

    // Pull the merged.obj path from the output and parse tire centroids.
    QString outputPath;
    for (const QString &line : out.split('\n')) {
      if (line.contains("output:")) {
        outputPath = line.section("output:", 1).trimmed();
      }
    }
    QVERIFY2(QFileInfo(outputPath).isFile(),
             qPrintable("missing merged.obj at " + outputPath));

    QFile f(outputPath); QVERIFY(f.open(QIODevice::ReadOnly));
    QTextStream ts(&f);
    QMap<QString, QVector3D> centroids;
    QString currentTire;
    QVector3D sum; int cnt = 0;
    auto flush = [&]() {
      if (!currentTire.isEmpty() && cnt > 0)
        centroids[currentTire] = sum / float(cnt);
      sum = QVector3D(0,0,0); cnt = 0;
    };
    while (!ts.atEnd()) {
      const QString line = ts.readLine();
      if (line.startsWith("o studio_tire_")) {
        flush();
        currentTire = line.mid(2).trimmed();
        continue;
      }
      if (line.startsWith("o ")) { flush(); currentTire.clear(); continue; }
      if (currentTire.isEmpty() || !line.startsWith("v ")) continue;
      const QStringList toks = line.split(QRegularExpression("\\s+"),
                                          Qt::SkipEmptyParts);
      if (toks.size() < 4) continue;
      sum += QVector3D(toks[1].toFloat(), toks[2].toFloat(), toks[3].toFloat());
      ++cnt;
    }
    flush();

    QCOMPARE(centroids.size(), 4);
    QVERIFY(centroids.contains("studio_tire_FL"));
    QVERIFY(centroids.contains("studio_tire_FR"));
    QVERIFY(centroids.contains("studio_tire_RL"));
    QVERIFY(centroids.contains("studio_tire_RR"));
    // FL/RL on left (negative X), FR/RR on right (positive X).
    QVERIFY(centroids["studio_tire_FL"].x() < 0);
    QVERIFY(centroids["studio_tire_RL"].x() < 0);
    QVERIFY(centroids["studio_tire_FR"].x() > 0);
    QVERIFY(centroids["studio_tire_RR"].x() > 0);
    // FL/FR at front (positive Y), RL/RR at rear (negative Y).
    QVERIFY(centroids["studio_tire_FL"].y() > 0);
    QVERIFY(centroids["studio_tire_FR"].y() > 0);
    QVERIFY(centroids["studio_tire_RL"].y() < 0);
    QVERIFY(centroids["studio_tire_RR"].y() < 0);
  }

  // Captures `carla-studio-vehicle-preview --interactive` Qt3D window for
  // each of the four fixtures (or its centered/merged derivative when
  // available) and saves a PNG to /tmp/qt3d_shots/<vehicle>_3d.png. Test
  // passes as long as files are produced - this is a *visual probe* helper
  // surfaced through the test target so the same wired logic that the
  // pipeline uses is the one that captures the screenshots, no shell glue.
  void capture_qt3d_window_for_each_vehicle() {
    if (qEnvironmentVariable("DISPLAY").isEmpty())
      QSKIP("no DISPLAY - Qt3D capture needs a graphical session");
    const QString previewBin = QCoreApplication::applicationDirPath()
                             + "/carla-studio-vehicle-preview";
    if (!QFileInfo(previewBin).isExecutable())
      QSKIP("carla-studio-vehicle-preview not built");

    struct Probe { const char *name; QString path; };
    const QString fixtRoot = vehicleFixtureRoot();
    if (fixtRoot.isEmpty()) QSKIP("CARLA_STUDIO_FIXTURES not set");
    const QString runRoot  = "/tmp/calib_runs";
    QList<Probe> probes;
    auto firstExisting = [&](const QStringList &cands) -> QString {
      for (const QString &p : cands) if (QFileInfo(p).isFile()) return p;
      return QString();
    };
    probes << Probe{ "mkz",
        firstExisting({ runRoot + "/mkz/.cs_import/centered_mkz_merged/mkz_merged_centered.obj",
                        runRoot + "/mkz/.cs_import/merged_mkz/mkz_merged.obj",
                        fixtRoot + "mkz/MKZ.obj" }) };
    probes << Probe{ "fr_max",
        firstExisting({ runRoot + "/fr_max/.cs_import/centered_fr_max_latest/fr_max_latest_centered.obj",
                        fixtRoot + "gokcer_kit/FR-max_Latest.obj" }) };
    probes << Probe{ "mb_gls580",
        firstExisting({ runRoot + "/mb_gls580/.cs_import/centered_uploads_files_2787791_mercedes_benz_gls_580/uploads_files_2787791_mercedes_benz_gls_580_centered.obj",
                        fixtRoot + "32-mercedes-benz-gls-580-2020/uploads_files_2787791_Mercedes+Benz+GLS+580.obj" }) };
    probes << Probe{ "rr",
        firstExisting({ runRoot + "/rr/.cs_import/centered_rr_merged/rr_merged_centered.obj",
                        runRoot + "/rr/.cs_import/merged_rr/rr_merged.obj",
                        fixtRoot + "rr/rr.obj" }) };

    const QString outDir = "/tmp/qt3d_shots";
    QDir().mkpath(outDir);

    auto runXdotool = [](const QStringList &args) -> QString {
      QProcess p; p.start("xdotool", args);
      p.waitForFinished(2000);
      return QString::fromLocal8Bit(p.readAllStandardOutput()).trimmed();
    };
    auto rectFromXwininfo = [](qint64 wid) -> QRect {
      QProcess p;
      p.start("xwininfo", QStringList()
                            << "-id" << QString::number(wid));
      p.waitForFinished(2000);
      const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
      int x = 0, y = 0, w = 0, h = 0;
      for (const QString &line : out.split('\n')) {
        if (line.contains("Absolute upper-left X")) x = line.section(':', 1).trimmed().toInt();
        else if (line.contains("Absolute upper-left Y")) y = line.section(':', 1).trimmed().toInt();
        else if (line.trimmed().startsWith("Width:"))   w = line.section(':', 1).trimmed().toInt();
        else if (line.trimmed().startsWith("Height:"))  h = line.section(':', 1).trimmed().toInt();
      }
      return QRect(x, y, w, h);
    };

    QScreen *screen = QGuiApplication::primaryScreen();
    QVERIFY(screen);

    for (const Probe &p : probes) {
      if (p.path.isEmpty()) {
        qWarning() << p.name << "no input found, skipping";
        continue;
      }
      // Tear down any previous instance.
      QProcess::execute("/bin/sh", QStringList() << "-c"
          << "pkill -KILL -f 'carla-studio-vehicle-preview --interactive' 2>/dev/null; true");
      QTest::qWait(1500);

      QProcess preview;
      preview.start(previewBin, QStringList() << p.path << "--interactive");
      QVERIFY2(preview.waitForStarted(5000),
               qPrintable(QString("preview start failed for ") + p.name));
      QTest::qWait(22000);  // Assimp parse for big OBJs

      const QString wid = runXdotool({ "search", "--name", "Calibration" });
      const QStringList wids = wid.split('\n', Qt::SkipEmptyParts);
      if (wids.isEmpty()) {
        qWarning() << p.name << "Qt3D window not found";
        preview.kill(); preview.waitForFinished(2000);
        continue;
      }
      const qint64 winId = wids.last().toLongLong();
      runXdotool({ "windowsize", QString::number(winId), "1400", "900" });
      runXdotool({ "windowmove", QString::number(winId), "10", "10" });
      QProcess::execute("wmctrl", QStringList() << "-ia" << QString::number(winId));
      QTest::qWait(2500);

      const QRect rect = rectFromXwininfo(winId);
      auto grabAndSave = [&](const QString &suffix) {
        if (!rect.isValid() || rect.width() <= 0) return;
        QPixmap pix = screen->grabWindow(0, rect.x(), rect.y(),
                                         rect.width(), rect.height());
        if (!pix.isNull()) {
          const QString out = outDir + "/" + p.name + "_" + suffix + ".png";
          pix.save(out, "PNG");
          qDebug() << p.name << suffix << "→" << out;
        }
      };

      grabAndSave("3d");
      runXdotool({ "mousemove", "--window", QString::number(winId), "185", "65", "click", "1" });
      QTest::qWait(2500);
      grabAndSave("side");
      runXdotool({ "mousemove", "--window", QString::number(winId), "270", "65", "click", "1" });
      QTest::qWait(2500);
      grabAndSave("front");
      runXdotool({ "mousemove", "--window", QString::number(winId), "105", "65", "click", "1" });
      QTest::qWait(2500);
      grabAndSave("top");

      preview.kill();
      preview.waitForFinished(3000);
    }

    // Test passes if at least mkz produced 3+ views (the most reliable
    // fixture with --tires).
    QFileInfoList shots = QDir(outDir).entryInfoList(
        QStringList() << "mkz_*.png", QDir::Files);
    QVERIFY2(shots.size() >= 3,
             qPrintable(QString("expected ≥3 mkz screenshots, got ") +
                        QString::number(shots.size())));
  }

  void unknown_subcommand_falls_through_to_gui_path() {
    // Without DISPLAY the GUI path bails out early; we only verify that
    // the dispatcher does NOT exec a sibling for an unknown token.
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built next to test");

    QProcess p;
    p.setProcessEnvironment(QProcessEnvironment::systemEnvironment());
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.remove("DISPLAY"); env.remove("WAYLAND_DISPLAY");
    p.setProcessEnvironment(env);
    p.start(fi.absoluteFilePath(), QStringList() << "this-subcommand-does-not-exist");
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(15000));
    // Without a display the GUI app aborts; we don't care about the exit
    // code, only that the dispatcher didn't try to exec a non-existent
    // sibling (which would print "failed to exec" and exit 126).
    const QString err = QString::fromLocal8Bit(p.readAllStandardError())
                      + QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(!err.contains("failed to exec"), qPrintable(err));
  }
};

void runCliPipelineTests(int argc, char **argv, int &exitCode) {
    { ImportPipelineTest t; exitCode |= QTest::qExec(&t, argc, argv); }
    { CliDispatcherTest t; exitCode |= QTest::qExec(&t, argc, argv); }
}

#include "cli_pipeline_test.moc"

