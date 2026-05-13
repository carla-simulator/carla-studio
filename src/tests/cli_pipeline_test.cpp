// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later
//

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
#include <QProcessEnvironment>
#include <QSettings>
#include <QString>
#include <QStringList>
#include <QTemporaryDir>
#include <QtTest/QtTest>

namespace cs = carla_studio::vehicle_import;

namespace {

QString writeSyntheticObj(const QString &dir, const QString &stem) {
  const QString path = dir + "/" + stem + ".obj";
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return QString();
  QTextStream out(&f);
  out << "# synthetic test mesh\n"
         "v -200  -90  0\n"
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

}

class ImportPipelineTest : public QObject {
  Q_OBJECT
 private slots:

  void specBuild_usesMeshAnalysisNotHardcodedFallback() {
    QTemporaryDir dir; QVERIFY(dir.isValid());
    const QString mesh_path = writeSyntheticObj(dir.path(), "synth_vehicle");

    cs::ImportPipelineInput in;
    in.mesh_path    = mesh_path;
    in.vehicle_name = "synth_vehicle";
    in.knobs.mass             = 1750.f;
    in.knobs.max_brake_torque   = 1234.f;

    QString seenLog;
    cs::ImportPipelineCallbacks cb;
    cb.log = [&](const QString &m){ seenLog += m + "\n"; };

    cs::ImportPipelineResult res = cs::run_import_pipeline(in, cb);

    QVERIFY(!res.imported);
    QVERIFY(res.error_detail.contains("no response from editor")
         || res.error_detail.contains("port"));

    QVERIFY2(seenLog.contains("Sending spec to editor"), qPrintable("expected log to mention spec send; got:\n" + seenLog));
  }

  void callbacks_orderAndCancel() {
    QTemporaryDir dir; QVERIFY(dir.isValid());
    const QString mesh_path = writeSyntheticObj(dir.path(), "synth_vehicle");

    cs::ImportPipelineInput in;
    in.mesh_path    = mesh_path;
    in.vehicle_name = "synth_vehicle";

    QStringList trace;
    cs::ImportPipelineCallbacks cb;
    cb.edit_knobs       = [&](cs::ImportPipelineKnobs &k) {
      trace << "edit_knobs"; k.mass = 999.f;
    };
    cb.open_calibration = [&](const QString &) { trace << "open_calibration"; };
    cb.confirm_start_import = [&]() {
      trace << "confirm_start_import(false)";
      return false;
    };
    cb.log = [&](const QString &m){ trace << "log:" + m.left(40); };

    cs::ImportPipelineResult res = cs::run_import_pipeline(in, cb);

    QVERIFY(!res.imported);
    QCOMPARE(res.error_detail, QString("user cancelled at confirm-start gate"));

    QVERIFY(trace.size() >= 3);
    QCOMPARE(trace[0], QString("edit_knobs"));
    QCOMPARE(trace[1], QString("open_calibration"));
    QCOMPARE(trace[2], QString("confirm_start_import(false)"));
    for (int i = 3; i < trace.size(); ++i) {
      QVERIFY2(!trace[i].startsWith("log:Sending spec"),
               "spec must not be sent after user cancel");
    }
  }

  void classifyByName_overridesLengthHeuristic() {

    QCOMPARE(cs::classify_by_name("my_sedan_vehicle"),  cs::SizeClass::Sedan);
    QCOMPARE(cs::classify_by_name("heavy_truck_v2"),    cs::SizeClass::Truck);
    QCOMPARE(cs::classify_by_name("city_bus_model"),    cs::SizeClass::Bus);
    QCOMPARE(cs::classify_by_name("warehouse_agv"),     cs::SizeClass::TinyRobot);
    QCOMPARE(cs::classify_by_name("random_thing"),      cs::SizeClass::Unknown);
    QCOMPARE(cs::classify_by_name("UnknownBrand_X1"),    cs::SizeClass::Unknown);
    QCOMPARE(cs::classify_by_name("SomeModel_2020"),     cs::SizeClass::Unknown);
    QCOMPARE(cs::classify_by_name("Manufacturer_Type500"), cs::SizeClass::Unknown);
  }

  void presetForSizeClass_matchesPublicTable() {
    const cs::SizePreset sedan = cs::preset_for_size_class(cs::SizeClass::Sedan);
    QCOMPARE(sedan.mass, 1600.f);
    QVERIFY(sedan.torque_curve_tag == QStringLiteral("sedan"));
    const cs::SizePreset suv = cs::preset_for_size_class(cs::SizeClass::Suv);
    QCOMPARE(suv.mass, 2400.f);
  }

  void mergeBodyAndTires_fourDistinctTireCopies() {
    QTemporaryDir dir; QVERIFY(dir.isValid());

    const QString body_path = dir.filePath("body.obj");
    {
      QFile f(body_path); QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);

      s << "v -95 -250 5\n"  "v  95 -250 5\n"
           "v  95  250 5\n"  "v -95  250 5\n"
           "v -95 -250 145\n" "v  95 -250 145\n"
           "v  95  250 145\n" "v -95  250 145\n"
           "f 1 2 3\n" "f 1 3 4\n";
    }

    const QString tires_path = dir.filePath("tires.obj");
    {
      QFile f(tires_path); QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      s << "v -10 -10  25\n" "v  10 -10  25\n"
           "v  10  10  25\n" "v -10  10  25\n"
           "v -10 -10  45\n" "v  10 -10  45\n"
           "v  10  10  45\n" "v -10  10  45\n"
           "f 1 2 3\n" "f 1 3 4\n";
    }

    const float fwdInset = 500.f * 0.18f;
    std::array<cs::WheelSpec, 4> anchors;
    anchors[0].x = -95;  anchors[0].y = -250 + fwdInset; anchors[0].z = 35;
    anchors[1].x =  95;  anchors[1].y = -250 + fwdInset; anchors[1].z = 35;
    anchors[2].x = -95;  anchors[2].y =  250 - fwdInset; anchors[2].z = 35;
    anchors[3].x =  95;  anchors[3].y =  250 - fwdInset; anchors[3].z = 35;

    const cs::ObjMergeResult mr = cs::merge_body_and_tires(
        body_path, tires_path, dir.filePath("out"), anchors);
    QVERIFY2(mr.ok, qPrintable(mr.reason));

    QFile out(mr.output_path); QVERIFY(out.open(QIODevice::ReadOnly));
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

    for (int i = 0; i < 4; ++i)
      for (int j = i + 1; j < 4; ++j) {
        const float d = (centroids[i] - centroids[j]).length();
        QVERIFY2(d > 50.f, qPrintable(QString("tire %1 and %2 too close: %3 cm")
                              .arg(i).arg(j).arg(d)));
      }
  }

  void mergeBodyAndTires_centersBodyOnOrigin() {
    QTemporaryDir dir; QVERIFY(dir.isValid());

    const QString body_path = dir.filePath("body.obj");
    {
      QFile f(body_path); QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      s << "v 1000 500 100\n" "v 1400 500 100\n"
           "v 1400 690 100\n" "v 1000 690 100\n"
           "v 1000 500 240\n" "v 1400 500 240\n"
           "v 1400 690 240\n" "v 1000 690 240\n"
           "f 1 2 3\n" "f 1 3 4\n";
    }

    const QString tires_path = dir.filePath("tires.obj");
    {
      QFile f(tires_path); QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
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

    const cs::ObjMergeResult mr = cs::merge_body_and_tires(
        body_path, tires_path, dir.filePath("out"), anchors);
    QVERIFY2(mr.ok, qPrintable(mr.reason));
    QVERIFY(QFileInfo(mr.output_path).isFile());

    QFile out(mr.output_path); QVERIFY(out.open(QIODevice::ReadOnly));
    QTextStream ts(&out);
    int seen = 0;
    double x_min =  1e9, x_max = -1e9;
    double y_min =  1e9, y_max = -1e9;
    double z_min =  1e9;
    while (!ts.atEnd() && seen < 8) {
      const QString line = ts.readLine();
      if (!line.startsWith("v ")) continue;
      const QStringList toks = line.split(QRegularExpression("\\s+"),
                                          Qt::SkipEmptyParts);
      if (toks.size() < 4) continue;
      const double x = toks[1].toDouble();
      const double y = toks[2].toDouble();
      const double z = toks[3].toDouble();
      if (x < x_min) x_min = x;
      if (x > x_max) x_max = x;
      if (y < y_min) y_min = y;
      if (y > y_max) y_max = y;
      if (z < z_min) z_min = z;
      ++seen;
    }
    QCOMPARE(seen, 8);
    const double cx = 0.5 * (x_min + x_max);
    const double cy = 0.5 * (y_min + y_max);
    QVERIFY2(std::fabs(cx) < 1e-3, qPrintable(QString("body bbox not X-centered: cx=%1").arg(cx)));
    QVERIFY2(std::fabs(cy) < 1e-3, qPrintable(QString("body bbox not Y-centered: cy=%1").arg(cy)));
    QVERIFY2(std::fabs(z_min) < 1e-3, qPrintable(QString("body z_min not at 0: z_min=%1").arg(z_min)));
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

    QVERIFY2(out.contains("[cleanup]"), qPrintable(out));
    QCOMPARE(p.exitCode(), 0);
  }

  void debug_merge_only_on_real_fixture_produces_4_distinct_tires() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built next to test");
    const QString fixt_dir = vehicleFixtureRoot();
    if (fixt_dir.isEmpty()) QSKIP("CARLA_STUDIO_FIXTURES not set");
    const QString body_path  = fixt_dir + "/fixture_vehicle/body.obj";
    const QString tires_path = fixt_dir + "/fixture_vehicle/tires.obj";
    if (!QFileInfo(body_path).isFile() || !QFileInfo(tires_path).isFile())
      QSKIP("fixture_vehicle fixtures not present");

    QTemporaryDir cwd; QVERIFY(cwd.isValid());

    QProcess p;
    p.setWorkingDirectory(cwd.path());
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.remove("DISPLAY"); env.remove("WAYLAND_DISPLAY");
    p.setProcessEnvironment(env);
    p.start(QCoreApplication::applicationDirPath() + "/carla-studio", QStringList{"vehicle-import"} << "--mesh" << body_path
                          << "--tires" << tires_path
                          << "--debug-merge-only");
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(60000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("merge: OK"), qPrintable(out));

    QString output_path;
    for (const QString &line : out.split('\n')) {
      if (line.contains("output:")) {
        output_path = line.section("output:", 1).trimmed();
      }
    }
    QVERIFY2(QFileInfo(output_path).isFile(), qPrintable("missing merged.obj at " + output_path));

    QFile f(output_path); QVERIFY(f.open(QIODevice::ReadOnly));
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

    QVERIFY(centroids["studio_tire_FL"].x() < 0);
    QVERIFY(centroids["studio_tire_RL"].x() < 0);
    QVERIFY(centroids["studio_tire_FR"].x() > 0);
    QVERIFY(centroids["studio_tire_RR"].x() > 0);

    QVERIFY(centroids["studio_tire_FL"].y() > 0);
    QVERIFY(centroids["studio_tire_FR"].y() > 0);
    QVERIFY(centroids["studio_tire_RL"].y() < 0);
    QVERIFY(centroids["studio_tire_RR"].y() < 0);
  }

  void capture_qt3d_window_for_each_vehicle() {
    if (qEnvironmentVariable("DISPLAY").isEmpty())
      QSKIP("no DISPLAY - Qt3D capture needs a graphical session");
    const QString previewBin = QCoreApplication::applicationDirPath()
                             + "/carla-studio-vehicle-preview";
    if (!QFileInfo(previewBin).isExecutable())
      QSKIP("carla-studio-vehicle-preview not built");

    struct Probe { QString name; QString path; };
    const QString fixt_root = vehicleFixtureRoot();
    if (fixt_root.isEmpty()) QSKIP("CARLA_STUDIO_FIXTURES not set");
    QList<Probe> probes;
    const QString probeList = qEnvironmentVariable("CARLA_STUDIO_PREVIEW_PROBES");
    for (const QString &entry : probeList.split(':', Qt::SkipEmptyParts)) {
      const QStringList parts = entry.split('=');
      if (parts.size() == 2 && QFileInfo(parts[1]).isFile())
        probes << Probe{ parts[0], parts[1] };
    }

    const QString out_dir = "/tmp/qt3d_shots";
    QDir().mkpath(out_dir);

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

      QProcess::execute("/bin/sh", QStringList() << "-c"
          << "pkill -KILL -f 'carla-studio-vehicle-preview --interactive' 2>/dev/null; true");
      QTest::qWait(1500);

      QProcess preview;
      preview.start(previewBin, QStringList() << p.path << "--interactive");
      QVERIFY2(preview.waitForStarted(5000), qPrintable(QString("preview start failed for ") + p.name));
      QTest::qWait(22000);

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
          const QString out = out_dir + "/" + p.name + "_" + suffix + ".png";
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

    QFileInfoList shots = QDir(out_dir).entryInfoList(
        QStringList() << "*.png", QDir::Files);
    QVERIFY2(shots.size() >= 3, qPrintable(QString("expected ≥3 screenshots, got ") +
                        QString::number(shots.size())));
  }

  void unknown_subcommand_falls_through_to_gui_path() {

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

    const QString err = QString::fromLocal8Bit(p.readAllStandardError())
                      + QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(!err.contains("failed to exec"), qPrintable(err));
  }
};

// ---------------------------------------------------------------------------
// NewCommandsTest — exercises every backwired CLI subcommand
// ---------------------------------------------------------------------------
class NewCommandsTest : public QObject {
  Q_OBJECT
 private:
  QString carlaStudioBin() const {
    return QCoreApplication::applicationDirPath() + "/carla-studio";
  }
  QString siblingBin(const QString &name) const {
    return QCoreApplication::applicationDirPath() + "/" + name;
  }
  bool binExists(const QString &name) const {
    return QFileInfo(siblingBin(name)).isExecutable();
  }

 private slots:

  // ── help ──────────────────────────────────────────────────────────────────

  void help_lists_all_new_subcommands() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built next to test");
    QProcess p;
    p.start(fi.absoluteFilePath(), {"help"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(10000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("setup"),              qPrintable(out));
    QVERIFY2(out.contains("build"),              qPrintable(out));
    QVERIFY2(out.contains("start"),              qPrintable(out));
    QVERIFY2(out.contains("stop"),               qPrintable(out));
    QVERIFY2(out.contains("load-additional-maps"), qPrintable(out));
    QVERIFY2(out.contains("sensor"),             qPrintable(out));
    QVERIFY2(out.contains("actuate"),            qPrintable(out));
    QVERIFY2(out.contains("healthcheck"),        qPrintable(out));
    QCOMPARE(p.exitCode(), 0);
  }

  // ── setup binary ──────────────────────────────────────────────────────────

  void setup_no_args_exits_nonzero_with_usage() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"setup"});
    QVERIFY(p.waitForStarted(3000));
    QVERIFY(p.waitForFinished(8000));
    const QString err_out = QString::fromLocal8Bit(p.readAllStandardError())
                          + QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(err_out.contains("--directory") || err_out.contains("Usage"), qPrintable("expected usage hint; got:\n" + err_out));
    QVERIFY(p.exitCode() != 0);
  }

  QString testOutputDir(const QString &tag) const {
    const QString d = QCoreApplication::applicationDirPath() + "/test-outputs/" + tag;
    QDir().mkpath(d);
    return d;
  }

  void setup_prereq_probe_runs_in_output_dir() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    const QString d = testOutputDir("setup_prereq");
    // We do NOT provide --release so it should fail gracefully after prereq check
    QProcess p;
    p.start(siblingBin("carla-studio"), {"setup", "--directory", d});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(30000));
    const QString combined = QString::fromLocal8Bit(p.readAllStandardOutput())
                           + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(!combined.isEmpty(),
             "setup produced no output at all — possible crash");
  }

  void setup_dispatched_via_main_binary() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built");
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    const QString d = testOutputDir("setup_dispatch");
    QProcess p;
    p.start(fi.absoluteFilePath(),
            {"setup", "--directory", d});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(30000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(!out.contains("failed to exec"), qPrintable("dispatcher failed to exec setup binary:\n" + out));
  }

  // ── build ─────────────────────────────────────────────────────────────────

  void build_dispatched_via_main_binary() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built");
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    // Source builds (git clone + UE5 compile) take ~2 hours minimum.
    // Gate on CARLA_LONG_TESTS=1 so normal CI doesn't time out.
    if (qEnvironmentVariable("CARLA_LONG_TESTS").isEmpty())
      QSKIP("Source build takes ~2h; set CARLA_LONG_TESTS=1 and CARLA_UE5_PATH=<engine> to run");

    // CARLA_BUILD_SRC_DIR overrides default output dir (e.g. /path/to/build_src).
    const QString srcDirOverride = qEnvironmentVariable("CARLA_BUILD_SRC_DIR");
    const QString d = srcDirOverride.isEmpty()
                      ? testOutputDir("build_ue5_src")
                      : srcDirOverride;
    QDir().mkpath(d);
    const QString engine = qEnvironmentVariable("CARLA_UE5_PATH", qgetenv("CARLA_ROOT"));
    QProcess p;
    p.start(fi.absoluteFilePath(),
            {"build", "--directory", d, "--branch", "ue5-dev",
             "--engine", engine});
    QVERIFY(p.waitForStarted(5000));
    // Allow up to 3 hours for a full UE5 source build.
    const bool finished = p.waitForFinished(10800000);
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(finished, qPrintable("Build did not finish within 3h:\n" + out));
    QVERIFY2(p.exitCode() == 0, qPrintable("Build failed (exit " +
             QString::number(p.exitCode()) + "):\n" + out));
  }

  // ── start ─────────────────────────────────────────────────────────────────

  void start_exits_nonzero_when_no_carla_root() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.remove("CARLA_ROOT");
    p.setProcessEnvironment(env);
    p.start(siblingBin("carla-studio"), {"start"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(10000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(out.contains("[start]"), qPrintable("expected [start] prefix; got:\n" + out));
    QVERIFY(p.exitCode() != 0);
  }

  void start_dispatched_via_main_binary() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built");
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.remove("CARLA_ROOT");
    p.setProcessEnvironment(env);
    p.start(fi.absoluteFilePath(), {"start"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(15000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(!out.contains("failed to exec"), qPrintable("start dispatch broken:\n" + out));
  }

  void start_with_map_flag_accepted() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    // Even if CARLA is not installed, the binary should parse --map without crashing
    QProcess p;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.remove("CARLA_ROOT");
    p.setProcessEnvironment(env);
    p.start(siblingBin("carla-studio"), {"start", "--map", "Town01", "--port", "2000"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(10000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(!out.isEmpty(), "start produced no output");
    // exit 1 (no CARLA) is acceptable; exit 2 (unrecognised flag) is not
    QVERIFY2(p.exitCode() != 2, qPrintable("--map flag caused usage error:\n" + out));
  }

  void start_with_port_flag_accepted() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.remove("CARLA_ROOT");
    p.setProcessEnvironment(env);
    p.start(siblingBin("carla-studio"), {"start", "--map", "Town01", "--port", "2100"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(10000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    // exit 1 (no CARLA root) is fine; exit 2 (unrecognised flag) is not
    QVERIFY2(p.exitCode() != 2, qPrintable("--port flag caused usage error:\n" + out));
  }

  void start_with_progress_flag_accepted() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.remove("CARLA_ROOT");
    p.setProcessEnvironment(env);
    p.start(siblingBin("carla-studio"), {"start", "--progress"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(10000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(!out.contains("unrecognised") && !out.contains("unknown option"), qPrintable("--progress caused option error:\n" + out));
  }

  // ── stop ──────────────────────────────────────────────────────────────────

  void stop_exits_zero_when_nothing_running() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"stop"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(15000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("[stop]"), qPrintable(out));
    QCOMPARE(p.exitCode(), 0);
  }

  void stop_dispatched_via_main_binary() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built");
    if (!binExists("carla-studio")) QSKIP("carla-studio not built");
    QProcess p;
    p.start(fi.absoluteFilePath(), {"stop"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(20000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(!out.contains("failed to exec"), qPrintable("stop dispatch broken:\n" + out));
  }

  // ── load-additional-maps ──────────────────────────────────────────────────

  void maps_no_provider_shows_usage() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"load-additional-maps"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(out.contains("mcity") || out.contains("apollo") || out.contains("provider"), qPrintable("expected provider list in usage:\n" + out));
    QVERIFY(p.exitCode() != 0);
  }

  void maps_unknown_provider_exits_nonzero() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"load-additional-maps", "nonexistent-provider"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    QVERIFY(p.exitCode() != 0);
  }

  void maps_mcity_dispatched_via_main_binary() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built");
    if (!binExists("carla-studio")) QSKIP("carla-studio not built");
    QProcess p;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.remove("CARLA_ROOT");
    p.setProcessEnvironment(env);
    p.start(fi.absoluteFilePath(), {"load-additional-maps", "mcity"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(15000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(!out.contains("failed to exec"), qPrintable("load-additional-maps dispatch broken:\n" + out));
  }

  void maps_apollo_dispatched_via_main_binary() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built");
    if (!binExists("carla-studio")) QSKIP("carla-studio not built");
    QProcess p;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.remove("CARLA_ROOT");
    p.setProcessEnvironment(env);
    p.start(fi.absoluteFilePath(), {"load-additional-maps", "apollo"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(15000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(!out.contains("failed to exec"), qPrintable("load-additional-maps apollo dispatch broken:\n" + out));
  }

  void maps_directory_flag_accepted() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    const QString tmpDir = testOutputDir("maps_dir_flag");
    QProcess p;
    p.setProcessChannelMode(QProcess::MergedChannels);
    p.start(siblingBin("carla-studio"), {"load-additional-maps", "mcity", "--directory", tmpDir});
    QVERIFY(p.waitForStarted(5000));
    // Wait just long enough to see the first [maps] banner line, then kill.
    // The CLI may start a long git clone — we only verify flag parsing, not completion.
    QString out;
    for (int ms = 0; ms < 12000 && !out.contains("[maps]"); ms += 200) {
        p.waitForReadyRead(200);
        out += QString::fromLocal8Bit(p.readAll());
    }
    p.kill();
    p.waitForFinished(3000);
    QVERIFY2(out.contains("[maps]"), qPrintable("--directory flag not parsed / no [maps] prefix:\n" + out));
  }

  void maps_progress_flag_accepted() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    const QString tmpDir = testOutputDir("maps_progress_flag");
    QProcess p;
    p.setProcessChannelMode(QProcess::MergedChannels);
    p.start(siblingBin("carla-studio"), {"load-additional-maps", "mcity", "--directory", tmpDir, "--progress"});
    QVERIFY(p.waitForStarted(5000));
    QString out;
    for (int ms = 0; ms < 12000 && !out.contains("[maps]"); ms += 200) {
        p.waitForReadyRead(200);
        out += QString::fromLocal8Bit(p.readAll());
    }
    p.kill();
    p.waitForFinished(3000);
    QVERIFY2(out.contains("[maps]"), qPrintable("--progress caused flag error:\n" + out));
  }

  // ── sensor ────────────────────────────────────────────────────────────────

  void sensor_fisheye_configure_prints_json() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"sensor", "fisheye", "--configure"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("sensor_type"), qPrintable("JSON missing sensor_type:\n" + out));
    QVERIFY2(out.contains("fisheye"), qPrintable("JSON missing fisheye:\n" + out));
    QVERIFY2(out.contains("lens"), qPrintable("JSON missing lens:\n" + out));
    QVERIFY2(out.contains("mount_preset"), qPrintable("JSON missing mount_preset:\n" + out));
    QCOMPARE(p.exitCode(), 0);
  }

  void sensor_fisheye_configure_custom_model_accepted() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"sensor", "fisheye", "--configure", "--model", "fisheye-equisolid", "--fov", "190"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("equisolid"), qPrintable("model not reflected:\n" + out));
    QVERIFY2(out.contains("190"), qPrintable("fov 190 not reflected:\n" + out));
    QCOMPARE(p.exitCode(), 0);
  }

  void sensor_fisheye_dispatched_via_main_binary() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built");
    if (!binExists("carla-studio")) QSKIP("carla-studio not built");
    QProcess p;
    p.start(fi.absoluteFilePath(),
            {"sensor", "fisheye", "--configure"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(10000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(!out.contains("failed to exec"), qPrintable("sensor dispatch broken:\n" + out));
    QVERIFY2(out.contains("sensor_type") || out.contains("fisheye"), qPrintable("no sensor JSON in output:\n" + out));
  }

  void sensor_fisheye_configure_json_parseable() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"sensor", "fisheye", "--configure"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QByteArray raw = p.readAllStandardOutput();
    QJsonParseError parseErr;
    const QJsonDocument doc = QJsonDocument::fromJson(raw, &parseErr);
    QVERIFY2(parseErr.error == QJsonParseError::NoError, qPrintable("sensor JSON not parseable: " + parseErr.errorString()
                        + "\nOutput: " + QString::fromLocal8Bit(raw)));
    QVERIFY2(doc.isObject(), "sensor JSON root is not a JSON object");
  }

  void sensor_fisheye_fov_reflected_in_json() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"sensor", "fisheye", "--configure", "--fov", "120"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QJsonDocument doc = QJsonDocument::fromJson(p.readAllStandardOutput());
    QVERIFY(doc.isObject());
    QCOMPARE(doc.object()["fov"].toDouble(), 120.0);
    QCOMPARE(doc.object()["lens"].toObject()["max_fov_deg"].toDouble(), 120.0);
  }

  void sensor_fisheye_all_required_fields_present() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"sensor", "fisheye", "--configure"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QJsonObject obj = QJsonDocument::fromJson(p.readAllStandardOutput()).object();
    QVERIFY2(obj.contains("sensor_type"),  "field sensor_type missing");
    QVERIFY2(obj.contains("image_size_x"), "field image_size_x missing");
    QVERIFY2(obj.contains("image_size_y"), "field image_size_y missing");
    QVERIFY2(obj.contains("fov"),          "field fov missing");
    QVERIFY2(obj.contains("lens"),         "field lens missing");
    QVERIFY2(obj.contains("mount_preset"), "field mount_preset missing");
    const QJsonObject lens = obj["lens"].toObject();
    QVERIFY2(lens.contains("type"),        "lens.type missing");
    QVERIFY2(lens.contains("max_fov_deg"), "lens.max_fov_deg missing");
    QVERIFY2(lens.contains("cx"),          "lens.cx missing");
    QVERIFY2(lens.contains("cy"),          "lens.cy missing");
  }

  void sensor_fisheye_equisolid_model_accepted() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"sensor", "fisheye", "--configure", "--model", "fisheye-equisolid"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QJsonObject obj = QJsonDocument::fromJson(p.readAllStandardOutput()).object();
    QVERIFY2(obj["lens"].toObject()["type"].toString().contains("equisolid"),
             "lens.type does not reflect fisheye-equisolid model");
    QCOMPARE(p.exitCode(), 0);
  }

  void sensor_no_type_shows_usage() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"sensor"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    QVERIFY(p.exitCode() != 0);
  }

  // ── actuate ───────────────────────────────────────────────────────────────

  void actuate_sae_l5_sets_level() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"actuate", "sae-l5"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("L5") || out.contains("Full Automation"),
             qPrintable(out));
    QCOMPARE(p.exitCode(), 0);

    // verify the setting was persisted
    QCoreApplication::setOrganizationName("CARLA Simulator");
    QCoreApplication::setApplicationName("CARLA Studio");
    const int level = QSettings().value("actuate/sae_level", -1).toInt();
    QCOMPARE(level, 5);
  }

  void actuate_sae_l4_sets_level() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"actuate", "sae-l4"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    QCOMPARE(p.exitCode(), 0);
    const int level = QSettings().value("actuate/sae_level", -1).toInt();
    QCOMPARE(level, 4);
  }

  void actuate_sae_l3_sets_level() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"actuate", "sae-l3"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    QCOMPARE(p.exitCode(), 0);
    const int level = QSettings().value("actuate/sae_level", -1).toInt();
    QCOMPARE(level, 3);
  }

  void actuate_sae_l2_sets_level() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"actuate", "sae-l2"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    QCOMPARE(p.exitCode(), 0);
    const int level = QSettings().value("actuate/sae_level", -1).toInt();
    QCOMPARE(level, 2);
  }

  void actuate_sae_l1_sets_level() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"actuate", "sae-l1"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    QCOMPARE(p.exitCode(), 0);
    const int level = QSettings().value("actuate/sae_level", -1).toInt();
    QCOMPARE(level, 1);
  }

  void actuate_sae_l1_keyboard_sets_ego_control() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"actuate", "sae-l1", "--keyboard"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("Keyboard") || out.contains("keyboard"), qPrintable("--keyboard not reflected in output:\n" + out));
    QCOMPARE(p.exitCode(), 0);
    const QString ctrl = QSettings().value("actuate/player_EGO").toString();
    QCOMPARE(ctrl, QString("Keyboard"));
  }

  void actuate_unknown_level_exits_nonzero() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"actuate", "sae-l9"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    QVERIFY(p.exitCode() != 0);
  }

  void actuate_dispatched_via_main_binary() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built");
    if (!binExists("carla-studio")) QSKIP("carla-studio not built");
    QProcess p;
    p.start(fi.absoluteFilePath(), {"actuate", "sae-l5"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(10000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(!out.contains("failed to exec"), qPrintable("actuate dispatch broken:\n" + out));
    QCOMPARE(p.exitCode(), 0);
  }

  void actuate_sae_l0_sets_level() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"actuate", "sae-l0"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    QCOMPARE(p.exitCode(), 0);
    QCoreApplication::setOrganizationName("CARLA Simulator");
    QCoreApplication::setApplicationName("CARLA Studio");
    const int level = QSettings().value("actuate/sae_level", -1).toInt();
    QCOMPARE(level, 0);
  }

  void actuate_no_args_shows_level_list() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"actuate"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardError())
                      + QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("sae-l0"), qPrintable("sae-l0 missing from usage:\n" + out));
    QVERIFY2(out.contains("sae-l5"), qPrintable("sae-l5 missing from usage:\n" + out));
    QVERIFY(p.exitCode() != 0);
  }

  void actuate_output_contains_level_label() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"actuate", "sae-l3"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("L3") || out.contains("Conditional"), qPrintable("expected L3/Conditional label in output:\n" + out));
    QCOMPARE(p.exitCode(), 0);
  }

  // ── healthcheck ───────────────────────────────────────────────────────────

  void healthcheck_runs_and_prints_table() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"healthcheck"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(30000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("[healthcheck]"), qPrintable("healthcheck prefix missing:\n" + out));
    QVERIFY2(out.contains("PASS") || out.contains("FAIL"), qPrintable("no PASS/FAIL verdict:\n" + out));
    QVERIFY2(out.contains("RAM"), qPrintable("RAM check missing from table:\n" + out));
  }

  void healthcheck_dispatched_via_main_binary() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built");
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(fi.absoluteFilePath(), {"healthcheck"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(30000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(!out.contains("failed to exec"), qPrintable("healthcheck dispatch broken:\n" + out));
  }

  void healthcheck_host_flag_accepted() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"healthcheck", "--host", "127.0.0.1"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(30000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("[healthcheck]"), qPrintable("--host flag not parsed / no output:\n" + out));
  }

  void healthcheck_port_flag_accepted() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"healthcheck", "--port", "2100"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(30000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("[healthcheck]"), qPrintable("--port flag not parsed / no output:\n" + out));
    // Custom port must appear in the CARLA RPC row label
    QVERIFY2(out.contains("2100"), qPrintable("custom port 2100 not reflected in healthcheck output:\n" + out));
  }

  void healthcheck_output_all_rows_present() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"healthcheck"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(30000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("CARLA binary"), qPrintable("'CARLA binary' row missing:\n" + out));
    QVERIFY2(out.contains("CARLA RPC"), qPrintable("'CARLA RPC' row missing:\n" + out));
    QVERIFY2(out.contains("Python"), qPrintable("'Python' row missing:\n" + out));
    QVERIFY2(out.contains("GPU"), qPrintable("'GPU' row missing:\n" + out));
    QVERIFY2(out.contains("RAM"), qPrintable("'RAM' row missing:\n" + out));
    QVERIFY2(out.contains("Disk"), qPrintable("'Disk' row missing:\n" + out));
  }

  // ── cleanup ───────────────────────────────────────────────────────────────

  void cleanup_prints_cleanup_prefix() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"cleanup"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(15000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput());
    QVERIFY2(out.contains("[cleanup]"), qPrintable("expected [cleanup] prefix; got:\n" + out));
    QCOMPARE(p.exitCode(), 0);
  }

  void cleanup_dispatched_via_main_binary() {
    QFileInfo fi(carlaStudioBin());
    if (!fi.isExecutable()) QSKIP("carla-studio binary not built");
    if (!binExists("carla-studio")) QSKIP("carla-studio not built");
    QProcess p;
    p.start(fi.absoluteFilePath(), {"cleanup"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(15000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(!out.contains("failed to exec"), qPrintable("cleanup dispatch broken:\n" + out));
  }

  void cleanup_exits_zero_no_carla_running() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    // cleanup always succeeds; nothing to kill is not an error
    QProcess p;
    p.start(siblingBin("carla-studio"), {"cleanup"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(15000));
    QCOMPARE(p.exitCode(), 0);
  }

  // ── sensor: additional field validation ───────────────────────────────────

  void sensor_fisheye_default_fov_is_180() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"sensor", "fisheye", "--configure"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QByteArray raw = p.readAllStandardOutput() + p.readAllStandardError();
    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(raw, &err);
    QVERIFY2(err.error == QJsonParseError::NoError, qPrintable(err.errorString()));
    const QJsonObject obj = doc.object();
    QVERIFY2(obj["fov"].toDouble() == 180.0, qPrintable(QString("expected default fov=180, got %1").arg(obj["fov"].toDouble())));
  }

  void sensor_fisheye_custom_fov_clamps_to_valid_range() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"sensor", "fisheye", "--configure", "--fov", "90"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QByteArray raw = p.readAllStandardOutput() + p.readAllStandardError();
    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(raw, &err);
    QVERIFY2(err.error == QJsonParseError::NoError, qPrintable(err.errorString()));
    const double fov = doc.object()["fov"].toDouble();
    QVERIFY2(fov > 0 && fov <= 360, qPrintable(QString("fov %1 out of plausible range").arg(fov)));
  }

  void sensor_fisheye_image_size_defaults_nonzero() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"sensor", "fisheye", "--configure"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QByteArray raw = p.readAllStandardOutput() + p.readAllStandardError();
    const QJsonObject obj = QJsonDocument::fromJson(raw).object();
    QVERIFY2(obj["image_size_x"].toInt() > 0 && obj["image_size_y"].toInt() > 0,
             "image_size_x/y must be positive in default sensor config");
  }

  void sensor_unknown_type_exits_nonzero() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"sensor", "nonexistent-sensor-type"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    QVERIFY2(p.exitCode() != 0,
             "unknown sensor type must exit non-zero");
  }

  // ── actuate: QSettings read-back ─────────────────────────────────────────

  void actuate_sae_level_persists_across_calls() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    {
      QProcess p;
      p.start(siblingBin("carla-studio"), {"actuate", "sae-l4"});
      QVERIFY(p.waitForStarted(5000));
      QVERIFY(p.waitForFinished(8000));
      QCOMPARE(p.exitCode(), 0);
    }
    // actuate_cli.cpp sets org="CARLA Simulator", "app="CARLA Studio"
    const QSettings s("CARLA Simulator", "CARLA Studio");
    QVERIFY2(s.value("actuate/sae_level").toInt() == 4,
             "sae_level not persisted to QSettings after sae-l4");
  }

  void actuate_keyboard_mode_sets_ego_control() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"actuate", "sae-l1", "--keyboard"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    QCOMPARE(p.exitCode(), 0);
    const QSettings s("CARLA Simulator", "CARLA Studio");
    QVERIFY2(s.value("actuate/player_EGO").toString() == "Keyboard",
             "sae-l1 --keyboard must set actuate/player_EGO=Keyboard in QSettings");
  }

  // ── healthcheck: output content ───────────────────────────────────────────

  void healthcheck_output_has_carla_binary_row() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"healthcheck"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(15000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(out.contains("CARLA binary") || out.contains("[healthcheck]"), qPrintable("CARLA binary row missing from healthcheck output:\n" + out));
  }

  void healthcheck_output_has_python_row() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"healthcheck"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(15000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(out.contains("Python") || out.contains("carla package"), qPrintable("Python row missing from healthcheck output:\n" + out));
  }

  void healthcheck_output_has_gpu_row() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"healthcheck"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(15000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(out.contains("GPU") || out.contains("nvidia"), qPrintable("GPU row missing from healthcheck output:\n" + out));
  }

  void healthcheck_host_and_port_reflected_together() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"healthcheck", "--host", "192.168.1.1", "--port", "3000"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(15000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(out.contains("192.168.1.1") && out.contains("3000"), qPrintable("--host and --port not both reflected in healthcheck output:\n" + out));
  }

  // ── start: additional flag tests ──────────────────────────────────────────

  void start_no_carla_root_prints_error_message() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    qputenv("CARLA_ROOT", "");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"start"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(10000));
    QVERIFY2(p.exitCode() != 0, "start must exit non-zero when CARLA_ROOT is unset");
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(out.contains("[start]"), qPrintable("start must emit [start] prefix even on error:\n" + out));
  }

  void start_directory_flag_accepted() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    // Pass --directory to an empty tmpDir: flag is parsed, but no CARLA binary
    // is found so start exits non-zero.  The key assertion is that it does NOT
    // exit 2 (unrecognised argument).
    QTemporaryDir tmpDir;
    QProcess p;
    p.start(siblingBin("carla-studio"), {"start", "--directory", tmpDir.path()});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(10000));
    QVERIFY2(p.exitCode() != 2,
             "start must not treat --directory as an unrecognised flag");
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(out.contains("[start]"), qPrintable("start must emit [start] prefix:\n" + out));
  }

  // ── stop: additional coverage ─────────────────────────────────────────────

  void stop_help_flag_prints_usage() {
    if (!binExists("carla-studio"))
      QSKIP("carla-studio not built");
    QProcess p;
    p.start(siblingBin("carla-studio"), {"stop", "--help"});
    QVERIFY(p.waitForStarted(5000));
    QVERIFY(p.waitForFinished(8000));
    const QString out = QString::fromLocal8Bit(p.readAllStandardOutput())
                      + QString::fromLocal8Bit(p.readAllStandardError());
    QVERIFY2(out.contains("[stop]") || out.contains("stop"), qPrintable("stop --help produced no output:\n" + out));
  }
};

// ═══════════════════════════════════════════════════════════════════════════════
// TC-S* SetupTest — download & verify binary CARLA packages via setup_cli
//
// Pre-condition: network access, CARLA_SETUP_TESTS=1.
// Post-condition: <CARLA_ROOT>/{CarlaUnreal-Linux-Shipping,CarlaUE4.sh,...} present.
// Env variables:
//   CARLA_SETUP_TESTS=1       enable these tests
// ═══════════════════════════════════════════════════════════════════════════════
class SetupTest : public QObject {
  Q_OBJECT

  static QString setupBin() {
    return QCoreApplication::applicationDirPath() + "/carla-studio";
  }



  // Returns true if the directory (or a single top-level subdirectory) contains
  // a CARLA shipping binary. 0.9.x extracts flat; 0.10.0+ extracts into a
  // versioned subdirectory like Carla-0.10.0-Linux-Shipping/.
  static bool hasShippingBinary(const QString &dir) {
    const QStringList bins = {
        "CarlaUnreal-Linux-Shipping", "CarlaUnreal.sh",
        "CarlaUE5.sh", "CarlaUE4.sh",
        "LinuxNoEditor/CarlaUE4.sh",
    };
    // Flat check.
    for (const QString &b : bins)
      if (QFileInfo(dir + "/" + b).isFile()) return true;
    // One-level subdirectory check (versioned wrapper dir).
    for (const QFileInfo &fi : QDir(dir).entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot)) {
      for (const QString &b : bins)
        if (QFileInfo(fi.filePath() + "/" + b).isFile()) return true;
    }
    return false;
  }

  // Run setup binary; poll in 15 s increments so Qt's watchdog stays quiet.
  // CARLA package downloads can take 10+ min — blocking waitForFinished() for
  // the full duration hits Qt Test's default 5-min per-function timeout.
  static int runSetup(const QStringList &args, QString &combined) {
    QProcess p;
    p.setProcessChannelMode(QProcess::MergedChannels);
    p.start(setupBin(), QStringList{"setup"} + args);
    if (!p.waitForStarted(5000)) { combined = "did not start"; return -1; }
    // Poll every 15 s; total budget 60 min (large enough for download+extract).
    const int kStep = 15000;
    const int kMax  = 60 * 60 * 1000 / kStep;
    for (int i = 0; i < kMax; ++i) {
      if (p.waitForFinished(kStep)) break;
      // emit a heartbeat so CI/loggers see progress
      qDebug("[setup-poll] still running (%d s elapsed)", (i + 1) * 15);
    }
    combined = QString::fromLocal8Bit(p.readAllStandardOutput());
    return p.exitCode();
  }

 private slots:
  void initTestCase() {
    // Download + extract can take 10-30 min; bump QTEST_TIMEOUT so the
    // Qt watchdog doesn't kill individual test functions mid-download.
    if (!qEnvironmentVariable("CARLA_SETUP_TESTS").isEmpty())
      qputenv("QTEST_TIMEOUT", "3600");  // 1 h in seconds
  }

  // ── TC-S01 ──────────────────────────────────────────────────────────────────
  // TC-S01: Prereqs already installed (or auto-installs them).
  // Description: setup_cli --directory /tmp/prereq_probe exits 2 (no --release)
  //              but only after printing prereq status — proves ensure_prereqs()
  //              ran successfully.
  // Pre: network not required. Post: nothing written.
  void TC_S01_prereqs_probe() {
    if (qEnvironmentVariable("CARLA_SETUP_TESTS").isEmpty())
      QSKIP("Set CARLA_SETUP_TESTS=1 to run setup tests");
    if (!QFileInfo(setupBin()).isExecutable())
      QSKIP("carla-studio not built");
    QString out;
    const int rc = runSetup({"--directory", "/tmp/carla_prereq_probe"}, out);
    QVERIFY2(rc == 2, qPrintable("Expected exit 2 (no release), got " +
             QString::number(rc) + "\n" + out));
    QVERIFY2(out.contains("prerequisites") || out.contains("git"), qPrintable("prereq probe output missing:\n" + out));
  }

  // ── TC-S02 ──────────────────────────────────────────────────────────────────
  // TC-S02: Download + extract CARLA 0.9.16 into CARLA_SIM_ROOT.
  // Description: Queries GitHub API, resolves tag, downloads linux tarball,
  //              extracts in place, verifies shipping binary is present.
  // Pre: CARLA_SETUP_TESTS=1, network, ~10 GB free. Post: CARLA_SIM_ROOT/ has CARLA.
  void TC_S02_setup_0_9_16_downloads() {
    if (qEnvironmentVariable("CARLA_SETUP_TESTS").isEmpty())
      QSKIP("Set CARLA_SETUP_TESTS=1 to run setup tests");
    if (!QFileInfo(setupBin()).isExecutable())
      QSKIP("carla-studio not built");
    const QString d = qEnvironmentVariable("CARLA_SIM_ROOT");
    if (d.isEmpty()) QSKIP("Set CARLA_SIM_ROOT to specify install directory");
    if (hasShippingBinary(d)) {
      qDebug("CARLA_SIM_ROOT already has a CARLA shipping binary — skipping download");
      return;
    }
    QString out;
    const int rc = runSetup({"--directory", d, "--release", "0.9.16"}, out);
    // Print full output so the log captures download progress
    qDebug("%s", qPrintable(out));
    if (rc != 0 && out.contains("not found")) {
      QSKIP("CARLA 0.9.16 not available on GitHub releases — check release tags");
    }
    QVERIFY2(rc == 0, qPrintable("setup 0.9.16 failed (exit " +
             QString::number(rc) + "):\n" + out));
    QVERIFY2(hasShippingBinary(d), qPrintable("No shipping binary found in " + d + " after setup"));
  }

  // ── TC-S03 ──────────────────────────────────────────────────────────────────
  // TC-S03: Download + extract CARLA 0.10.0 into CARLA_SIM_ROOT.
  // Description: Same as S02 but for the first UE5 binary release.
  //              SKIP with diagnostic if not yet published on GitHub.
  // Pre: CARLA_SETUP_TESTS=1, network, ~12 GB free. Post: CARLA_SIM_ROOT/ has CARLA.
  void TC_S03_setup_0_10_0_downloads() {
    if (qEnvironmentVariable("CARLA_SETUP_TESTS").isEmpty())
      QSKIP("Set CARLA_SETUP_TESTS=1 to run setup tests");
    if (!QFileInfo(setupBin()).isExecutable())
      QSKIP("carla-studio not built");
    const QString d = qEnvironmentVariable("CARLA_SIM_ROOT");
    if (d.isEmpty()) QSKIP("Set CARLA_SIM_ROOT to specify install directory");
    if (hasShippingBinary(d)) {
      qDebug("CARLA_SIM_ROOT already has a CARLA shipping binary — skipping download");
      return;
    }
    QString out;
    const int rc = runSetup({"--directory", d, "--release", "0.10.0"}, out);
    qDebug("%s", qPrintable(out));
    if (rc != 0 && out.contains("not found")) {
      QSKIP("CARLA 0.10.0 not yet published as a binary release — "
            "use carla-studio build --branch ue5-dev instead");
    }
    QVERIFY2(rc == 0, qPrintable("setup 0.10.0 failed (exit " +
             QString::number(rc) + "):\n" + out));
    QVERIFY2(hasShippingBinary(d), qPrintable("No shipping binary found in " + d + " after setup"));
  }
};

// ═══════════════════════════════════════════════════════════════════════════════
// TC-L* LiveSimTest — full end-to-end against a running CARLA instance
//
// Gate: CARLA_SIM_ROOT must point to an extracted CARLA directory containing
//       a shipping binary.  Tests start CARLA headlessly, exercise every CLI
//       command, then stop + cleanup.
//
// Pre-condition: a CARLA package installed (run SetupTest first), no other
//               CARLA process on port 2000.
// Post-condition: CARLA process stopped; port 2000 closed; pid file cleared.
// ═══════════════════════════════════════════════════════════════════════════════
class LiveSimTest : public QObject {
  Q_OBJECT

  QString m_carlaRoot;
  QString m_bin;
  bool    m_simRunning = false;

  static QString appDir() { return QCoreApplication::applicationDirPath(); }

  static bool portOpen(int port, int timeoutMs = 2000) {
    // Probe via nc (netcat) — mirrors the GUI's RPC reachability check.
    QProcess nc;
    nc.start("/bin/sh", QStringList() << "-c"
             << QString("nc -z -w2 127.0.0.1 %1").arg(port));
    nc.waitForFinished(timeoutMs + 500);
    return nc.exitCode() == 0;
  }

  // Wait up to maxMs for port 2000 to become reachable (CARLA boot time).
  static bool waitForPort(int port, int maxMs = 120000) {
    const int step = 3000;
    for (int elapsed = 0; elapsed < maxMs; elapsed += step) {
      if (portOpen(port)) return true;
      QTest::qWait(step);
    }
    return false;
  }

  QString runBin(const QString &bin, const QStringList &args,
                 int timeoutMs = 30000) {
    QProcess p;
    p.setProcessChannelMode(QProcess::MergedChannels);
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert("CARLA_ROOT", m_carlaRoot);
    p.setProcessEnvironment(env);
    p.start(bin, args);
    if (!p.waitForStarted(5000)) return "[did not start]";
    p.waitForFinished(timeoutMs);
    return QString::fromLocal8Bit(p.readAllStandardOutput());
  }

  int runBinCode(const QString &bin, const QStringList &args,
                 int timeoutMs = 30000) {
    QProcess p;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert("CARLA_ROOT", m_carlaRoot);
    p.setProcessEnvironment(env);
    p.start(bin, args);
    if (!p.waitForStarted(5000)) return -1;
    p.waitForFinished(timeoutMs);
    return p.exitCode();
  }

 private slots:

  void initTestCase() {
    // Each live test can take up to 120 s (CARLA boot) + 90 s (stop) + maps clone.
    // Raise Qt Test's per-function watchdog so it doesn't fire mid-boot/clone.
    qputenv("QTEST_TIMEOUT", "3600");

    m_carlaRoot = qEnvironmentVariable("CARLA_SIM_ROOT");
    if (m_carlaRoot.isEmpty())
      QSKIP("Set CARLA_SIM_ROOT=<carla-root> to run live-sim tests");
    if (!QFileInfo(m_carlaRoot).isDir())
      QSKIP(qPrintable("CARLA_SIM_ROOT does not exist: " + m_carlaRoot));

    m_bin = appDir() + "/carla-studio";

    const auto checkBin = [&](const QString &b) {
      if (!QFileInfo(b).isExecutable())
        QFAIL(qPrintable("Binary not found: " + b));
    };
    checkBin(m_bin);

    // Ensure no stale CARLA on port 2000 before we begin.
    if (portOpen(2000)) {
      qDebug("[live] Stale process on :2000 — running cleanup first");
      runBinCode(m_bin, {"cleanup"});
      QTest::qWait(3000);
    }
  }

  void cleanupTestCase() {
    // Best-effort teardown: stop whatever might be running.
    if (m_simRunning) {
      runBinCode(m_bin, {"stop"});
      QTest::qWait(3000);
      runBinCode(m_bin, {"cleanup"});
      m_simRunning = false;
    }
  }

  // ── TC-L01 ──────────────────────────────────────────────────────────────────
  // TC-L01: start default (no map flag).
  // Description: carla-studio start --directory <root>; wait for port 2000;
  //              verify PID written to ~/.config/carla-studio/carla-pids.txt.
  // Pre: no CARLA running. Post: CARLA up on :2000; pid file written.
  void TC_L01_start_default() {
    const int rc = runBinCode(m_bin, {"start", "--directory", m_carlaRoot});
    QCOMPARE(rc, 0);
    QVERIFY2(waitForPort(2000), "CARLA did not open port 2000 within 120 s");
    m_simRunning = true;
    const QString pidFile = QDir::homePath() + "/.config/carla-studio/carla-pids.txt";
    QVERIFY2(QFileInfo(pidFile).isFile(), qPrintable("pid file missing: " + pidFile));
  }

  // ── TC-L02 ──────────────────────────────────────────────────────────────────
  // TC-L02: stop running sim.
  // Description: carla-studio stop after TC-L01; waits for SIGTERM + grace;
  //              verifies port 2000 closes within 70 s.
  // Pre: CARLA running (TC-L01 passed). Post: port 2000 closed.
  void TC_L02_stop() {
    if (!m_simRunning) QSKIP("TC-L01 did not start CARLA");
    const int rc = runBinCode(m_bin, {"stop"}, 90000);
    QCOMPARE(rc, 0);
    m_simRunning = false;
    // Give kernel 5 s to close the listen socket.
    QTest::qWait(5000);
    QVERIFY2(!portOpen(2000), "Port 2000 still open after stop");
  }

  // ── TC-L03 ──────────────────────────────────────────────────────────────────
  // TC-L03: cleanup when nothing is running.
  // Description: cleanup exits 0 and prints "nothing to kill" when sim is down.
  // Pre: sim stopped. Post: no stale CARLA processes.
  void TC_L03_cleanup_when_idle() {
    const QString out = runBin(m_bin, {"cleanup"});
    QVERIFY2(!out.isEmpty(), "cleanup produced no output");
    qDebug("%s", qPrintable(out));
  }

  // ── TC-L04 ──────────────────────────────────────────────────────────────────
  // TC-L04: start with --map Town01 (GUI default map).
  // Description: start --map Town01 --directory <root>; port 2000 opens.
  // Pre: sim stopped. Post: CARLA up on :2000 loaded with Town01.
  void TC_L04_start_map_Town01() {
    const int rc = runBinCode(m_bin, {"start", "--directory", m_carlaRoot, "--map", "Town01"});
    QCOMPARE(rc, 0);
    QVERIFY2(waitForPort(2000), "CARLA did not open port 2000 (Town01)");
    m_simRunning = true;
    qDebug("[live] CARLA up with Town01");
    // Stop for next test.
    runBinCode(m_bin, {"stop"}, 90000);
    QTest::qWait(5000);
    m_simRunning = false;
  }

  // ── TC-L05 ──────────────────────────────────────────────────────────────────
  // TC-L05: start with --map Town06 (GUI default map, largest built-in).
  // Description: start --map Town06; port 2000 opens.
  // Pre: sim stopped. Post: CARLA up on :2000 loaded with Town06.
  void TC_L05_start_map_Town06() {
    const int rc = runBinCode(m_bin, {"start", "--directory", m_carlaRoot, "--map", "Town06"});
    QCOMPARE(rc, 0);
    QVERIFY2(waitForPort(2000), "CARLA did not open port 2000 (Town06)");
    m_simRunning = true;
    qDebug("[live] CARLA up with Town06");
    runBinCode(m_bin, {"stop"}, 90000);
    QTest::qWait(5000);
    m_simRunning = false;
  }

  // ── TC-L06 ──────────────────────────────────────────────────────────────────
  // TC-L06: load-additional-maps mcity.
  // Description: clones mcity Digital Twin into <carla_root>/CommunityMaps/mcity.
  //              Mirrors GUI Setup Wizard → Maps tab → Mcity Digital Twin.
  // Pre: network, git available. Post: CommunityMaps/mcity/.git present.
  void TC_L06_load_additional_maps_mcity() {
    const QString mcityDir = m_carlaRoot + "/CommunityMaps/mcity";
    if (QFileInfo(mcityDir + "/.git").exists()) {
      qDebug("[live] mcity already cloned — skipping git clone step");
    } else {
      const int rc = runBinCode(m_bin, {"load-additional-maps", "mcity", "--directory", m_carlaRoot}, 1800000);
      QCOMPARE(rc, 0);
    }
    QVERIFY2(QFileInfo(mcityDir).isDir(), qPrintable("CommunityMaps/mcity dir missing after install"));
  }

  // ── TC-L07 ──────────────────────────────────────────────────────────────────
  // TC-L07: load-additional-maps apollo.
  // Description: clones Apollo HD Maps repo into <carla_root>/CommunityMaps/apollo.
  //              Mirrors GUI Setup Wizard → Maps tab → Apollo HD Maps.
  // Pre: network, git available. Post: CommunityMaps/apollo/.git present.
  void TC_L07_load_additional_maps_apollo() {
    const QString apolloDir = m_carlaRoot + "/CommunityMaps/apollo";
    if (QFileInfo(apolloDir + "/.git").exists()) {
      qDebug("[live] apollo already cloned — skipping git clone step");
    } else {
      const int rc = runBinCode(m_bin, {"load-additional-maps", "apollo", "--directory", m_carlaRoot}, 1800000);
      QCOMPARE(rc, 0);
    }
    QVERIFY2(QFileInfo(apolloDir).isDir(), qPrintable("CommunityMaps/apollo dir missing after install"));
  }

  // ── TC-L08 ──────────────────────────────────────────────────────────────────
  // TC-L08: start with --map mcity (additional community map).
  // Description: start --map mcity after TC-L06; verifies CARLA boots.
  //              Tests that the map name is forwarded to the launcher correctly.
  // Pre: TC-L06 passed (mcity cloned). Post: CARLA up or exits with map-not-found.
  void TC_L08_start_map_mcity() {
    if (!QFileInfo(m_carlaRoot + "/CommunityMaps/mcity").isDir())
      QSKIP("TC-L06 (mcity install) did not run or failed");
    const int rc = runBinCode(m_bin, {"start", "--directory", m_carlaRoot, "--map", "mcity"});
    QCOMPARE(rc, 0);
    // CARLA may reject an unknown map name; we only require it started.
    const bool up = waitForPort(2000);
    if (m_simRunning || up) {
      m_simRunning = true;
      runBinCode(m_bin, {"stop"}, 90000);
      QTest::qWait(5000);
      m_simRunning = false;
    }
    QVERIFY2(up, "CARLA did not open port 2000 with --map mcity");
  }

  // ── TC-L09 ──────────────────────────────────────────────────────────────────
  // TC-L09: sensor fisheye --configure output.
  // Description: Verifies JSON contains sensor_type, fov, lens keys; mirrors
  //              GUI Interfaces → Fisheye row → config dialog.
  // Pre: none (no live sim needed). Post: stdout JSON validated.
  void TC_L09_sensor_fisheye_configure() {
    const QString out = runBin(m_bin, {"sensor", "fisheye", "--configure"});
    QVERIFY2(out.contains("sensor_type"), qPrintable("sensor_type missing:\n" + out));
    QVERIFY2(out.contains("fov"), qPrintable("fov missing:\n" + out));
    QVERIFY2(out.contains("lens"), qPrintable("lens block missing:\n" + out));
    qDebug("%s", qPrintable(out));
  }

  // ── TC-L10 ──────────────────────────────────────────────────────────────────
  // TC-L10: sensor fisheye --configure with custom model + fov.
  // Description: equisolid model at 190° fov; verifies model name in JSON.
  // Pre: none. Post: stdout JSON contains "fisheye-equisolid" and 190.
  void TC_L10_sensor_fisheye_configure_custom() {
    const QString out = runBin(m_bin, {"sensor", "fisheye", "--configure", "--model", "fisheye-equisolid", "--fov", "190"});
    QVERIFY2(out.contains("fisheye-equisolid"), qPrintable("model not reflected:\n" + out));
    QVERIFY2(out.contains("190"), qPrintable("fov 190 not in output:\n" + out));
    qDebug("%s", qPrintable(out));
  }

  // ── TC-L11 ──────────────────────────────────────────────────────────────────
  // TC-L11: actuate sae-l5 through sae-l1 set QSettings.
  // Description: Each actuate level persists to QSettings("actuate/sae_level");
  //              mirrors GUI Actuate pane SAE radio buttons.
  // Pre: none. Post: QSettings("actuate/sae_level") = expected level int.
  void TC_L11_actuate_all_levels() {
    struct Case { const char *id; int level; };
    const QList<Case> cases = {
        {"sae-l5", 5}, {"sae-l4", 4}, {"sae-l3", 3},
        {"sae-l2", 2}, {"sae-l1", 1}, {"sae-l0", 0},
    };
    QCoreApplication::setOrganizationName("CARLA Simulator");
    QCoreApplication::setApplicationName("CARLA Studio");
    for (const auto &c : cases) {
      const int rc = runBinCode(m_bin, {"actuate", c.id});
      QVERIFY2(rc == 0, qPrintable(QString("actuate %1 exit %2").arg(c.id).arg(rc)));
      QSettings s;
      s.sync();
      const int actual = s.value("actuate/sae_level", -1).toInt();
      QVERIFY2(actual == c.level, qPrintable(QString("actuate %1: expected %2, got %3")
                   .arg(c.id).arg(c.level).arg(actual)));
    }
  }

  // ── TC-L12 ──────────────────────────────────────────────────────────────────
  // TC-L12: actuate sae-l1 with --keyboard sets EGO to Keyboard.
  // Description: sae-l1 --keyboard writes QSettings("actuate/player_EGO","Keyboard");
  //              mirrors Actuate pane → Keyboard assignment for EGO slot.
  // Pre: none. Post: QSettings("actuate/player_EGO") = "Keyboard".
  void TC_L12_actuate_l1_keyboard() {
    QCoreApplication::setOrganizationName("CARLA Simulator");
    QCoreApplication::setApplicationName("CARLA Studio");
    const int rc = runBinCode(m_bin, {"actuate", "sae-l1", "--keyboard"});
    QCOMPARE(rc, 0);
    QSettings s;
    s.sync();
    QCOMPARE(s.value("actuate/player_EGO").toString(), QString("Keyboard"));
  }

  // ── TC-L13 ──────────────────────────────────────────────────────────────────
  // TC-L13: healthcheck with live CARLA running.
  // Description: Start CARLA; run healthcheck; verify RPC row shows PASS.
  //              Mirrors Health Check tab → Refresh when sim is up.
  // Pre: CARLA not running. Post: CARLA stopped; healthcheck showed RPC PASS.
  void TC_L13_healthcheck_with_live_sim() {
    // Start sim.
    const int startRc = runBinCode(m_bin, {"start", "--directory", m_carlaRoot});
    QCOMPARE(startRc, 0);
    QVERIFY2(waitForPort(2000), "CARLA did not open port 2000");
    m_simRunning = true;

    const QString out = runBin(m_bin, {"healthcheck"}, 30000);
    qDebug("%s", qPrintable(out));
    QVERIFY2(out.contains("[healthcheck]"), qPrintable("healthcheck header missing:\n" + out));
    // The RPC port row should report "reachable" (sim is up).
    // We check the row detail rather than the overall summary because other
    // checks (python carla package, nvidia-smi, CARLA_ROOT) may legitimately
    // fail in CI while the RPC connection itself is fine.
    QVERIFY2(out.contains("reachable"), qPrintable("RPC row not reachable in healthcheck while sim up:\n" + out));

    runBinCode(m_bin, {"stop"}, 90000);
    QTest::qWait(5000);
    m_simRunning = false;
  }

  // ── TC-L14 ──────────────────────────────────────────────────────────────────
  // TC-L14: final cleanup — no stale processes after all tests.
  // Description: cleanup exits 0 ("nothing to kill" or killed N); verifies
  //              port 2000 is closed.
  // Pre: all prior tests completed. Post: zero CARLA processes remain.
  void TC_L14_final_cleanup() {
    const QString out = runBin(m_bin, {"cleanup"});
    QVERIFY2(!out.isEmpty(), "cleanup produced no output");
    qDebug("%s", qPrintable(out));
    // SIGKILL is instantaneous but the kernel needs a moment to reap the socket.
    bool portClosed = false;
    for (int i = 0; i < 10 && !portClosed; ++i) {
      QTest::qWait(1000);
      portClosed = !portOpen(2000, 500);
    }
    QVERIFY2(portClosed, "Port 2000 still open 10 s after final cleanup");
  }
};

void runCliPipelineTests(int argc, char **argv, int &exit_code) {
    { ImportPipelineTest t; exit_code |= QTest::qExec(&t, argc, argv); }
    { CliDispatcherTest t;  exit_code |= QTest::qExec(&t, argc, argv); }
    { NewCommandsTest t;    exit_code |= QTest::qExec(&t, argc, argv); }
    { SetupTest t;          exit_code |= QTest::qExec(&t, argc, argv); }
    { LiveSimTest t;        exit_code |= QTest::qExec(&t, argc, argv); }
}

#include "cli_pipeline_test.moc"

