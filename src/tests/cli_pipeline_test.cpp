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

    QVERIFY2(seenLog.contains("Sending spec to editor"),
             qPrintable("expected log to mention spec send; got:\n" + seenLog));
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
    QCOMPARE(res.error_detail,
             QString("user cancelled at confirm-start gate"));

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
        QVERIFY2(d > 50.f,
                 qPrintable(QString("tire %1 and %2 too close: %3 cm")
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
    QVERIFY2(std::fabs(cx) < 1e-3,
             qPrintable(QString("body bbox not X-centered: cx=%1").arg(cx)));
    QVERIFY2(std::fabs(cy) < 1e-3,
             qPrintable(QString("body bbox not Y-centered: cy=%1").arg(cy)));
    QVERIFY2(std::fabs(z_min) < 1e-3,
             qPrintable(QString("body z_min not at 0: z_min=%1").arg(z_min)));
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
    p.start(QCoreApplication::applicationDirPath() + "/carla-studio-cli_vehicle-import",
            QStringList() << "--mesh" << body_path
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
    QVERIFY2(QFileInfo(output_path).isFile(),
             qPrintable("missing merged.obj at " + output_path));

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
      QVERIFY2(preview.waitForStarted(5000),
               qPrintable(QString("preview start failed for ") + p.name));
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
    QVERIFY2(shots.size() >= 3,
             qPrintable(QString("expected ≥3 screenshots, got ") +
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

void runCliPipelineTests(int argc, char **argv, int &exit_code) {
    { ImportPipelineTest t; exit_code |= QTest::qExec(&t, argc, argv); }
    { CliDispatcherTest t; exit_code |= QTest::qExec(&t, argc, argv); }
}

#include "cli_pipeline_test.moc"

