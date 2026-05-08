// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include <QtTest/QtTest>

#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QProcess>
#include <QTemporaryDir>
#include <QTextStream>
#include <QSet>
#include <QVector>
#include <QVector3D>
#include <cmath>

#include "core/PlayerSlots.h"
#include "core/SensorMountKey.h"
#include "core/StudioAppContext.h"
#include "utils/ResourceFit.h"
#include "vehicle_import/BPAutopicker.h"
#include "vehicle_import/MeshAABB.h"
#include "vehicle_import/MeshAnalysis.h"
#include "vehicle_import/ImportMode.h"
#include "vehicle_import/KitBundler.h"
#include "vehicle_import/MergedSpecBuilder.h"
#include "vehicle_import/MeshGeometry.h"
#include "vehicle_import/NameSanitizer.h"
#include "vehicle_import/ObjSanitizer.h"
#include "vehicle_import/VehicleSpec.h"

class CarlaStudioAppTest : public QObject {
  Q_OBJECT

private:
  QString sourcePath;

  QString readSource() const {
    QFile file(sourcePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      return QString();
    }
    QTextStream in(&file);
    return in.readAll();
  }

private slots:
  void initTestCase() {
#ifdef CARLA_STUDIO_SOURCE_PATH
    sourcePath = QStringLiteral(CARLA_STUDIO_SOURCE_PATH);
#else
  sourcePath = QStringLiteral("../../CarlaStudio/src/app/carla_studio.cpp");
#endif

    if (!QFile::exists(sourcePath)) {
      QSKIP("app carla_studio.cpp not found; set CARLA_STUDIO_SOURCE_PATH or run tests from Apps/CarlaStudio.", SkipSingle);
    }
  }

  void startup_status_states_present() {
    const QString source = readSource();
    QVERIFY2(!source.isEmpty(), "Unable to read app carla_studio.cpp");

    QVERIFY2(source.contains("setSimulationStatus(\"Idle\")"), "Missing Idle status state");
    QVERIFY2(source.contains("setSimulationStatus(\"Running\")"), "Missing Running status state");
    QVERIFY2(source.contains("setSimulationStatus(\"Stopped\")"), "Missing Stopped status state");
    QVERIFY2(source.contains("setSimulationStatus(\"Initializing\")"), "Missing Initializing status state");
  }

  void stop_timeout_force_exit_present() {
    const QString source = readSource();
    QVERIFY2(!source.isEmpty(), "Unable to read app carla_studio.cpp");

    QVERIFY2(source.contains("forceStopTimer->start(60000)"), "Missing 60-second force-stop timeout");
  }

  void live_process_table_ui_present() {
    const QString source = readSource();
    QVERIFY2(!source.isEmpty(), "Unable to read app carla_studio.cpp");

    QVERIFY2(source.contains("carlaProcessTable"), "Missing live process table widget");
    QVERIFY2(source.contains("\"PID\" << \"Process\" << \"CPU\" << \"Memory\" << \"GPU\""),
             "Live process table header columns changed");
    QVERIFY2(source.contains("trackedCarlaPids"), "Missing tracked PID store");
    QVERIFY2(source.contains("echo $!"), "START path does not capture child PID");
  }

  void sensor_assembly_rotation_range_present() {
    const QString source = readSource();
    QVERIFY2(!source.isEmpty(), "Unable to read app carla_studio.cpp");

    QVERIFY2(source.contains("rx->setRange(0.0, 359.0)"), "rx range is not 0..359");
    QVERIFY2(source.contains("ry->setRange(0.0, 359.0)"), "ry range is not 0..359");
    QVERIFY2(source.contains("rz->setRange(0.0, 359.0)"), "rz range is not 0..359");
  }

  void resource_fit_run_cmd_timeout_returns_empty() {
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString scriptPath = dir.path() + "/sleeper.sh";
    {
      QFile f(scriptPath);
      QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream(&f) << "#!/bin/sh\nsleep 5\n";
    }
    QFile::setPermissions(scriptPath,
        QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner |
        QFile::ReadUser  | QFile::WriteUser  | QFile::ExeUser);

    QElapsedTimer t;
    t.start();
    const QString out = carla_studio::utils::detail::runCmd(
        "/bin/sh", QStringList() << scriptPath, 200);
    const qint64 elapsed = t.elapsed();
    QVERIFY2(out.isEmpty(), "runCmd should return empty QString on timeout");
    QVERIFY2(elapsed < 5000,
        qPrintable(QString("runCmd should escalate terminate->kill, elapsed=%1ms").arg(elapsed)));
  }

  void resource_fit_run_cmd_normal_completion() {
    const QString out = carla_studio::utils::detail::runCmd(
        "/bin/sh", QStringList() << "-c" << "printf hello", 3000);
    QCOMPARE(out, QStringLiteral("hello"));
  }

  void resource_fit_detect_os_pretty_non_empty() {
    const QString os = carla_studio::utils::detail::detectOsPretty();
    QVERIFY2(!os.isEmpty(), "detectOsPretty returned empty string on Linux host");
  }

  void resource_fit_detect_cpu_non_zero() {
    QString model;
    int logical = 0;
    int physical = 0;
    carla_studio::utils::detail::detectCpu(&model, &logical, &physical);
    QVERIFY2(logical > 0,  qPrintable(QString("logical=%1").arg(logical)));
    QVERIFY2(physical > 0, qPrintable(QString("physical=%1").arg(physical)));
  }

  void resource_fit_detect_ram_non_zero() {
    std::int64_t total = 0;
    std::int64_t free = 0;
    carla_studio::utils::detail::detectRam(&total, &free);
    QVERIFY2(total > 0, qPrintable(QString("ram total bytes=%1").arg(total)));
  }

  void studio_app_context_workspace_round_trip() {
    carla::studio::core::StudioAppContext ctx;
    QVERIFY(!ctx.HasWorkspaceRoot());
    QVERIFY(ctx.GetWorkspaceRoot().isEmpty());
    ctx.SetWorkspaceRoot(QStringLiteral("/tmp/ws-root"));
    QCOMPARE(ctx.GetWorkspaceRoot(), QStringLiteral("/tmp/ws-root"));
    QVERIFY(ctx.HasWorkspaceRoot());
    ctx.SetWorkspaceRoot(QString());
    QVERIFY(!ctx.HasWorkspaceRoot());
  }

  void studio_app_context_ui_only_round_trip() {
    carla::studio::core::StudioAppContext ctx;
    QCOMPARE(ctx.IsUiOnlyMode(), false);
    ctx.SetUiOnlyMode(true);
    QCOMPARE(ctx.IsUiOnlyMode(), true);
    ctx.SetUiOnlyMode(false);
    QCOMPARE(ctx.IsUiOnlyMode(), false);
  }

  void sensor_mount_key_instance_one_is_bare() {
    using carla::studio::core::sensorMountKey;
    QCOMPARE(sensorMountKey("Fisheye", 1), QStringLiteral("Fisheye"));
  }

  void sensor_mount_key_instance_n_appends_hash() {
    using carla::studio::core::sensorMountKey;
    QCOMPARE(sensorMountKey("Fisheye", 4), QStringLiteral("Fisheye#4"));
    QCOMPARE(sensorMountKey("Lidar",   2), QStringLiteral("Lidar#2"));
  }

  void sensor_mount_key_non_positive_clamps_to_bare() {
    using carla::studio::core::sensorMountKey;
    QCOMPARE(sensorMountKey("Fisheye",  0), QStringLiteral("Fisheye"));
    QCOMPARE(sensorMountKey("Fisheye", -1), QStringLiteral("Fisheye"));
  }

  void player_slots_default_names_full_range() {
    using carla::studio::core::defaultPlayerName;
    QCOMPARE(defaultPlayerName(0),  QStringLiteral("EGO"));
    QCOMPARE(defaultPlayerName(1),  QStringLiteral("POV.01"));
    QCOMPARE(defaultPlayerName(2),  QStringLiteral("POV.02"));
    QCOMPARE(defaultPlayerName(3),  QStringLiteral("POV.03"));
    QCOMPARE(defaultPlayerName(4),  QStringLiteral("POV.04"));
    QCOMPARE(defaultPlayerName(5),  QStringLiteral("POV.05"));
    QCOMPARE(defaultPlayerName(6),  QStringLiteral("POV.06"));
    QCOMPARE(defaultPlayerName(7),  QStringLiteral("POV.07"));
    QCOMPARE(defaultPlayerName(8),  QStringLiteral("POV.08"));
    QCOMPARE(defaultPlayerName(9),  QStringLiteral("POV.09"));
    QCOMPARE(defaultPlayerName(10), QStringLiteral("POV.10"));
    QCOMPARE(defaultPlayerName(11), QStringLiteral("V2X.01"));
    QCOMPARE(defaultPlayerName(12), QStringLiteral("V2X.02"));
    QCOMPARE(defaultPlayerName(13), QStringLiteral("V2X.03"));
    QCOMPARE(defaultPlayerName(14), QStringLiteral("V2X.04"));
    QCOMPARE(defaultPlayerName(15), QStringLiteral("V2X.05"));
    QCOMPARE(defaultPlayerName(16), QStringLiteral("V2X.06"));
  }

  void player_slots_out_of_range_returns_empty() {
    using carla::studio::core::defaultPlayerName;
    QVERIFY(defaultPlayerName(-1).isEmpty());
    QVERIFY(defaultPlayerName(17).isEmpty());
    QVERIFY(defaultPlayerName(99).isEmpty());
  }

  void mesh_aabb_feeds_extents() {
    using carla_studio::vehicle_import::MeshAABB;
    MeshAABB bb;
    QVERIFY(!bb.valid);
    bb.feed(1.0f, 2.0f, -3.0f);
    bb.feed(-4.0f, 5.0f,  6.0f);
    QVERIFY(bb.valid);
    QCOMPARE(bb.xMin, -4.0f); QCOMPARE(bb.xMax, 1.0f);
    QCOMPARE(bb.yMin,  2.0f); QCOMPARE(bb.yMax, 5.0f);
    QCOMPARE(bb.zMin, -3.0f); QCOMPARE(bb.zMax, 6.0f);
  }

  void mesh_aabb_meters_blender_obj_scales_to_cm() {
    using carla_studio::vehicle_import::MeshAABB;
    MeshAABB bb;
    bb.feed(0.6f, 0.0f,  0.0f);
    bb.feed(3.0f, 2.1f, -6.9f);
    bb.feed(0.6f, 0.0f, -0.95f);
    bb.detectConventions("obj");
    QCOMPARE(bb.scaleToCm, 100.0f);
    QCOMPARE(bb.upAxis,    1);
  }

  void mesh_aabb_already_cm_obj_no_scale() {
    using carla_studio::vehicle_import::MeshAABB;
    MeshAABB bb;
    bb.feed(  0.f,  0.f,  0.f);
    bb.feed(300.f, 200.f, 600.f);
    bb.detectConventions("obj");
    QCOMPARE(bb.scaleToCm, 1.0f);
  }

  void mesh_aabb_to_ue_projects_axes() {
    using carla_studio::vehicle_import::MeshAABB;
    MeshAABB bb;
    bb.feed(0.0f, 0.0f, 0.0f);
    bb.feed(2.0f, 1.0f, 5.0f);
    bb.detectConventions("obj");
    float xLo, xHi, yLo, yHi, zLo, zHi;
    bb.toUE(xLo, xHi, yLo, yHi, zLo, zHi);
    QVERIFY(xHi - xLo >= yHi - yLo);
    QVERIFY(xHi - xLo >= zHi - zLo);
  }

  void bp_autopicker_chooses_close_match() {
    using carla_studio::vehicle_import::pickClosestBaseVehicleBP;
    const QString p = pickClosestBaseVehicleBP(490.0f);
    QVERIFY(p.contains("USDImportTemplates"));
    QVERIFY(p.endsWith("BaseUSDImportVehicle"));
  }

  void bp_autopicker_extreme_inputs_still_resolve() {
    using carla_studio::vehicle_import::pickClosestBaseVehicleBP;
    QCOMPARE(pickClosestBaseVehicleBP(0.0f),     pickClosestBaseVehicleBP(99999.0f));
    QVERIFY(pickClosestBaseVehicleBP(0.0f).endsWith("BaseUSDImportVehicle"));
  }

  void obj_sanitizer_writes_scaled_copy_and_drops_bad_faces() {
    using carla_studio::vehicle_import::sanitizeOBJ;
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString in = dir.path() + "/in.obj";
    {
      QFile f(in);
      QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      s << "v 1.0 2.0 3.0\nv 4.0 5.0 6.0\nv 7.0 8.0 9.0\n";
      s << "f 1 2 3\nf 1\nvt 0.0 0.0\n";
    }
    const auto rep = sanitizeOBJ(in, 100.0f);
    QVERIFY(rep.ok);
    QCOMPARE(rep.skippedFaceLines, 1);
    QFile out(rep.outputPath);
    QVERIFY(out.open(QIODevice::ReadOnly | QIODevice::Text));
    const QString body = QTextStream(&out).readAll();
    QVERIFY(body.contains("v 100"));
    QVERIFY(body.contains("v 400"));
    QVERIFY(!body.contains("\nf 1\n"));
  }

  void name_sanitizer_known_failures() {
    using carla_studio::vehicle_import::sanitizeVehicleName;
    QCOMPARE(sanitizeVehicleName("Mercedes+Benz+GLS+580"),
             QStringLiteral("mercedes_benz_gls_580"));
    QCOMPARE(sanitizeVehicleName("uploads_files_2787791_Mercedes+Benz+GLS+580"),
             QStringLiteral("uploads_files_2787791_mercedes_benz_gls_580"));
    QCOMPARE(sanitizeVehicleName("Tesla Model S"),
             QStringLiteral("tesla_model_s"));
    QCOMPARE(sanitizeVehicleName("vehicle.audi.tt"),
             QStringLiteral("vehicle_audi_tt"));
  }

  void name_sanitizer_edges() {
    using carla_studio::vehicle_import::sanitizeVehicleName;
    QCOMPARE(sanitizeVehicleName(""), QString());
    QCOMPARE(sanitizeVehicleName("   spaces   "), QStringLiteral("spaces"));
    QCOMPARE(sanitizeVehicleName("--__--"), QString());
    QCOMPARE(sanitizeVehicleName("MyTruck"), QStringLiteral("mytruck"));
    QCOMPARE(sanitizeVehicleName("a__b___c"), QStringLiteral("a_b_c"));
    QCOMPARE(sanitizeVehicleName("___leading"), QStringLiteral("leading"));
    QCOMPARE(sanitizeVehicleName("trailing___"), QStringLiteral("trailing"));
  }

  void name_sanitizer_leading_digit_prefix() {
    using carla_studio::vehicle_import::sanitizeVehicleName;
    QCOMPARE(sanitizeVehicleName("2024_Tesla"), QStringLiteral("v_2024_tesla"));
    QCOMPARE(sanitizeVehicleName("9life"), QStringLiteral("v_9life"));
  }

  void name_sanitizer_unicode_strip() {
    using carla_studio::vehicle_import::sanitizeVehicleName;
    QCOMPARE(sanitizeVehicleName(QString::fromUtf8("\xF0\x9F\x9A\x97" "car")),
             QStringLiteral("car"));
    QCOMPARE(sanitizeVehicleName(QString::fromUtf8("Citro" "\xC3\xAB" "n_C4")),
             QStringLiteral("citro_n_c4"));
  }

  void name_sanitizer_length_cap() {
    using carla_studio::vehicle_import::sanitizeVehicleName;
    using carla_studio::vehicle_import::kMaxVehicleNameLength;
    const QString very_long(kMaxVehicleNameLength + 25, QLatin1Char('a'));
    const QString result = sanitizeVehicleName(very_long);
    QCOMPARE(result.size(), kMaxVehicleNameLength);
    QVERIFY(!result.endsWith(QLatin1Char('_')));
  }

  void name_sanitizer_idempotent() {
    using carla_studio::vehicle_import::sanitizeVehicleName;
    using carla_studio::vehicle_import::isCanonicalVehicleName;
    const QStringList samples = {
      "Mercedes+Benz+GLS+580", "Tesla Model S", "2024_Tesla",
      "MyTruck", "vehicle.audi.tt", "  spaces  ",
    };
    for (const QString &s : samples) {
      const QString first  = sanitizeVehicleName(s);
      const QString second = sanitizeVehicleName(first);
      QCOMPARE(second, first);
      if (!first.isEmpty()) QVERIFY(isCanonicalVehicleName(first));
    }
  }

  void name_validator_states() {
    using carla_studio::vehicle_import::VehicleNameValidator;
    VehicleNameValidator v(nullptr);
    QString s; int pos = 0;
    s = "";              QCOMPARE(v.validate(s, pos), QValidator::Intermediate);
    s = "my_truck";      QCOMPARE(v.validate(s, pos), QValidator::Acceptable);
    s = "MyTruck";       QCOMPARE(v.validate(s, pos), QValidator::Intermediate);
    s = "Bad+Name";      QCOMPARE(v.validate(s, pos), QValidator::Intermediate);
    QString fix = "Bad+Name"; v.fixup(fix); QCOMPARE(fix, QStringLiteral("bad_name"));
  }

  void mesh_analysis_four_wheel_sedan() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString p = dir.filePath("sedan.obj");
    {
      QFile f(p);
      QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      s << "v -2.2 -0.8 0.4\nv  2.2 -0.8 0.4\nv  2.2  0.8 0.4\nv -2.2  0.8 0.4\n";
      s << "v -2.2 -0.8 1.4\nv  2.2 -0.8 1.4\nv  2.2  0.8 1.4\nv -2.2  0.8 1.4\n";
      s << "f 1 2 3\nf 1 3 4\nf 5 6 7\nf 5 7 8\n";
      s << "f 1 2 6\nf 1 6 5\nf 2 3 7\nf 2 7 6\n";
      s << "f 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n";
      auto wheel = [&](float cx, float cy, float cz, float r, float halfW, int base) {
        QVector<QVector3D> verts;
        for (int i = 0; i < 8; ++i) {
          const float a = i * 6.283185f / 8.0f;
          verts.append(QVector3D(cx + r * std::cos(a), cy - halfW, cz + r * std::sin(a)));
          verts.append(QVector3D(cx + r * std::cos(a), cy + halfW, cz + r * std::sin(a)));
        }
        for (const auto &v : verts) s << "v " << v.x() << " " << v.y() << " " << v.z() << "\n";
        for (int i = 0; i < 8; ++i) {
          const int a = base + 2 * i + 1;
          const int b = base + 2 * ((i + 1) % 8) + 1;
          const int c = a + 1, d = b + 1;
          s << "f " << a << " " << b << " " << c << "\n";
          s << "f " << b << " " << d << " " << c << "\n";
        }
      };
      wheel(-1.6f, -0.85f, 0.35f, 0.35f, 0.13f,  8);
      wheel( 1.6f, -0.85f, 0.35f, 0.35f, 0.13f, 24);
      wheel(-1.6f,  0.85f, 0.35f, 0.35f, 0.13f, 40);
      wheel( 1.6f,  0.85f, 0.35f, 0.35f, 0.13f, 56);
    }
    const MeshGeometry g = loadMeshGeometryOBJ(p);
    QVERIFY(g.valid);
    QVERIFY(g.faceCount() > 16);
    const MeshAnalysisResult r = analyzeMesh(g, 100.0f);
    QVERIFY(r.ok);
    QVERIFY2(r.hasFourWheels, qPrintable(QString("got %1 wheels").arg(r.wheels.size())));
    QCOMPARE(static_cast<int>(r.wheels.size()), 4);
    for (const WheelCandidate &w : r.wheels) {
      QVERIFY(w.radius > 25.0f && w.radius < 50.0f);
    }
    QCOMPARE(r.sizeClass, SizeClass::Sedan);
    QVERIFY(!r.smallVehicleNeedsWheelShape);
  }

  void mesh_analysis_tiny_robot_flags_wheelshape() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString p = dir.filePath("robot.obj");
    {
      QFile f(p);
      QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      s << "v -0.5 -0.3 0.10\nv  0.5 -0.3 0.10\nv  0.5  0.3 0.10\nv -0.5  0.3 0.10\n";
      s << "v -0.5 -0.3 0.30\nv  0.5 -0.3 0.30\nv  0.5  0.3 0.30\nv -0.5  0.3 0.30\n";
      s << "f 1 2 3\nf 1 3 4\nf 5 6 7\nf 5 7 8\n";
      s << "f 1 2 6\nf 1 6 5\nf 2 3 7\nf 2 7 6\n";
      s << "f 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n";
      auto sphere = [&](float cx, float cy, float cz, float r, int base) {
        QVector<QVector3D> verts;
        for (int i = 0; i < 8; ++i) {
          const float a = i * 6.283185f / 8.0f;
          verts.append(QVector3D(cx + r * std::cos(a), cy, cz + r * std::sin(a)));
          verts.append(QVector3D(cx + r * std::cos(a), cy + 0.05f, cz + r * std::sin(a)));
        }
        for (const auto &v : verts) s << "v " << v.x() << " " << v.y() << " " << v.z() << "\n";
        for (int i = 0; i < 8; ++i) {
          const int a = base + 2 * i + 1;
          const int b = base + 2 * ((i + 1) % 8) + 1;
          const int c = a + 1, d = b + 1;
          s << "f " << a << " " << b << " " << c << "\n";
          s << "f " << b << " " << d << " " << c << "\n";
        }
      };
      sphere(-0.4f, -0.3f, 0.05f, 0.05f,  8);
      sphere( 0.4f, -0.3f, 0.05f, 0.05f, 24);
      sphere(-0.4f,  0.3f, 0.05f, 0.05f, 40);
      sphere( 0.4f,  0.3f, 0.05f, 0.05f, 56);
    }
    const MeshGeometry g = loadMeshGeometryOBJ(p);
    QVERIFY(g.valid);
    const MeshAnalysisResult r = analyzeMesh(g, 100.0f);
    QVERIFY(r.ok);
    QVERIFY(r.hasFourWheels);
    QCOMPARE(r.sizeClass, SizeClass::TinyRobot);
    QVERIFY(r.smallVehicleNeedsWheelShape);
  }

  static void writeVehicleObj(QTextStream &s, float halfLenM, float halfWidM,
                              float groundM, float roofM, float wheelR, float wheelHalfW) {
    s << "v " << -halfLenM << " " << -halfWidM << " " << groundM << "\n";
    s << "v "  << halfLenM << " " << -halfWidM << " " << groundM << "\n";
    s << "v "  << halfLenM << " "  << halfWidM << " " << groundM << "\n";
    s << "v " << -halfLenM << " "  << halfWidM << " " << groundM << "\n";
    s << "v " << -halfLenM << " " << -halfWidM << " " << roofM   << "\n";
    s << "v "  << halfLenM << " " << -halfWidM << " " << roofM   << "\n";
    s << "v "  << halfLenM << " "  << halfWidM << " " << roofM   << "\n";
    s << "v " << -halfLenM << " "  << halfWidM << " " << roofM   << "\n";
    s << "f 1 2 3\nf 1 3 4\nf 5 6 7\nf 5 7 8\n";
    s << "f 1 2 6\nf 1 6 5\nf 2 3 7\nf 2 7 6\n";
    s << "f 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n";
    auto wheel = [&](float cx, float cy, float cz, int base) {
      for (int i = 0; i < 8; ++i) {
        const float a = i * 6.283185f / 8.0f;
        s << "v " << cx + wheelR * std::cos(a) << " " << cy - wheelHalfW
          << " " << cz + wheelR * std::sin(a) << "\n";
        s << "v " << cx + wheelR * std::cos(a) << " " << cy + wheelHalfW
          << " " << cz + wheelR * std::sin(a) << "\n";
      }
      for (int i = 0; i < 8; ++i) {
        const int a = base + 2 * i + 1;
        const int b = base + 2 * ((i + 1) % 8) + 1;
        const int c = a + 1, d = b + 1;
        s << "f " << a << " " << b << " " << c << "\n";
        s << "f " << b << " " << d << " " << c << "\n";
      }
    };
    const float wheelOffsetX = halfLenM * 0.72f;
    const float wheelOffsetY = halfWidM + wheelHalfW * 0.5f;
    const float wheelCz      = groundM + wheelR;
    wheel(-wheelOffsetX, -wheelOffsetY, wheelCz,  8);
    wheel( wheelOffsetX, -wheelOffsetY, wheelCz, 24);
    wheel(-wheelOffsetX,  wheelOffsetY, wheelCz, 40);
    wheel( wheelOffsetX,  wheelOffsetY, wheelCz, 56);
  }

  void mesh_analysis_size_classes_full_coverage() {
    using namespace carla_studio::vehicle_import;
    struct Case {
      const char *name;
      float halfLenM, halfWidM, groundM, roofM, wheelR, wheelHalfW;
      SizeClass expect;
      bool      expectSmallFlag;
    };
    const Case cases[] = {
      {"compact", 1.5f,  0.65f, 0.30f, 1.30f, 0.28f, 0.10f, SizeClass::Compact, false},
      {"sedan",   2.2f,  0.80f, 0.40f, 1.40f, 0.35f, 0.13f, SizeClass::Sedan,   false},
      {"suv",     2.6f,  0.95f, 0.45f, 1.75f, 0.40f, 0.14f, SizeClass::Suv,     false},
      {"truck",   3.5f,  1.20f, 0.50f, 2.50f, 0.55f, 0.18f, SizeClass::Truck,   false},
      {"bus",     5.0f,  1.30f, 0.55f, 3.00f, 0.52f, 0.17f, SizeClass::Bus,     false},
    };
    for (const Case &c : cases) {
      QTemporaryDir dir;
      QVERIFY(dir.isValid());
      const QString p = dir.filePath(QString("%1.obj").arg(c.name));
      {
        QFile f(p);
        QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
        QTextStream s(&f);
        writeVehicleObj(s, c.halfLenM, c.halfWidM, c.groundM, c.roofM, c.wheelR, c.wheelHalfW);
      }
      const MeshGeometry g = loadMeshGeometryOBJ(p);
      QVERIFY2(g.valid, c.name);
      const MeshAnalysisResult r = analyzeMesh(g, 100.0f);
      QVERIFY2(r.ok, c.name);
      QVERIFY2(r.hasFourWheels,
               qPrintable(QString("%1: got %2 wheels").arg(c.name).arg(r.wheels.size())));
      QVERIFY2(r.sizeClass == c.expect,
               qPrintable(QString("%1: class=%2 expect=%3")
                            .arg(c.name)
                            .arg(sizeClassName(r.sizeClass))
                            .arg(sizeClassName(c.expect))));
      QCOMPARE(r.smallVehicleNeedsWheelShape, c.expectSmallFlag);
    }
  }

  void preset_per_size_class_distinct() {
    using namespace carla_studio::vehicle_import;
    const SizeClass classes[] = { SizeClass::TinyRobot, SizeClass::Compact,
                                  SizeClass::Sedan, SizeClass::Suv,
                                  SizeClass::Truck, SizeClass::Bus };
    QSet<QString> seenTags;
    float prevMass = -1.0f;
    for (SizeClass c : classes) {
      const SizePreset p = presetForSizeClass(c);
      QVERIFY2(p.mass > prevMass, qPrintable(sizeClassName(c)));
      QVERIFY2(!p.torqueCurveTag.isEmpty(), qPrintable(sizeClassName(c)));
      QVERIFY2(!seenTags.contains(p.torqueCurveTag), qPrintable(p.torqueCurveTag));
      seenTags.insert(p.torqueCurveTag);
      prevMass = p.mass;
    }
  }

  void vehicle_spec_round_trip_json() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString p = dir.filePath("sedan.obj");
    {
      QFile f(p);
      QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      writeVehicleObj(s, 2.2f, 0.80f, 0.40f, 1.40f, 0.35f, 0.13f);
    }
    const MeshGeometry      g = loadMeshGeometryOBJ(p);
    const MeshAnalysisResult r = analyzeMesh(g, 100.0f);
    const VehicleSpec spec = buildSpecFromAnalysis(r, "my_truck", p, 100.0f, 2, 0);
    QCOMPARE(spec.name, QStringLiteral("my_truck"));
    QCOMPARE(spec.sizeClass, SizeClass::Sedan);
    QVERIFY(spec.hasFourWheels);
    QCOMPARE(spec.mass, 1500.0f);
    const QJsonObject j = specToJson(spec);
    QCOMPARE(j.value("vehicle_name").toString(), QStringLiteral("my_truck"));
    QCOMPARE(j.value("size_class").toString(),   QStringLiteral("sedan"));
    QCOMPARE(j.value("has_four_wheels").toBool(), true);
    QVERIFY(j.contains("wheel_fl"));
    QVERIFY(j.contains("chassis_aabb_cm"));
  }

  void import_mode_lite_when_nothing_set() {
    using namespace carla_studio::vehicle_import;
    const ImportModeStatus s = determineImportMode("", "", false);
    QCOMPARE(s.mode, ImportMode::Lite);
    QVERIFY(!s.hasUproject); QVERIFY(!s.hasEngine); QVERIFY(!s.pluginAligned);
    QVERIFY(!s.forcedLite);
    QVERIFY(s.reason.contains("missing"));
  }
  void import_mode_force_lite_overrides() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir d; QVERIFY(d.isValid());
    const QString uproj = d.filePath("Carla.uproject");
    { QFile f(uproj); QVERIFY(f.open(QIODevice::WriteOnly)); f.write("{}"); }
    const ImportModeStatus s = determineImportMode(uproj, "/no/such/engine", true);
    QCOMPARE(s.mode, ImportMode::Lite);
    QVERIFY(s.forcedLite);
    QVERIFY(s.reason.contains("forced"));
  }
  void import_mode_lite_when_uproject_only() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir d; QVERIFY(d.isValid());
    const QString uproj = d.filePath("Carla.uproject");
    { QFile f(uproj); QVERIFY(f.open(QIODevice::WriteOnly)); f.write("{}"); }
    const ImportModeStatus s = determineImportMode(uproj, "", false);
    QCOMPARE(s.mode, ImportMode::Lite);
    QVERIFY(s.hasUproject);
    QVERIFY(!s.hasEngine);
  }
  void kit_bundler_writes_full_bundle() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir d; QVERIFY(d.isValid());
    const QString meshPath = d.filePath("source.obj");
    {
      QFile f(meshPath);
      QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      writeVehicleObj(s, 2.2f, 0.80f, 0.40f, 1.40f, 0.35f, 0.13f);
    }
    const MeshGeometry g = loadMeshGeometryOBJ(meshPath);
    const MeshAnalysisResult r = analyzeMesh(g, 100.0f);
    VehicleSpec spec = buildSpecFromAnalysis(r, "test_sedan", meshPath, 100.0f, 2, 0);
    const QString outDir = d.filePath("kit_out");
    const KitBundleResult res = writeVehicleKit(spec, outDir);
    QVERIFY2(res.ok, qPrintable(res.detail));
    QVERIFY(QFileInfo(outDir + "/spec.json").isFile());
    QVERIFY(QFileInfo(outDir + "/README.md").isFile());
    QVERIFY(QFileInfo(outDir + "/VehicleParameters.entry.json").isFile());
    QVERIFY(QFileInfo(outDir + "/mesh.obj").isFile());

    QFile sj(outDir + "/spec.json"); QVERIFY(sj.open(QIODevice::ReadOnly));
    QJsonParseError e; QJsonDocument doc = QJsonDocument::fromJson(sj.readAll(), &e);
    QCOMPARE(e.error, QJsonParseError::NoError);
    QCOMPARE(doc.object().value("vehicle_name").toString(), QStringLiteral("test_sedan"));
    QCOMPARE(doc.object().value("size_class").toString(), QStringLiteral("sedan"));

    QFile rd(outDir + "/README.md"); QVERIFY(rd.open(QIODevice::ReadOnly | QIODevice::Text));
    const QString readme = QTextStream(&rd).readAll();
    QVERIFY(readme.contains("Vehicle Kit"));
    QVERIFY(readme.contains("test_sedan"));
    QVERIFY(readme.contains("Detected vehicle profile"));
    QVERIFY(readme.contains("Per-wheel measurements"));

    QFile ej(outDir + "/VehicleParameters.entry.json"); QVERIFY(ej.open(QIODevice::ReadOnly));
    QJsonDocument edoc = QJsonDocument::fromJson(ej.readAll(), &e);
    QCOMPARE(e.error, QJsonParseError::NoError);
    const QJsonArray vehicles = edoc.object().value("Vehicles").toArray();
    QCOMPARE(vehicles.size(), 1);
    QCOMPARE(vehicles.at(0).toObject().value("Model").toString(), QStringLiteral("test_sedan"));
    QVERIFY(vehicles.at(0).toObject().value("Class").toString().endsWith("BP_test_sedan_C"));
  }
  void kit_bundler_handles_small_vehicle_warning() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir d; QVERIFY(d.isValid());
    const QString meshPath = d.filePath("robot.obj");
    {
      QFile f(meshPath);
      QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      writeVehicleObj(s, 0.50f, 0.30f, 0.05f, 0.35f, 0.05f, 0.02f);
    }
    const MeshGeometry g = loadMeshGeometryOBJ(meshPath);
    const MeshAnalysisResult r = analyzeMesh(g, 100.0f);
    QVERIFY(r.smallVehicleNeedsWheelShape);
    VehicleSpec spec = buildSpecFromAnalysis(r, "tiny_robot", meshPath, 100.0f, 2, 0);
    const QString outDir = d.filePath("kit");
    QVERIFY(writeVehicleKit(spec, outDir).ok);
    QFile rd(outDir + "/README.md");
    QVERIFY(rd.open(QIODevice::ReadOnly | QIODevice::Text));
    const QString readme = QTextStream(&rd).readAll();
    QVERIFY(readme.contains("tiny_robot"));
    QVERIFY(readme.contains("Chaos") || readme.contains("WheelShape") || readme.contains("smaller"));
  }

  void merge_combined_mode_when_body_has_wheels_only() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir d; QVERIFY(d.isValid());
    const QString p = d.filePath("sedan.obj");
    {
      QFile f(p); QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      writeVehicleObj(s, 2.2f, 0.80f, 0.40f, 1.40f, 0.35f, 0.13f);
    }
    const MeshGeometry g = loadMeshGeometryOBJ(p);
    const MeshAnalysisResult r = analyzeMesh(g, 100.0f);
    const VehicleSpec body = buildSpecFromAnalysis(r, "s", p, 100.0f, 2, 0);
    const MergeResult mr = mergeBodyAndWheels(body, WheelTemplate{}, false);
    QVERIFY2(mr.ok, qPrintable(mr.reason));
    QCOMPARE(mr.mode, IngestionMode::Combined);
    QVERIFY(mr.spec.hasFourWheels);
  }
  void merge_tire_swap_replaces_radii() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir d; QVERIFY(d.isValid());
    const QString p = d.filePath("sedan.obj");
    {
      QFile f(p); QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      writeVehicleObj(s, 2.2f, 0.80f, 0.40f, 1.40f, 0.35f, 0.13f);
    }
    const MeshGeometry g = loadMeshGeometryOBJ(p);
    const MeshAnalysisResult r = analyzeMesh(g, 100.0f);
    const VehicleSpec body = buildSpecFromAnalysis(r, "s", p, 100.0f, 2, 0);
    WheelTemplate tpl; tpl.valid = true; tpl.radius = 42.0f; tpl.width = 22.0f;
    const MergeResult mr = mergeBodyAndWheels(body, tpl, true);
    QVERIFY2(mr.ok, qPrintable(mr.reason));
    QCOMPARE(mr.mode, IngestionMode::TireSwap);
    for (const WheelSpec &w : mr.spec.wheels) {
      QCOMPARE(w.radius, 42.0f);
      QCOMPARE(w.width,  22.0f);
    }
    QVERIFY(!mr.spec.smallVehicleNeedsWheelShape);
  }
  void merge_body_plus_tires_derives_anchors() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir d; QVERIFY(d.isValid());
    const QString p = d.filePath("body.obj");
    {
      QFile f(p); QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      s << "v -2.0 -0.8 0.4\nv  2.0 -0.8 0.4\nv  2.0  0.8 0.4\nv -2.0  0.8 0.4\n";
      s << "v -2.0 -0.8 1.4\nv  2.0 -0.8 1.4\nv  2.0  0.8 1.4\nv -2.0  0.8 1.4\n";
      s << "f 1 2 3\nf 1 3 4\nf 5 6 7\nf 5 7 8\n";
      s << "f 1 2 6\nf 1 6 5\nf 2 3 7\nf 2 7 6\n";
      s << "f 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n";
    }
    const MeshGeometry g = loadMeshGeometryOBJ(p);
    const MeshAnalysisResult r = analyzeMesh(g, 100.0f);
    const VehicleSpec body = buildSpecFromAnalysis(r, "chassis", p, 100.0f, 2, 0);
    QVERIFY(!body.hasFourWheels);
    WheelTemplate tpl; tpl.valid = true; tpl.radius = 30.0f; tpl.width = 18.0f;
    const MergeResult mr = mergeBodyAndWheels(body, tpl, true);
    QVERIFY2(mr.ok, qPrintable(mr.reason));
    QCOMPARE(mr.mode, IngestionMode::BodyPlusTires);
    QVERIFY(mr.spec.hasFourWheels);
    for (const WheelSpec &w : mr.spec.wheels) {
      QCOMPARE(w.radius, 30.0f);
    }
  }
  void merge_rejects_body_only_without_tires() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir d; QVERIFY(d.isValid());
    const QString p = d.filePath("body.obj");
    {
      QFile f(p); QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      s << "v -2.0 -0.8 0.4\nv  2.0 -0.8 0.4\nv  2.0  0.8 0.4\nv -2.0  0.8 0.4\n";
      s << "v -2.0 -0.8 1.4\nv  2.0 -0.8 1.4\nv  2.0  0.8 1.4\nv -2.0  0.8 1.4\n";
      s << "f 1 2 3\nf 1 3 4\nf 5 6 7\nf 5 7 8\n";
      s << "f 1 2 6\nf 1 6 5\nf 2 3 7\nf 2 7 6\n";
      s << "f 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n";
    }
    const MeshGeometry g = loadMeshGeometryOBJ(p);
    const MeshAnalysisResult r = analyzeMesh(g, 100.0f);
    const VehicleSpec body = buildSpecFromAnalysis(r, "chassis", p, 100.0f, 2, 0);
    const MergeResult mr = mergeBodyAndWheels(body, WheelTemplate{}, false);
    QVERIFY(!mr.ok);
    QVERIFY(mr.reason.contains("would not be drivable"));
  }

  void mesh_analysis_body_only_no_wheels() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString p = dir.filePath("body.obj");
    {
      QFile f(p);
      QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      s << "v -2.0 -0.8 0.4\nv  2.0 -0.8 0.4\nv  2.0  0.8 0.4\nv -2.0  0.8 0.4\n";
      s << "v -2.0 -0.8 1.4\nv  2.0 -0.8 1.4\nv  2.0  0.8 1.4\nv -2.0  0.8 1.4\n";
      s << "f 1 2 3\nf 1 3 4\nf 5 6 7\nf 5 7 8\n";
      s << "f 1 2 6\nf 1 6 5\nf 2 3 7\nf 2 7 6\n";
      s << "f 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n";
    }
    const MeshGeometry g = loadMeshGeometryOBJ(p);
    QVERIFY(g.valid);
    const MeshAnalysisResult r = analyzeMesh(g, 100.0f);
    QVERIFY(r.ok);
    QVERIFY(!r.hasFourWheels);
  }

  void binary_smoke_startup_offscreen() {
    const QString binaryPath = qEnvironmentVariable("CARLA_STUDIO_BIN", QStringLiteral("./carla-studio"));
    if (!QFile::exists(binaryPath)) {
      QSKIP("carla-studio binary not found; set CARLA_STUDIO_BIN to run smoke test.", SkipSingle);
    }

    QProcess process;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert("QT_QPA_PLATFORM", "offscreen");
    process.setProcessEnvironment(env);
    process.start(binaryPath, QStringList());

    QVERIFY2(process.waitForStarted(5000), "carla-studio failed to start");
    QTest::qWait(1200);
    QVERIFY2(process.state() == QProcess::Running, "carla-studio exited unexpectedly during smoke run");

    process.terminate();
    if (!process.waitForFinished(3000)) {
      process.kill();
      process.waitForFinished(3000);
    }
  }
};

QTEST_MAIN(CarlaStudioAppTest)
#include "CarlaStudioAppTest.moc"
