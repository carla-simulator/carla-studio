// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
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

#include "traffic/player_slots.h"
#include "vehicle/mount/sensor_mount_key.h"
#include "core/studio_app_context.h"
#include "utils/resource_fit.h"
#include "vehicle/import/bp_autopicker.h"
#include "vehicle/import/mesh_aabb.h"
#include "vehicle/import/mesh_analysis.h"
#include "vehicle/import/import_mode.h"
#include "vehicle/import/kit_bundler.h"
#include "vehicle/import/merged_spec_builder.h"
#include "vehicle/import/mesh_geometry.h"
#include "vehicle/import/name_sanitizer.h"
#include "vehicle/import/obj_sanitizer.h"
#include "vehicle/import/vehicle_spec.h"

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

    QVERIFY2(source.contains("carla_process_table"), "Missing live process table widget");
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
    const QString out = carla_studio::utils::detail::run_cmd(
        "/bin/sh", QStringList() << scriptPath, 200);
    const qint64 elapsed = t.elapsed();
    QVERIFY2(out.isEmpty(), "run_cmd should return empty QString on timeout");
    QVERIFY2(elapsed < 5000,
        qPrintable(QString("run_cmd should escalate terminate->kill, elapsed=%1ms").arg(elapsed)));
  }

  void resource_fit_run_cmd_normal_completion() {
    const QString out = carla_studio::utils::detail::run_cmd(
        "/bin/sh", QStringList() << "-c" << "printf hello", 3000);
    QCOMPARE(out, QStringLiteral("hello"));
  }

  void resource_fit_detect_os_pretty_non_empty() {
    const QString os = carla_studio::utils::detail::detect_os_pretty();
    QVERIFY2(!os.isEmpty(), "detect_os_pretty returned empty string on Linux host");
  }

  void resource_fit_detect_cpu_non_zero() {
    QString model;
    int logical = 0;
    int physical = 0;
    carla_studio::utils::detail::detect_cpu(&model, &logical, &physical);
    QVERIFY2(logical > 0,  qPrintable(QString("logical=%1").arg(logical)));
    QVERIFY2(physical > 0, qPrintable(QString("physical=%1").arg(physical)));
  }

  void resource_fit_detect_ram_non_zero() {
    std::int64_t total = 0;
    std::int64_t free = 0;
    carla_studio::utils::detail::detect_ram(&total, &free);
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
    using carla::studio::core::sensor_mount_key;
    QCOMPARE(sensor_mount_key("Fisheye", 1), QStringLiteral("Fisheye"));
  }

  void sensor_mount_key_instance_n_appends_hash() {
    using carla::studio::core::sensor_mount_key;
    QCOMPARE(sensor_mount_key("Fisheye", 4), QStringLiteral("Fisheye#4"));
    QCOMPARE(sensor_mount_key("Lidar",   2), QStringLiteral("Lidar#2"));
  }

  void sensor_mount_key_non_positive_clamps_to_bare() {
    using carla::studio::core::sensor_mount_key;
    QCOMPARE(sensor_mount_key("Fisheye",  0), QStringLiteral("Fisheye"));
    QCOMPARE(sensor_mount_key("Fisheye", -1), QStringLiteral("Fisheye"));
  }

  void player_slots_default_names_full_range() {
    using carla::studio::core::default_player_name;
    QCOMPARE(default_player_name(0),  QStringLiteral("EGO"));
    QCOMPARE(default_player_name(1),  QStringLiteral("POV.01"));
    QCOMPARE(default_player_name(2),  QStringLiteral("POV.02"));
    QCOMPARE(default_player_name(3),  QStringLiteral("POV.03"));
    QCOMPARE(default_player_name(4),  QStringLiteral("POV.04"));
    QCOMPARE(default_player_name(5),  QStringLiteral("POV.05"));
    QCOMPARE(default_player_name(6),  QStringLiteral("POV.06"));
    QCOMPARE(default_player_name(7),  QStringLiteral("POV.07"));
    QCOMPARE(default_player_name(8),  QStringLiteral("POV.08"));
    QCOMPARE(default_player_name(9),  QStringLiteral("POV.09"));
    QCOMPARE(default_player_name(10), QStringLiteral("POV.10"));
    QCOMPARE(default_player_name(11), QStringLiteral("V2X.01"));
    QCOMPARE(default_player_name(12), QStringLiteral("V2X.02"));
    QCOMPARE(default_player_name(13), QStringLiteral("V2X.03"));
    QCOMPARE(default_player_name(14), QStringLiteral("V2X.04"));
    QCOMPARE(default_player_name(15), QStringLiteral("V2X.05"));
    QCOMPARE(default_player_name(16), QStringLiteral("V2X.06"));
  }

  void player_slots_out_of_range_returns_empty() {
    using carla::studio::core::default_player_name;
    QVERIFY(default_player_name(-1).isEmpty());
    QVERIFY(default_player_name(17).isEmpty());
    QVERIFY(default_player_name(99).isEmpty());
  }

  void mesh_aabb_feeds_extents() {
    using carla_studio::vehicle_import::MeshAABB;
    MeshAABB bb;
    QVERIFY(!bb.valid);
    bb.feed(1.0f, 2.0f, -3.0f);
    bb.feed(-4.0f, 5.0f,  6.0f);
    QVERIFY(bb.valid);
    QCOMPARE(bb.x_min, -4.0f); QCOMPARE(bb.x_max, 1.0f);
    QCOMPARE(bb.y_min,  2.0f); QCOMPARE(bb.y_max, 5.0f);
    QCOMPARE(bb.z_min, -3.0f); QCOMPARE(bb.z_max, 6.0f);
  }

  void mesh_aabb_meters_blender_obj_scales_to_cm() {
    using carla_studio::vehicle_import::MeshAABB;
    MeshAABB bb;
    bb.feed(0.6f, 0.0f,  0.0f);
    bb.feed(3.0f, 2.1f, -6.9f);
    bb.feed(0.6f, 0.0f, -0.95f);
    bb.detect_conventions("obj");
    QCOMPARE(bb.scale_to_cm, 100.0f);
    QCOMPARE(bb.up_axis,    1);
  }

  void mesh_aabb_already_cm_obj_no_scale() {
    using carla_studio::vehicle_import::MeshAABB;
    MeshAABB bb;
    bb.feed(  0.f,  0.f,  0.f);
    bb.feed(300.f, 200.f, 600.f);
    bb.detect_conventions("obj");
    QCOMPARE(bb.scale_to_cm, 1.0f);
  }

  void mesh_aabb_to_ue_projects_axes() {
    using carla_studio::vehicle_import::MeshAABB;
    MeshAABB bb;
    bb.feed(0.0f, 0.0f, 0.0f);
    bb.feed(2.0f, 1.0f, 5.0f);
    bb.detect_conventions("obj");
    float xLo, xHi, yLo, yHi, zLo, zHi;
    bb.to_ue(xLo, xHi, yLo, yHi, zLo, zHi);
    QVERIFY(xHi - xLo >= yHi - yLo);
    QVERIFY(xHi - xLo >= zHi - zLo);
  }

  void bp_autopicker_chooses_close_match() {
    using carla_studio::vehicle_import::pick_closest_base_vehicle_bp;
    const QString p = pick_closest_base_vehicle_bp(490.0f);
    QVERIFY(p.contains("USDImportTemplates"));
    QVERIFY(p.endsWith("BaseUSDImportVehicle"));
  }

  void bp_autopicker_extreme_inputs_still_resolve() {
    using carla_studio::vehicle_import::pick_closest_base_vehicle_bp;
    QCOMPARE(pick_closest_base_vehicle_bp(0.0f),     pick_closest_base_vehicle_bp(99999.0f));
    QVERIFY(pick_closest_base_vehicle_bp(0.0f).endsWith("BaseUSDImportVehicle"));
  }

  void obj_sanitizer_writes_scaled_copy_and_drops_bad_faces() {
    using carla_studio::vehicle_import::sanitize_obj;
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
    const auto rep = sanitize_obj(in, 100.0f);
    QVERIFY(rep.ok);
    QCOMPARE(rep.skipped_face_lines, 1);
    QFile out(rep.output_path);
    QVERIFY(out.open(QIODevice::ReadOnly | QIODevice::Text));
    const QString body = QTextStream(&out).readAll();
    QVERIFY(body.contains("v 100"));
    QVERIFY(body.contains("v 400"));
    QVERIFY(!body.contains("\nf 1\n"));
  }

  void name_sanitizer_known_failures() {
    using carla_studio::vehicle_import::sanitize_vehicle_name;
    QCOMPARE(sanitize_vehicle_name("Brand+Model+Name+2024"),
             QStringLiteral("brand_model_name_2024"));
    QCOMPARE(sanitize_vehicle_name("uploads_files_1234567_Brand+Model+Name+2024"),
             QStringLiteral("uploads_files_1234567_brand_model_name_2024"));
    QCOMPARE(sanitize_vehicle_name("Tesla Model S"),
             QStringLiteral("tesla_model_s"));
    QCOMPARE(sanitize_vehicle_name("vehicle.audi.tt"),
             QStringLiteral("vehicle_audi_tt"));
  }

  void name_sanitizer_edges() {
    using carla_studio::vehicle_import::sanitize_vehicle_name;
    QCOMPARE(sanitize_vehicle_name(""), QString());
    QCOMPARE(sanitize_vehicle_name("   spaces   "), QStringLiteral("spaces"));
    QCOMPARE(sanitize_vehicle_name("--__--"), QString());
    QCOMPARE(sanitize_vehicle_name("MyTruck"), QStringLiteral("mytruck"));
    QCOMPARE(sanitize_vehicle_name("a__b___c"), QStringLiteral("a_b_c"));
    QCOMPARE(sanitize_vehicle_name("___leading"), QStringLiteral("leading"));
    QCOMPARE(sanitize_vehicle_name("trailing___"), QStringLiteral("trailing"));
  }

  void name_sanitizer_leading_digit_prefix() {
    using carla_studio::vehicle_import::sanitize_vehicle_name;
    QCOMPARE(sanitize_vehicle_name("2024_Tesla"), QStringLiteral("v_2024_tesla"));
    QCOMPARE(sanitize_vehicle_name("9life"), QStringLiteral("v_9life"));
  }

  void name_sanitizer_unicode_strip() {
    using carla_studio::vehicle_import::sanitize_vehicle_name;
    QCOMPARE(sanitize_vehicle_name(QString::fromUtf8("\xF0\x9F\x9A\x97" "car")),
             QStringLiteral("car"));
    QCOMPARE(sanitize_vehicle_name(QString::fromUtf8("Citro" "\xC3\xAB" "n_C4")),
             QStringLiteral("citro_n_c4"));
  }

  void name_sanitizer_length_cap() {
    using carla_studio::vehicle_import::sanitize_vehicle_name;
    using carla_studio::vehicle_import::kMaxVehicleNameLength;
    const QString very_long(kMaxVehicleNameLength + 25, QLatin1Char('a'));
    const QString result = sanitize_vehicle_name(very_long);
    QCOMPARE(result.size(), kMaxVehicleNameLength);
    QVERIFY(!result.endsWith(QLatin1Char('_')));
  }

  void name_sanitizer_idempotent() {
    using carla_studio::vehicle_import::sanitize_vehicle_name;
    using carla_studio::vehicle_import::is_canonical_vehicle_name;
    const QStringList samples = {
      "Brand+Model+Name+2024", "Tesla Model S", "2024_Tesla",
      "MyTruck", "vehicle.audi.tt", "  spaces  ",
    };
    for (const QString &s : samples) {
      const QString first  = sanitize_vehicle_name(s);
      const QString second = sanitize_vehicle_name(first);
      QCOMPARE(second, first);
      if (!first.isEmpty()) QVERIFY(is_canonical_vehicle_name(first));
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
    const MeshGeometry g = load_mesh_geometry_obj(p);
    QVERIFY(g.valid);
    QVERIFY(g.face_count() > 16);
    const MeshAnalysisResult r = analyze_mesh(g, 100.0f);
    QVERIFY(r.ok);
    QVERIFY2(r.has_four_wheels, qPrintable(QString("got %1 wheels").arg(r.wheels.size())));
    QCOMPARE(static_cast<int>(r.wheels.size()), 4);
    for (const WheelCandidate &w : r.wheels) {
      QVERIFY(w.radius > 25.0f && w.radius < 50.0f);
    }
    QCOMPARE(r.size_class, SizeClass::Sedan);
    QVERIFY(!r.small_vehicle_needs_wheel_shape);
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
    const MeshGeometry g = load_mesh_geometry_obj(p);
    QVERIFY(g.valid);
    const MeshAnalysisResult r = analyze_mesh(g, 100.0f);
    QVERIFY(r.ok);
    QVERIFY(r.has_four_wheels);
    QCOMPARE(r.size_class, SizeClass::TinyRobot);
    QVERIFY(r.small_vehicle_needs_wheel_shape);
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
      const MeshGeometry g = load_mesh_geometry_obj(p);
      QVERIFY2(g.valid, c.name);
      const MeshAnalysisResult r = analyze_mesh(g, 100.0f);
      QVERIFY2(r.ok, c.name);
      QVERIFY2(r.has_four_wheels,
               qPrintable(QString("%1: got %2 wheels").arg(c.name).arg(r.wheels.size())));
      QVERIFY2(r.size_class == c.expect,
               qPrintable(QString("%1: class=%2 expect=%3")
                            .arg(c.name)
                            .arg(size_class_name(r.size_class))
                            .arg(size_class_name(c.expect))));
      QCOMPARE(r.small_vehicle_needs_wheel_shape, c.expectSmallFlag);
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
      const SizePreset p = preset_for_size_class(c);
      QVERIFY2(p.mass > prevMass, qPrintable(size_class_name(c)));
      QVERIFY2(!p.torque_curve_tag.isEmpty(), qPrintable(size_class_name(c)));
      QVERIFY2(!seenTags.contains(p.torque_curve_tag), qPrintable(p.torque_curve_tag));
      seenTags.insert(p.torque_curve_tag);
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
    const MeshGeometry      g = load_mesh_geometry_obj(p);
    const MeshAnalysisResult r = analyze_mesh(g, 100.0f);
    const VehicleSpec spec = build_spec_from_analysis(r, "my_truck", p, 100.0f, 2, 0);
    QCOMPARE(spec.name, QStringLiteral("my_truck"));
    QCOMPARE(spec.size_class, SizeClass::Sedan);
    QVERIFY(spec.has_four_wheels);
    QCOMPARE(spec.mass, 1500.0f);
    const QJsonObject j = spec_to_json(spec);
    QCOMPARE(j.value("vehicle_name").toString(), QStringLiteral("my_truck"));
    QCOMPARE(j.value("size_class").toString(),   QStringLiteral("sedan"));
    QCOMPARE(j.value("has_four_wheels").toBool(), true);
    QVERIFY(j.contains("wheel_fl"));
    QVERIFY(j.contains("chassis_aabb_cm"));
  }

  void import_mode_lite_when_nothing_set() {
    using namespace carla_studio::vehicle_import;
    const ImportModeStatus s = determine_import_mode("", "", false);
    QCOMPARE(s.mode, ImportMode::Lite);
    QVERIFY(!s.has_uproject); QVERIFY(!s.has_engine); QVERIFY(!s.plugin_aligned);
    QVERIFY(!s.forced_lite);
    QVERIFY(s.reason.contains("missing"));
  }
  void import_mode_force_lite_overrides() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir d; QVERIFY(d.isValid());
    const QString uproj = d.filePath("Carla.uproject");
    { QFile f(uproj); QVERIFY(f.open(QIODevice::WriteOnly)); f.write("{}"); }
    const ImportModeStatus s = determine_import_mode(uproj, "/no/such/engine", true);
    QCOMPARE(s.mode, ImportMode::Lite);
    QVERIFY(s.forced_lite);
    QVERIFY(s.reason.contains("forced"));
  }
  void import_mode_lite_when_uproject_only() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir d; QVERIFY(d.isValid());
    const QString uproj = d.filePath("Carla.uproject");
    { QFile f(uproj); QVERIFY(f.open(QIODevice::WriteOnly)); f.write("{}"); }
    const ImportModeStatus s = determine_import_mode(uproj, "", false);
    QCOMPARE(s.mode, ImportMode::Lite);
    QVERIFY(s.has_uproject);
    QVERIFY(!s.has_engine);
  }
  void kit_bundler_writes_full_bundle() {
    using namespace carla_studio::vehicle_import;
    QTemporaryDir d; QVERIFY(d.isValid());
    const QString mesh_path = d.filePath("source.obj");
    {
      QFile f(mesh_path);
      QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      writeVehicleObj(s, 2.2f, 0.80f, 0.40f, 1.40f, 0.35f, 0.13f);
    }
    const MeshGeometry g = load_mesh_geometry_obj(mesh_path);
    const MeshAnalysisResult r = analyze_mesh(g, 100.0f);
    VehicleSpec spec = build_spec_from_analysis(r, "test_sedan", mesh_path, 100.0f, 2, 0);
    const QString out_dir = d.filePath("kit_out");
    const KitBundleResult res = write_vehicle_kit(spec, out_dir);
    QVERIFY2(res.ok, qPrintable(res.detail));
    QVERIFY(QFileInfo(out_dir + "/spec.json").isFile());
    QVERIFY(QFileInfo(out_dir + "/README.md").isFile());
    QVERIFY(QFileInfo(out_dir + "/VehicleParameters.entry.json").isFile());
    QVERIFY(QFileInfo(out_dir + "/mesh.obj").isFile());

    QFile sj(out_dir + "/spec.json"); QVERIFY(sj.open(QIODevice::ReadOnly));
    QJsonParseError e; QJsonDocument doc = QJsonDocument::fromJson(sj.readAll(), &e);
    QCOMPARE(e.error, QJsonParseError::NoError);
    QCOMPARE(doc.object().value("vehicle_name").toString(), QStringLiteral("test_sedan"));
    QCOMPARE(doc.object().value("size_class").toString(), QStringLiteral("sedan"));

    QFile rd(out_dir + "/README.md"); QVERIFY(rd.open(QIODevice::ReadOnly | QIODevice::Text));
    const QString readme = QTextStream(&rd).readAll();
    QVERIFY(readme.contains("Vehicle Kit"));
    QVERIFY(readme.contains("test_sedan"));
    QVERIFY(readme.contains("Detected vehicle profile"));
    QVERIFY(readme.contains("Per-wheel measurements"));

    QFile ej(out_dir + "/VehicleParameters.entry.json"); QVERIFY(ej.open(QIODevice::ReadOnly));
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
    const QString mesh_path = d.filePath("robot.obj");
    {
      QFile f(mesh_path);
      QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
      QTextStream s(&f);
      writeVehicleObj(s, 0.50f, 0.30f, 0.05f, 0.35f, 0.05f, 0.02f);
    }
    const MeshGeometry g = load_mesh_geometry_obj(mesh_path);
    const MeshAnalysisResult r = analyze_mesh(g, 100.0f);
    QVERIFY(r.small_vehicle_needs_wheel_shape);
    VehicleSpec spec = build_spec_from_analysis(r, "tiny_robot", mesh_path, 100.0f, 2, 0);
    const QString out_dir = d.filePath("kit");
    QVERIFY(write_vehicle_kit(spec, out_dir).ok);
    QFile rd(out_dir + "/README.md");
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
    const MeshGeometry g = load_mesh_geometry_obj(p);
    const MeshAnalysisResult r = analyze_mesh(g, 100.0f);
    const VehicleSpec body = build_spec_from_analysis(r, "s", p, 100.0f, 2, 0);
    const MergeResult mr = merge_body_and_wheels(body, WheelTemplate{}, false);
    QVERIFY2(mr.ok, qPrintable(mr.reason));
    QCOMPARE(mr.mode, IngestionMode::Combined);
    QVERIFY(mr.spec.has_four_wheels);
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
    const MeshGeometry g = load_mesh_geometry_obj(p);
    const MeshAnalysisResult r = analyze_mesh(g, 100.0f);
    const VehicleSpec body = build_spec_from_analysis(r, "s", p, 100.0f, 2, 0);
    WheelTemplate tpl; tpl.valid = true; tpl.radius = 42.0f; tpl.width = 22.0f;
    const MergeResult mr = merge_body_and_wheels(body, tpl, true);
    QVERIFY2(mr.ok, qPrintable(mr.reason));
    QCOMPARE(mr.mode, IngestionMode::TireSwap);
    for (const WheelSpec &w : mr.spec.wheels) {
      QCOMPARE(w.radius, 42.0f);
      QCOMPARE(w.width,  22.0f);
    }
    QVERIFY(!mr.spec.small_vehicle_needs_wheel_shape);
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
    const MeshGeometry g = load_mesh_geometry_obj(p);
    const MeshAnalysisResult r = analyze_mesh(g, 100.0f);
    const VehicleSpec body = build_spec_from_analysis(r, "chassis", p, 100.0f, 2, 0);
    QVERIFY(!body.has_four_wheels);
    WheelTemplate tpl; tpl.valid = true; tpl.radius = 30.0f; tpl.width = 18.0f;
    const MergeResult mr = merge_body_and_wheels(body, tpl, true);
    QVERIFY2(mr.ok, qPrintable(mr.reason));
    QCOMPARE(mr.mode, IngestionMode::BodyPlusTires);
    QVERIFY(mr.spec.has_four_wheels);
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
    const MeshGeometry g = load_mesh_geometry_obj(p);
    const MeshAnalysisResult r = analyze_mesh(g, 100.0f);
    const VehicleSpec body = build_spec_from_analysis(r, "chassis", p, 100.0f, 2, 0);
    const MergeResult mr = merge_body_and_wheels(body, WheelTemplate{}, false);
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
    const MeshGeometry g = load_mesh_geometry_obj(p);
    QVERIFY(g.valid);
    const MeshAnalysisResult r = analyze_mesh(g, 100.0f);
    QVERIFY(r.ok);
    QVERIFY(!r.has_four_wheels);
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
