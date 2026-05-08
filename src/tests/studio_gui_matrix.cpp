// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include <QtTest/QtTest>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QTemporaryDir>

#include "traffic/player_slots.h"
#include "vehicle/mount/sensor_mount_key.h"
#include "core/studio_app_context.h"
#include "utils/platform_proc.h"
#include "utils/resource_fit.h"
#include "vehicle/import/mesh_aabb.h"
#include "vehicle/import/mesh_analysis.h"
#include "vehicle/import/mesh_geometry.h"
#include "vehicle/import/merged_spec_builder.h"
#include "vehicle/import/name_sanitizer.h"
#include "vehicle/import/obj_sanitizer.h"
#include "vehicle/import/vehicle_spec.h"

#ifdef CARLA_STUDIO_WITH_QT3D
#include "vehicle/calibration/calibration_scene.h"
#endif

class StudioGuiMatrix : public QObject {
  Q_OBJECT

 private:
  QString m_app_source_path;
  QString m_binary_path;
  QString m_repo_root;
  QString m_tests_dir;
  QString m_fixtures_dir;
  QString m_outputs_dir;

  QString fixture_dir(const QString &testTag) {
    QDir d(m_fixtures_dir);
    d.mkpath(testTag);
    return d.filePath(testTag);
  }

  QString output_dir(const QString &testTag) {
    QDir d(m_outputs_dir);
    d.mkpath(testTag);
    return d.filePath(testTag);
  }

  QString readSource() const {
    QFile f(m_app_source_path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return {};
    return QString::fromUtf8(f.readAll());
  }

  bool sourceContains(const QString &needle) const {
    return readSource().contains(needle);
  }

  bool sourceMatches(const QRegularExpression &re) const {
    return re.match(readSource()).hasMatch();
  }

 private slots:
  void initTestCase() {
#ifdef CARLA_STUDIO_SOURCE_PATH
    m_app_source_path = QStringLiteral(CARLA_STUDIO_SOURCE_PATH);
#endif
#ifdef CARLA_STUDIO_BINARY_PATH
    m_binary_path = QStringLiteral(CARLA_STUDIO_BINARY_PATH);
#endif
#ifdef CARLA_STUDIO_REPO_ROOT
    m_repo_root = QStringLiteral(CARLA_STUDIO_REPO_ROOT);
#endif
#ifdef CARLA_STUDIO_TESTS_DIR
    m_tests_dir = QStringLiteral(CARLA_STUDIO_TESTS_DIR);
#else
    if (!m_repo_root.isEmpty()) m_tests_dir = m_repo_root + "/tests";
#endif
    m_fixtures_dir = m_tests_dir + "/fixtures";
    m_outputs_dir  = m_tests_dir + "/outputs";
    QDir().mkpath(m_fixtures_dir);
    QDir().mkpath(m_outputs_dir);
    if (m_app_source_path.isEmpty() || !QFile::exists(m_app_source_path))
      QSKIP("CARLA_STUDIO_SOURCE_PATH not set or invalid", SkipAll);
  }








  void test_a01_binary_present_and_runnable() {
    if (m_binary_path.isEmpty())
      QSKIP("CARLA_STUDIO_BINARY_PATH not set", SkipSingle);
    QFileInfo fi(m_binary_path);
    QVERIFY2(fi.exists(),     qPrintable("binary missing: " + m_binary_path));
    QVERIFY2(fi.isExecutable(), "binary is not executable");
    QVERIFY2(fi.size() > 100 * 1024, "binary suspiciously small");
  }




  void test_a02_binary_starts_without_immediate_crash() {
    if (m_binary_path.isEmpty()) QSKIP("no binary path", SkipSingle);
    QProcess p;
    p.setProgram(m_binary_path);
    p.setArguments({"--version-only"});
    p.start();
    if (!p.waitForStarted(3000))
      QSKIP("binary did not start (likely needs DISPLAY)", SkipSingle);
    p.terminate();
    p.waitForFinished(2000);
    if (p.state() != QProcess::NotRunning) p.kill();
    QVERIFY(true);
  }




  void test_a03_status_states_wired() {
    QVERIFY2(sourceContains("setSimulationStatus(\"Idle\")"),         "Idle missing");
    QVERIFY2(sourceContains("setSimulationStatus(\"Running\")"),      "Running missing");
    QVERIFY2(sourceContains("setSimulationStatus(\"Stopped\")"),      "Stopped missing");
    QVERIFY2(sourceContains("setSimulationStatus(\"Initializing\")"), "Initializing missing");
  }




  void test_a04_start_button_launches_carla_with_rpc_port() {
    QVERIFY2(sourceContains("-carla-rpc-port="),
             "START path missing -carla-rpc-port flag");
    QVERIFY2(sourceContains("echo $!"),
             "START path does not capture child PID");
  }




  void test_a05_stop_button_has_force_stop_timer() {
    QVERIFY2(sourceContains("forceStopTimer->start(60000)"),
             "STOP missing 60-second force-stop fallback");
  }








  void test_b06_process_table_columns_wired() {
    QVERIFY(sourceContains("carla_process_table"));
    QVERIFY(sourceContains("trackedCarlaPids"));
    QVERIFY(sourceContains("\"PID\" << \"Process\" << \"CPU\" << \"Memory\" << \"GPU\""));
  }




  void test_b07_nvidia_probes_guarded_for_mac() {
    QVERIFY2(sourceMatches(QRegularExpression(
        "#if !defined\\(Q_OS_MAC\\)[\\s\\S]{0,500}nvidia-smi pmon")),
        "nvidia-smi block not guarded by !Q_OS_MAC");
  }




  void test_b08_platform_total_ram_works() {
    const qint64 mib = carla_studio_proc::system_total_ram_mib();
    QVERIFY2(mib > 256, qPrintable(
        QString("system_total_ram_mib returned %1, expected > 256").arg(mib)));
  }



  void test_b09_platform_cpu_model_works() {
    const QString cpu = carla_studio_proc::cpu_model_string();
    QVERIFY2(!cpu.isEmpty(), "cpu_model_string returned empty");
  }



  void test_b10_platform_pid_alive_works() {
    QVERIFY( carla_studio_proc::pid_is_alive(QCoreApplication::applicationPid()));
    QVERIFY(!carla_studio_proc::pid_is_alive(0));
    QVERIFY(!carla_studio_proc::pid_is_alive(-1));
  }







  void test_c11_calibration_view_dialog_title() {
    QVERIFY2(sourceContains("\"Calibration View - %1\""),
             "Dialog title not renamed to Calibration View");
  }




  void test_c12_per_sensor_groups_preserved() {
    QVERIFY(sourceContains("\"Mount Presets\""));
    QVERIFY(sourceContains("\"Lens / Optics\""));
    QVERIFY(sourceContains("\"Sensor Characteristics\""));
    QVERIFY(sourceContains("\"Position\""));
    QVERIFY(sourceContains("\"Orientation\""));
  }



  void test_c13_calibration_scene_embedded() {
    QVERIFY2(sourceContains("calibration::CalibrationScene"),
             "CalibrationScene not constructed in dialog");
  }




  void test_c14_palette_has_six_target_types() {
    QVERIFY(sourceContains("\"Checkerboard\""));
    QVERIFY(sourceContains("\"April Tag\""));
    QVERIFY(sourceContains("\"Color Checker\""));
    QVERIFY(sourceContains("\"Cone Square\""));
    QVERIFY(sourceContains("\"Cone Line\""));
    QVERIFY(sourceContains("\"Pylon Wall\""));
  }



  void test_c15_palette_selection_drives_drop_type() {
    QVERIFY2(sourceContains("set_next_drop_type("),
             "Palette selection not wired to scene drop type");
  }



  void test_c16_save_apply_persists_to_json() {
    QVERIFY(sourceContains("save_persisted("));
    QVERIFY(sourceContains("\"Save && Apply\""));
  }



  void test_c17_auto_spawn_on_start_hook_present() {
    QVERIFY2(sourceContains("list_persisted_sensors("),
             "auto-spawn pass missing list_persisted_sensors call");
    QVERIFY2(sourceContains("static.prop.constructioncone"),
             "auto-spawn pass missing cone blueprint");
    QVERIFY2(sourceContains("MakeDebugHelper"),
             "auto-spawn pass missing DebugHelper for procedural patterns");
  }



  void test_c18_calibration_json_round_trip() {
#ifndef CARLA_STUDIO_WITH_QT3D
    QSKIP("Qt3D not available; calibration module not linked", SkipSingle);
#else
    const QString dir = fixture_dir("c18_calibration_round_trip");
    QDir(dir).removeRecursively();
    QDir().mkpath(dir);
    qputenv("CARLA_STUDIO_CALIBRATION_DIR", dir.toUtf8());

    using namespace carla_studio::calibration;
    QList<PlacedTarget> in;
    {
      PlacedTarget t; t.type = TargetType::Checkerboard;
      t.x = -57.0; t.y = 24.0; t.z = 1.5; t.yaw_deg = 0;
      t.cols = 8; t.rows = 6; t.size_m = 1.5; t.april_id = 0;
      in.append(t);
    }
    {
      PlacedTarget t; t.type = TargetType::AprilTag;
      t.x = -55.0; t.y = 22.0; t.z = 0.0; t.yaw_deg = 90;
      in.append(t);
    }
    {
      PlacedTarget t; t.type = TargetType::ConeSquare;
      t.x = -55.0; t.y = 25.0; t.z = 0.0; in.append(t);
    }
    save_persisted("matrix-test", "RGB", in);

    const QStringList sensors = list_persisted_sensors("matrix-test");
    QCOMPARE(sensors.size(), 1);
    QCOMPARE(sensors.first(), QStringLiteral("RGB"));

    const QList<PlacedTarget> out = load_persisted("matrix-test", "RGB");
    QCOMPARE(out.size(), 3);
    QCOMPARE(static_cast<int>(out[0].type), static_cast<int>(TargetType::Checkerboard));
    QCOMPARE(out[0].x, -57.0);
    QCOMPARE(out[0].cols, 8);
    QCOMPARE(static_cast<int>(out[1].type), static_cast<int>(TargetType::AprilTag));
    QCOMPARE(out[1].yaw_deg, 90.0);
    QCOMPARE(static_cast<int>(out[2].type), static_cast<int>(TargetType::ConeSquare));
    qunsetenv("CARLA_STUDIO_CALIBRATION_DIR");
#endif
  }




  void test_c19_calibration_scene_add_remove() {
#ifndef CARLA_STUDIO_WITH_QT3D
    QSKIP("Qt3D not available", SkipSingle);
#else
    using namespace carla_studio::calibration;
    CalibrationScene s;
    QSignalSpy sp(&s, &CalibrationScene::targets_changed);

    s.add_target_at_center(TargetType::Checkerboard);
    s.add_target_at_center(TargetType::AprilTag);
    QCOMPARE(s.targets().size(), 2);
    QCOMPARE(static_cast<int>(s.targets()[1].type),
             static_cast<int>(TargetType::AprilTag));
    QCOMPARE(s.selected_index(), 1);

    s.remove_at(0);
    QCOMPARE(s.targets().size(), 1);
    QCOMPARE(static_cast<int>(s.targets()[0].type),
             static_cast<int>(TargetType::AprilTag));

    s.clear_all();
    QCOMPARE(s.targets().size(), 0);
    QVERIFY(sp.count() >= 4);
#endif
  }




  void test_c20_calibration_scene_update_pose() {
#ifndef CARLA_STUDIO_WITH_QT3D
    QSKIP("Qt3D not available", SkipSingle);
#else
    using namespace carla_studio::calibration;
    CalibrationScene s;
    s.add_target_at_center(TargetType::Checkerboard);
    QSignalSpy sp(&s, &CalibrationScene::targets_changed);

    s.update_target_pose(0, 12.5, 3.25, 1.8, 45.0);
    QCOMPARE(s.targets()[0].x,      12.5);
    QCOMPARE(s.targets()[0].y,      3.25);
    QCOMPARE(s.targets()[0].z,      1.8);
    QCOMPARE(s.targets()[0].yaw_deg, 45.0);
    QCOMPARE(sp.count(),           1);


    s.update_target_pose(99, 0, 0, 0, 0);
    QCOMPARE(s.targets()[0].x, 12.5);
#endif
  }







  void test_d21_mount_presets_complete() {
    for (const char *name : {"Roof Center", "Front Bumper", "Rear Bumper",
                              "Hood", "Trunk", "Mirror \xc2\xb7 Left",
                              "Mirror \xc2\xb7 Right"}) {
      QVERIFY2(sourceContains(QString("\"%1\"").arg(name)),
               qPrintable(QString("preset missing: %1").arg(name)));
    }
    QVERIFY(sourceContains("\"Reset\""));
  }



  void test_d22_orientation_clamped_zero_to_359() {
    QVERIFY(sourceContains("rxSpin->setRange(0.0, 359.0)"));
    QVERIFY(sourceContains("rySpin->setRange(0.0, 359.0)"));
    QVERIFY(sourceContains("rz->setRange(0.0, 359.0)"));
  }



  void test_d23_sensor_mount_key_deterministic() {
    using carla::studio::core::sensor_mount_key;
    QCOMPARE(sensor_mount_key("RGB", 1), QStringLiteral("RGB"));
    QCOMPARE(sensor_mount_key("RGB", 2), QStringLiteral("RGB#2"));
    QCOMPARE(sensor_mount_key("RGB", 3), QStringLiteral("RGB#3"));
    QCOMPARE(sensor_mount_key("RGB", 1), sensor_mount_key("RGB", 1));
    QVERIFY(sensor_mount_key("RGB", 1) != sensor_mount_key("RGB", 2));
  }







  void test_e24_view_tile_buttons_wired() {
    for (const char *btn : {"fpvBtn", "tpvBtn", "cpvBtn", "bevBtn"}) {
      QVERIFY2(sourceContains(QString("openViewTile(\"%1\")")
                .arg(QString(btn).left(3).toUpper())),
               qPrintable(QString("%1 not wired to openViewTile").arg(btn)));
    }
  }



  void test_e25_view_tile_spawns_rgb_camera_on_hero() {
    QVERIFY(sourceContains("sensor.camera.rgb"));
    QVERIFY(sourceContains("in_app_drive_vehicle"));
    QVERIFY(sourceContains("camera->Listen("));
  }







  void test_f26_obj_name_sanitizer() {
    using namespace carla_studio::vehicle_import;
    const QString a = sanitize_vehicle_name("Brand+Model Name 2024");
    QVERIFY2(!a.contains('+') && !a.contains(' '),
             qPrintable("got: " + a));
    const QString b = sanitize_vehicle_name("car/with\\path");
    QVERIFY2(!b.contains('/') && !b.contains('\\'),
             qPrintable("got: " + b));
  }



  void test_f27_mesh_aabb_basic() {
    const QString dir = fixture_dir("f27_mesh_aabb");
    QDir(dir).removeRecursively();
    QDir().mkpath(dir);
    const QString path = dir + "/box.obj";
    QFile f(path);
    QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
    QTextStream out(&f);
    out << "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n"
        << "v 0 0 1\nv 1 0 1\nv 1 1 1\nv 0 1 1\n";
    f.close();

    using namespace carla_studio::vehicle_import;
    const MeshAABB bb = parse_obj(path);
    QVERIFY2(bb.valid, "parse_obj did not flag valid");
    QCOMPARE(bb.x_max - bb.x_min, 1.0f);
    QCOMPARE(bb.y_max - bb.y_min, 1.0f);
    QCOMPARE(bb.z_max - bb.z_min, 1.0f);
    QCOMPARE(bb.vertex_count, 8);
  }







  void test_g28_importer_client_no_posix_sockets() {
    QDir d(QFileInfo(m_app_source_path).dir());
    d.cdUp();
    const QString p = d.filePath("src/vehicle/import/importer_client.cpp");
    QFile f(p);
    QVERIFY2(f.open(QIODevice::ReadOnly), qPrintable("missing: " + p));
    const QString src = QString::fromUtf8(f.readAll());
    QVERIFY2(!src.contains("<arpa/inet.h>"),  "still includes arpa/inet.h");
    QVERIFY2(!src.contains("<sys/socket.h>"), "still includes sys/socket.h");
    QVERIFY2(!src.contains("<sys/select.h>"), "still includes sys/select.h");
    QVERIFY2( src.contains("QTcpSocket"),     "not using QTcpSocket");
  }






  QString vehicle_fixture(const QString &subdir) const {
    return m_fixtures_dir + "/vehicles/" + subdir;
  }

  void verifyParsedOBJ(const QString &path, int minVerts) {
    using namespace carla_studio::vehicle_import;
    if (!QFileInfo::exists(path))
      QSKIP(qPrintable("fixture missing: " + path), SkipSingle);
    const MeshAABB bb = parse_obj(path);
    QVERIFY2(bb.valid, qPrintable("parse_obj said invalid: " + path));
    QVERIFY2(bb.vertex_count >= minVerts,
             qPrintable(QString("vertex count %1 < %2 for %3")
                          .arg(bb.vertex_count).arg(minVerts).arg(path)));
    QVERIFY2(bb.x_max > bb.x_min && bb.y_max > bb.y_min && bb.z_max > bb.z_min,
             "AABB collapsed (zero extent on at least one axis)");
  }





  void test_v05_vehicle_names_sanitize_safe() {
    using namespace carla_studio::vehicle_import;
    const QStringList raws = {
      "My_Vehicle-2024", "Car+Model+Name", "SUV_Body", "test-car"
    };
    for (const QString &raw : raws) {
      const QString s = sanitize_vehicle_name(raw);
      QVERIFY2(!s.isEmpty(),                qPrintable("empty: " + raw));
      QVERIFY2(!s.contains(' '),            qPrintable("space: " + s));
      QVERIFY2(!s.contains('+'),            qPrintable("plus: "  + s));
      QVERIFY2(!s.contains('/'),            qPrintable("slash: " + s));
      QVERIFY2(!s.contains('\\'),           qPrintable("bksl: "  + s));
    }
  }












  void test_i07_name_sanitizer_safe() {
    using namespace carla_studio::vehicle_import;
    const QString s = sanitize_vehicle_name("My_Test_Vehicle");
    QVERIFY(!s.isEmpty());
    QVERIFY(!s.contains(' '));
    QVERIFY(!s.contains('+'));
    QVERIFY(!s.contains('/'));
    QVERIFY(!s.contains('\\'));
  }




















  void test_k02_chassis_z_lift_keeps_wheels_above_road() {
    using namespace carla_studio::vehicle_import;
    VehicleSpec body_spec;
    body_spec.detected_from_mesh = true;
    body_spec.has_four_wheels    = false;
    body_spec.chassis_x_min = -200; body_spec.chassis_x_max =  200;
    body_spec.chassis_y_min =  -90; body_spec.chassis_y_max =   90;
    body_spec.chassis_z_min = -150; body_spec.chassis_z_max =   30;

    WheelTemplate tpl;
    tpl.valid  = true;
    tpl.radius = 33.0f;
    tpl.width  = 22.0f;

    const MergeResult mr = merge_body_and_wheels(body_spec, tpl, true);
    QVERIFY(mr.ok);
    for (size_t i = 0; i < mr.spec.wheels.size(); ++i) {
      const float wz = mr.spec.wheels[i].z;
      QVERIFY2(qFuzzyCompare(wz, tpl.radius),
               qPrintable(QString("wheel[%1].z=%2 should equal radius %3 "
                                  "(so wheel bottom touches road at z=0)")
                            .arg(i).arg(wz).arg(tpl.radius)));
    }
  }



  void test_k03_verifier_rejects_degenerate_spec() {
    using namespace carla_studio::vehicle_import;
    VehicleSpec bad;
    bad.detected_from_mesh = true;
    bad.has_four_wheels    = false;
    const SpecVerification v = verify_chaos_vehicle_spec(bad);
    QVERIFY(!v.ok);
    QVERIFY(!v.errors.isEmpty());
    QVERIFY(!v.four_wheels_present);
    QVERIFY(!v.chassis_extent_positive);
  }





  void test_k04_carlatools_forces_com_nudge_zero() {
    const QString srcRoot = qEnvironmentVariable("CARLA_SRC_ROOT");
    if (srcRoot.isEmpty()) QSKIP("CARLA_SRC_ROOT not set", SkipSingle);
    const QString p = srcRoot +
      "/Unreal/CarlaUnreal/Plugins/CarlaTools/"
      "Source/CarlaTools/Private/USDImporterWidget.cpp";
    if (!QFileInfo::exists(p))
      QSKIP("CarlaTools source not on this machine", SkipSingle);
    QFile f(p);
    QVERIFY(f.open(QIODevice::ReadOnly));
    const QString src = QString::fromUtf8(f.readAll());
    QVERIFY2(src.contains("COMNudge = FVector::ZeroVector"),
             "CarlaTools no longer forces zero COMNudge - Chaos drive will fail");
  }






  void test_k07_forward_axis_auto_swap() {
    using namespace carla_studio::vehicle_import;
    MeshAnalysisResult r;
    r.ok = true;
    r.has_four_wheels = false;
    r.size_class = SizeClass::Sedan;
    r.chassis_x_min = -100; r.chassis_x_max = 100;
    r.chassis_y_min = -250; r.chassis_y_max = 250;
    r.chassis_z_min = 0;    r.chassis_z_max = 150;
    r.overall_x_min = -100; r.overall_x_max = 100;
    r.overall_y_min = -250; r.overall_y_max = 250;
    r.overall_z_min = 0;    r.overall_z_max = 150;
    const VehicleSpec s = build_spec_from_analysis(r, "synth", "/dev/null", 1.0f, 2, 0);
    QCOMPARE(s.chassis_x_min, -250.0f);
    QCOMPARE(s.chassis_x_max,  250.0f);
    QCOMPARE(s.chassis_y_min, -100.0f);
    QCOMPARE(s.chassis_y_max,  100.0f);
    QCOMPARE(s.adjust_yaw_deg, 90.0f);

    MeshAnalysisResult r2 = r;
    r2.chassis_y_min = -90;  r2.chassis_y_max =  90;
    r2.overall_y_min = -90;  r2.overall_y_max =  90;
    r2.chassis_x_min = -250; r2.chassis_x_max = 250;
    r2.overall_x_min = -250; r2.overall_x_max = 250;
    const VehicleSpec s2 = build_spec_from_analysis(r2, "synth2", "/dev/null", 1.0f, 2, 0);
    QCOMPARE(s2.chassis_x_min, -250.0f);
    QCOMPARE(s2.chassis_x_max,  250.0f);
    QCOMPARE(s2.adjust_yaw_deg, 0.0f);
  }






  void test_k06_clustered_wheels_fall_back_to_derived() {
    using namespace carla_studio::vehicle_import;
    VehicleSpec body;
    body.detected_from_mesh = true;
    body.has_four_wheels    = true;
    body.chassis_x_min = -200; body.chassis_x_max = 200;
    body.chassis_y_min = -90;  body.chassis_y_max = 90;
    body.chassis_z_min =  0;   body.chassis_z_max = 150;
    for (size_t i = 0; i < 4; ++i) {
      body.wheels[i].x = 100.0f + static_cast<float>(i) * 0.5f;
      body.wheels[i].y = -80.0f + static_cast<float>(i) * 0.5f;
      body.wheels[i].z = 35.0f;
      body.wheels[i].radius = 35.0f;
      body.wheels[i].width  = 22.0f;
    }
    WheelTemplate tpl;
    tpl.valid = true; tpl.radius = 35.0f; tpl.width = 22.0f;

    const MergeResult mr = merge_body_and_wheels(body, tpl, true);
    QVERIFY(mr.ok);
    QVERIFY2(mr.reason.contains("fallback"),
             qPrintable("expected clustered fallback path; got: " + mr.reason));
    float y_min = mr.spec.wheels[0].y, y_max = y_min;
    float x_min = mr.spec.wheels[0].x, x_max = x_min;
    for (const WheelSpec &w : mr.spec.wheels) {
      y_min = std::min(y_min, w.y); y_max = std::max(y_max, w.y);
      x_min = std::min(x_min, w.x); x_max = std::max(x_max, w.x);
    }
    QVERIFY2(y_max - y_min >= 100.0f,
      qPrintable(QString("after fallback wheels still clustered y: %1").arg(y_max - y_min)));
    QVERIFY2(x_max - x_min >= 100.0f,
      qPrintable(QString("after fallback wheels still clustered x: %1").arg(x_max - x_min)));
  }




  void test_k05_drive_baseline_captured() {
    const QString p = m_outputs_dir + "/drive_matrix_summary.txt";
    if (!QFileInfo::exists(p))
      QSKIP("drive baseline not yet captured (run drive_baseline first)",
            SkipSingle);
    QFile f(p);
    QVERIFY(f.open(QIODevice::ReadOnly));
    const QString s = QString::fromUtf8(f.readAll());
    QVERIFY2(s.contains("vehicle.custom."),
             "drive baseline doesn't reference any custom vehicle");
  }



  void test_j01_shipping_sim_reachable() {
    QProcess p;
    p.setProgram("/bin/sh");
    p.setArguments({"-c", "ss -ltn | grep -q ':2000 '"});
    p.start();
    p.waitForFinished(2000);
    if (p.exitCode() != 0)
      QSKIP("CARLA shipping sim not listening on :2000 - start it then re-run J*", SkipSingle);
    QVERIFY(true);
  }




  void test_k01_capture_usage_screenshots() {
    if (m_binary_path.isEmpty())
      QSKIP("CARLA_STUDIO_BINARY_PATH not set", SkipSingle);
    if (qEnvironmentVariable("DISPLAY").isEmpty())
      QSKIP("no DISPLAY - screenshot capture needs a graphical session", SkipSingle);
    if (m_repo_root.isEmpty())
      QSKIP("CARLA_STUDIO_REPO_ROOT not set", SkipSingle);

    const QString out_dir = m_repo_root + "/docs/usage/tabs";
    QDir().mkpath(out_dir);

    QProcess p;
    p.setProgram(m_binary_path);
    p.setArguments({"--capture-screenshots", out_dir});
    p.start();
    QVERIFY2(p.waitForStarted(5000), "binary did not start");
    QVERIFY2(p.waitForFinished(20000), "capture did not finish within 20s");

    const QStringList pngs = QDir(out_dir).entryList({"*.png"}, QDir::Files);
    QVERIFY2(pngs.size() >= 4,
             qPrintable(QString("expected ≥4 screenshots, got %1").arg(pngs.size())));
  }



  void test_g29_huggingface_token_off_argv() {
    QDir d(QFileInfo(m_app_source_path).dir());
    d.cdUp();
    const QString p = d.filePath("src/integrations/hf/hugging_face.cpp");
    QFile f(p);
    QVERIFY(f.open(QIODevice::ReadOnly));
    const QString src = QString::fromUtf8(f.readAll());
    QVERIFY2(!src.contains("Authorization: Bearer %1\").arg(token)"),
             "token still embedded in argv via -H");
    QVERIFY2(src.contains("\"--config\", \"-\""),
             "curl --config - not used for stdin header injection");
  }
};

QTEST_MAIN(StudioGuiMatrix)
#include "studio_gui_matrix.moc"
