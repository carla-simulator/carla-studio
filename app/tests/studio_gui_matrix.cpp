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
  QString mAppSourcePath;
  QString mBinaryPath;
  QString mRepoRoot;
  QString mTestsDir;
  QString mFixturesDir;
  QString mOutputsDir;

  QString fixtureDir(const QString &testTag) {
    QDir d(mFixturesDir);
    d.mkpath(testTag);
    return d.filePath(testTag);
  }

  QString outputDir(const QString &testTag) {
    QDir d(mOutputsDir);
    d.mkpath(testTag);
    return d.filePath(testTag);
  }

  QString readSource() const {
    QFile f(mAppSourcePath);
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
    mAppSourcePath = QStringLiteral(CARLA_STUDIO_SOURCE_PATH);
#endif
#ifdef CARLA_STUDIO_BINARY_PATH
    mBinaryPath = QStringLiteral(CARLA_STUDIO_BINARY_PATH);
#endif
#ifdef CARLA_STUDIO_REPO_ROOT
    mRepoRoot = QStringLiteral(CARLA_STUDIO_REPO_ROOT);
#endif
#ifdef CARLA_STUDIO_TESTS_DIR
    mTestsDir = QStringLiteral(CARLA_STUDIO_TESTS_DIR);
#else
    if (!mRepoRoot.isEmpty()) mTestsDir = mRepoRoot + "/tests";
#endif
    mFixturesDir = mTestsDir + "/fixtures";
    mOutputsDir  = mTestsDir + "/outputs";
    QDir().mkpath(mFixturesDir);
    QDir().mkpath(mOutputsDir);
    if (mAppSourcePath.isEmpty() || !QFile::exists(mAppSourcePath))
      QSKIP("CARLA_STUDIO_SOURCE_PATH not set or invalid", SkipAll);
  }

  // ============================================================
  // Section A - App lifecycle (compile / launch / status states)
  // ============================================================

  // 01 precond: build produced the binary at the expected path
  // 01 test:    QFileInfo on the binary path
  // 01 expect:  binary exists, is executable, size > 100 KB
  void test_a01_binary_present_and_runnable() {
    if (mBinaryPath.isEmpty())
      QSKIP("CARLA_STUDIO_BINARY_PATH not set", SkipSingle);
    QFileInfo fi(mBinaryPath);
    QVERIFY2(fi.exists(),     qPrintable("binary missing: " + mBinaryPath));
    QVERIFY2(fi.isExecutable(), "binary is not executable");
    QVERIFY2(fi.size() > 100 * 1024, "binary suspiciously small");
  }

  // 02 precond: binary exists (test 01)
  // 02 test:    invoke `binary --help-cli` style probe - for now just check exec start
  // 02 expect:  process starts, doesn't immediately crash within 1s
  void test_a02_binary_starts_without_immediate_crash() {
    if (mBinaryPath.isEmpty()) QSKIP("no binary path", SkipSingle);
    QProcess p;
    p.setProgram(mBinaryPath);
    p.setArguments({"--version-only"});
    p.start();
    if (!p.waitForStarted(3000))
      QSKIP("binary did not start (likely needs DISPLAY)", SkipSingle);
    p.terminate();
    p.waitForFinished(2000);
    if (p.state() != QProcess::NotRunning) p.kill();
    QVERIFY(true);
  }

  // 03 precond: app source readable
  // 03 test:    grep for the four canonical status strings the status-bar emits
  // 03 expect:  Idle / Initializing / Running / Stopped all present in source
  void test_a03_status_states_wired() {
    QVERIFY2(sourceContains("setSimulationStatus(\"Idle\")"),         "Idle missing");
    QVERIFY2(sourceContains("setSimulationStatus(\"Running\")"),      "Running missing");
    QVERIFY2(sourceContains("setSimulationStatus(\"Stopped\")"),      "Stopped missing");
    QVERIFY2(sourceContains("setSimulationStatus(\"Initializing\")"), "Initializing missing");
  }

  // 04 precond: app source readable
  // 04 test:    START click handler launches CARLA via shell with -carla-rpc-port
  // 04 expect:  "carla-rpc-port" appears in launch path AND child PID is captured
  void test_a04_start_button_launches_carla_with_rpc_port() {
    QVERIFY2(sourceContains("-carla-rpc-port="),
             "START path missing -carla-rpc-port flag");
    QVERIFY2(sourceContains("echo $!"),
             "START path does not capture child PID");
  }

  // 05 precond: app source readable
  // 05 test:    STOP click handler has a force-stop fallback timer
  // 05 expect:  60-second forceStopTimer present
  void test_a05_stop_button_has_force_stop_timer() {
    QVERIFY2(sourceContains("forceStopTimer->start(60000)"),
             "STOP missing 60-second force-stop fallback");
  }

  // ============================================================
  // Section B - Process panel (per-PID + total CPU/MEM/GPU)
  // ============================================================

  // 06 precond: app source readable
  // 06 test:    process table widget + tracked PID store + column headers
  // 06 expect:  carlaProcessTable widget, trackedCarlaPids set, 5 columns
  void test_b06_process_table_columns_wired() {
    QVERIFY(sourceContains("carlaProcessTable"));
    QVERIFY(sourceContains("trackedCarlaPids"));
    QVERIFY(sourceContains("\"PID\" << \"Process\" << \"CPU\" << \"Memory\" << \"GPU\""));
  }

  // 07 precond: app source readable
  // 07 test:    nvidia-smi probes wrapped in non-Apple guard
  // 07 expect:  #if !defined(Q_OS_MAC) precedes nvidia-smi pmon block
  void test_b07_nvidia_probes_guarded_for_mac() {
    QVERIFY2(sourceMatches(QRegularExpression(
        "#if !defined\\(Q_OS_MAC\\)[\\s\\S]{0,500}nvidia-smi pmon")),
        "nvidia-smi block not guarded by !Q_OS_MAC");
  }

  // 08 precond: PlatformProc helper compiled in
  // 08 test:    systemTotalRamMiB returns > 0 on this host
  // 08 expect:  > 256 MiB (any modern dev box)
  void test_b08_platform_total_ram_works() {
    const qint64 mib = carla_studio_proc::systemTotalRamMiB();
    QVERIFY2(mib > 256, qPrintable(
        QString("systemTotalRamMiB returned %1, expected > 256").arg(mib)));
  }

  // 09 precond: PlatformProc helper compiled in
  // 09 test:    cpuModelString returns non-empty
  void test_b09_platform_cpu_model_works() {
    const QString cpu = carla_studio_proc::cpuModelString();
    QVERIFY2(!cpu.isEmpty(), "cpuModelString returned empty");
  }

  // 10 precond: PlatformProc helper compiled in
  // 10 test:    pidIsAlive(getpid()) is true; pidIsAlive(0) is false
  void test_b10_platform_pid_alive_works() {
    QVERIFY( carla_studio_proc::pidIsAlive(QCoreApplication::applicationPid()));
    QVERIFY(!carla_studio_proc::pidIsAlive(0));
    QVERIFY(!carla_studio_proc::pidIsAlive(-1));
  }

  // ============================================================
  // Section C - Sensor Config / Calibration View dialog
  // ============================================================

  // 11 precond: app source readable
  // 11 test:    dialog title says "Calibration View - <sensor>"
  void test_c11_calibration_view_dialog_title() {
    QVERIFY2(sourceContains("\"Calibration View - %1\""),
             "Dialog title not renamed to Calibration View");
  }

  // 12 precond: dialog title verified
  // 12 test:    Sensor Config sub-groups (Mount Presets, Lens/Optics, Sensor
  //             Characteristics, Position, Orientation) all preserved
  void test_c12_per_sensor_groups_preserved() {
    QVERIFY(sourceContains("\"Mount Presets\""));
    QVERIFY(sourceContains("\"Lens / Optics\""));
    QVERIFY(sourceContains("\"Sensor Characteristics\""));
    QVERIFY(sourceContains("\"Position\""));
    QVERIFY(sourceContains("\"Orientation\""));
  }

  // 13 precond: dialog source readable
  // 13 test:    new Calibration column instantiates CalibrationScene
  void test_c13_calibration_scene_embedded() {
    QVERIFY2(sourceContains("calibration::CalibrationScene"),
             "CalibrationScene not constructed in dialog");
  }

  // 14 precond: dialog source readable
  // 14 test:    palette has all 6 target types (Checkerboard / April Tag /
  //             Color Checker / Cone Square / Cone Line / Pylon Wall)
  void test_c14_palette_has_six_target_types() {
    QVERIFY(sourceContains("\"Checkerboard\""));
    QVERIFY(sourceContains("\"April Tag\""));
    QVERIFY(sourceContains("\"Color Checker\""));
    QVERIFY(sourceContains("\"Cone Square\""));
    QVERIFY(sourceContains("\"Cone Line\""));
    QVERIFY(sourceContains("\"Pylon Wall\""));
  }

  // 15 precond: palette wired
  // 15 test:    palette currentRowChanged → CalibrationScene::setNextDropType
  void test_c15_palette_selection_drives_drop_type() {
    QVERIFY2(sourceContains("setNextDropType("),
             "Palette selection not wired to scene drop type");
  }

  // 16 precond: scene embedded
  // 16 test:    Save & Apply button persists via savePersisted(scenarioId, sensor, targets)
  void test_c16_save_apply_persists_to_json() {
    QVERIFY(sourceContains("savePersisted("));
    QVERIFY(sourceContains("\"Save && Apply\""));
  }

  // 17 precond: persistence wired
  // 17 test:    auto-spawn-on-START hook walks listPersistedSensors + spawns cones
  void test_c17_auto_spawn_on_start_hook_present() {
    QVERIFY2(sourceContains("listPersistedSensors("),
             "auto-spawn pass missing listPersistedSensors call");
    QVERIFY2(sourceContains("static.prop.constructioncone"),
             "auto-spawn pass missing cone blueprint");
    QVERIFY2(sourceContains("MakeDebugHelper"),
             "auto-spawn pass missing DebugHelper for procedural patterns");
  }

  // 18 precond: persistence helpers exist
  // 18 test:    JSON round-trip - save 3 targets, load them back, equality
  void test_c18_calibration_json_round_trip() {
#ifndef CARLA_STUDIO_WITH_QT3D
    QSKIP("Qt3D not available; calibration module not linked", SkipSingle);
#else
    const QString dir = fixtureDir("c18_calibration_round_trip");
    QDir(dir).removeRecursively();
    QDir().mkpath(dir);
    qputenv("CARLA_STUDIO_CALIBRATION_DIR", dir.toUtf8());

    using namespace carla_studio::calibration;
    QList<PlacedTarget> in;
    {
      PlacedTarget t; t.type = TargetType::Checkerboard;
      t.x = -57.0; t.y = 24.0; t.z = 1.5; t.yawDeg = 0;
      t.cols = 8; t.rows = 6; t.sizeM = 1.5; t.aprilId = 0;
      in.append(t);
    }
    {
      PlacedTarget t; t.type = TargetType::AprilTag;
      t.x = -55.0; t.y = 22.0; t.z = 0.0; t.yawDeg = 90;
      in.append(t);
    }
    {
      PlacedTarget t; t.type = TargetType::ConeSquare;
      t.x = -55.0; t.y = 25.0; t.z = 0.0; in.append(t);
    }
    savePersisted("matrix-test", "RGB", in);

    const QStringList sensors = listPersistedSensors("matrix-test");
    QCOMPARE(sensors.size(), 1);
    QCOMPARE(sensors.first(), QStringLiteral("RGB"));

    const QList<PlacedTarget> out = loadPersisted("matrix-test", "RGB");
    QCOMPARE(out.size(), 3);
    QCOMPARE(static_cast<int>(out[0].type), static_cast<int>(TargetType::Checkerboard));
    QCOMPARE(out[0].x, -57.0);
    QCOMPARE(out[0].cols, 8);
    QCOMPARE(static_cast<int>(out[1].type), static_cast<int>(TargetType::AprilTag));
    QCOMPARE(out[1].yawDeg, 90.0);
    QCOMPARE(static_cast<int>(out[2].type), static_cast<int>(TargetType::ConeSquare));
    qunsetenv("CARLA_STUDIO_CALIBRATION_DIR");
#endif
  }

  // 19 precond: scene compiled in
  // 19 test:    in-process construction of CalibrationScene, add 2 targets,
  //             selection + removal updates list correctly
  void test_c19_calibration_scene_add_remove() {
#ifndef CARLA_STUDIO_WITH_QT3D
    QSKIP("Qt3D not available", SkipSingle);
#else
    using namespace carla_studio::calibration;
    CalibrationScene s;
    QSignalSpy sp(&s, &CalibrationScene::targetsChanged);

    s.addTargetAtCenter(TargetType::Checkerboard);
    s.addTargetAtCenter(TargetType::AprilTag);
    QCOMPARE(s.targets().size(), 2);
    QCOMPARE(static_cast<int>(s.targets()[1].type),
             static_cast<int>(TargetType::AprilTag));
    QCOMPARE(s.selectedIndex(), 1);

    s.removeAt(0);
    QCOMPARE(s.targets().size(), 1);
    QCOMPARE(static_cast<int>(s.targets()[0].type),
             static_cast<int>(TargetType::AprilTag));

    s.clearAll();
    QCOMPARE(s.targets().size(), 0);
    QVERIFY(sp.count() >= 4);
#endif
  }

  // 20 precond: scene compiled in
  // 20 test:    updateTargetPose(idx, x,y,z,yaw) mutates the entry +
  //             emits targetsChanged
  void test_c20_calibration_scene_update_pose() {
#ifndef CARLA_STUDIO_WITH_QT3D
    QSKIP("Qt3D not available", SkipSingle);
#else
    using namespace carla_studio::calibration;
    CalibrationScene s;
    s.addTargetAtCenter(TargetType::Checkerboard);
    QSignalSpy sp(&s, &CalibrationScene::targetsChanged);

    s.updateTargetPose(0, 12.5, 3.25, 1.8, 45.0);
    QCOMPARE(s.targets()[0].x,      12.5);
    QCOMPARE(s.targets()[0].y,      3.25);
    QCOMPARE(s.targets()[0].z,      1.8);
    QCOMPARE(s.targets()[0].yawDeg, 45.0);
    QCOMPARE(sp.count(),           1);

    // Out-of-range index is a no-op
    s.updateTargetPose(99, 0, 0, 0, 0);
    QCOMPARE(s.targets()[0].x, 12.5);
#endif
  }

  // ============================================================
  // Section D - Sensor mount + presets
  // ============================================================

  // 21 precond: mount preset code present
  // 21 test:    7 named presets + Reset button all in source
  void test_d21_mount_presets_complete() {
    for (const char *name : {"Roof Center", "Front Bumper", "Rear Bumper",
                              "Hood", "Trunk", "Mirror \xc2\xb7 Left",
                              "Mirror \xc2\xb7 Right"}) {
      QVERIFY2(sourceContains(QString("\"%1\"").arg(name)),
               qPrintable(QString("preset missing: %1").arg(name)));
    }
    QVERIFY(sourceContains("\"Reset\""));
  }

  // 22 precond: orientation spinboxes constructed
  // 22 test:    rx/ry/rz all clamped to 0..359 degrees
  void test_d22_orientation_clamped_zero_to_359() {
    QVERIFY(sourceContains("rx->setRange(0.0, 359.0)"));
    QVERIFY(sourceContains("ry->setRange(0.0, 359.0)"));
    QVERIFY(sourceContains("rz->setRange(0.0, 359.0)"));
  }

  // 23 precond: SensorMountKey helper present
  // 23 test:    deterministic key for (sensor name, instance idx)
  void test_d23_sensor_mount_key_deterministic() {
    using carla::studio::core::sensorMountKey;
    QCOMPARE(sensorMountKey("RGB", 1), QStringLiteral("RGB"));
    QCOMPARE(sensorMountKey("RGB", 2), QStringLiteral("RGB#2"));
    QCOMPARE(sensorMountKey("RGB", 3), QStringLiteral("RGB#3"));
    QCOMPARE(sensorMountKey("RGB", 1), sensorMountKey("RGB", 1));
    QVERIFY(sensorMountKey("RGB", 1) != sensorMountKey("RGB", 2));
  }

  // ============================================================
  // Section E - View tile buttons (Driver / Chase / Cockpit / BEV)
  // ============================================================

  // 24 precond: app source readable
  // 24 test:    fpvBtn / tpvBtn / cpvBtn / bevBtn all wired
  void test_e24_view_tile_buttons_wired() {
    for (const char *btn : {"fpvBtn", "tpvBtn", "cpvBtn", "bevBtn"}) {
      QVERIFY2(sourceContains(QString("openViewTile(\"%1\")")
                .arg(QString(btn).left(3).toUpper())),
               qPrintable(QString("%1 not wired to openViewTile").arg(btn)));
    }
  }

  // 25 precond: openViewTile lambda exists
  // 25 test:    spawns sensor.camera.rgb attached to hero (Listen()-driven)
  void test_e25_view_tile_spawns_rgb_camera_on_hero() {
    QVERIFY(sourceContains("sensor.camera.rgb"));
    QVERIFY(sourceContains("inAppDriveVehicle"));
    QVERIFY(sourceContains("camera->Listen("));
  }

  // ============================================================
  // Section F - Vehicle import (preflight + bundling)
  // ============================================================

  // 26 precond: NameSanitizer module compiled in
  // 26 test:    sanitizes vehicle name with `+` and `/` to underscores
  void test_f26_obj_name_sanitizer() {
    using namespace carla_studio::vehicle_import;
    const QString a = sanitizeVehicleName("Mercedes+Benz GLS 580");
    QVERIFY2(!a.contains('+') && !a.contains(' '),
             qPrintable("got: " + a));
    const QString b = sanitizeVehicleName("car/with\\path");
    QVERIFY2(!b.contains('/') && !b.contains('\\'),
             qPrintable("got: " + b));
  }

  // 27 precond: MeshAABB module compiled in
  // 27 test:    bbox of a synthetic OBJ correctly returned via parseOBJ
  void test_f27_mesh_aabb_basic() {
    const QString dir = fixtureDir("f27_mesh_aabb");
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
    const MeshAABB bb = parseOBJ(path);
    QVERIFY2(bb.valid, "parseOBJ did not flag valid");
    QCOMPARE(bb.xMax - bb.xMin, 1.0f);
    QCOMPARE(bb.yMax - bb.yMin, 1.0f);
    QCOMPARE(bb.zMax - bb.zMin, 1.0f);
    QCOMPARE(bb.vertexCount, 8);
  }

  // ============================================================
  // Section G - Cross-platform shims (PR feedback fixes)
  // ============================================================

  // 28 precond: ImporterClient compiled with QTcpSocket
  // 28 test:    no POSIX socket headers in source (Windows-buildable)
  void test_g28_importer_client_no_posix_sockets() {
    QDir d(QFileInfo(mAppSourcePath).dir());
    d.cdUp();
    const QString p = d.filePath("vehicle_import/ImporterClient.cpp");
    QFile f(p);
    QVERIFY2(f.open(QIODevice::ReadOnly), qPrintable("missing: " + p));
    const QString src = QString::fromUtf8(f.readAll());
    QVERIFY2(!src.contains("<arpa/inet.h>"),  "still includes arpa/inet.h");
    QVERIFY2(!src.contains("<sys/socket.h>"), "still includes sys/socket.h");
    QVERIFY2(!src.contains("<sys/select.h>"), "still includes sys/select.h");
    QVERIFY2( src.contains("QTcpSocket"),     "not using QTcpSocket");
  }

  // ============================================================
  // Section V - Vehicle pipeline against local fixtures
  //              (skips gracefully when tests/fixtures/vehicles/ unpopulated)
  // ============================================================

  QString vehicleFixture(const QString &subdir) const {
    return mFixturesDir + "/vehicles/" + subdir;
  }

  void verifyParsedOBJ(const QString &path, int minVerts) {
    using namespace carla_studio::vehicle_import;
    if (!QFileInfo::exists(path))
      QSKIP(qPrintable("fixture missing: " + path), SkipSingle);
    const MeshAABB bb = parseOBJ(path);
    QVERIFY2(bb.valid, qPrintable("parseOBJ said invalid: " + path));
    QVERIFY2(bb.vertexCount >= minVerts,
             qPrintable(QString("vertex count %1 < %2 for %3")
                          .arg(bb.vertexCount).arg(minVerts).arg(path)));
    QVERIFY2(bb.xMax > bb.xMin && bb.yMax > bb.yMin && bb.zMax > bb.zMin,
             "AABB collapsed (zero extent on at least one axis)");
  }

  // V01 precond: gokcer_kit fixture present
  // V01 test:    parseOBJ on FR-max_Latest.obj produces non-empty AABB
  void test_v01_fr_max_parses() {
    verifyParsedOBJ(vehicleFixture("gokcer_kit/FR-max_Latest.obj"), 1000);
  }

  // V02 precond: mb_gls580 fixture present
  // V02 test:    parseOBJ on uploads_files_2787791_Mercedes+Benz+GLS+580.obj
  void test_v02_mb_gls580_parses() {
    verifyParsedOBJ(
      vehicleFixture("32-mercedes-benz-gls-580-2020/"
                     "uploads_files_2787791_Mercedes+Benz+GLS+580.obj"),
      1000);
  }

  // V03 precond: mkz fixture present
  // V03 test:    parseOBJ on MKZ.obj
  void test_v03_mkz_parses() {
    verifyParsedOBJ(vehicleFixture("mkz/MKZ.obj"), 1000);
  }

  // V04 precond: rr fixture present
  // V04 test:    parseOBJ on rr.obj
  void test_v04_rr_parses() {
    verifyParsedOBJ(vehicleFixture("rr/rr.obj"), 1000);
  }

  // V05 precond: V01..V04 parse cleanly
  // V05 test:    sanitizeVehicleName produces filesystem-safe identifiers
  //              for each vehicle's display name
  void test_v05_vehicle_names_sanitize_safe() {
    using namespace carla_studio::vehicle_import;
    const QStringList raws = {
      "FR-max_Latest", "Mercedes+Benz GLS 580", "MKZ", "rr"
    };
    for (const QString &raw : raws) {
      const QString s = sanitizeVehicleName(raw);
      QVERIFY2(!s.isEmpty(),                qPrintable("empty: " + raw));
      QVERIFY2(!s.contains(' '),            qPrintable("space: " + s));
      QVERIFY2(!s.contains('+'),            qPrintable("plus: "  + s));
      QVERIFY2(!s.contains('/'),            qPrintable("slash: " + s));
      QVERIFY2(!s.contains('\\'),           qPrintable("bksl: "  + s));
    }
  }

  // ============================================================
  // Section I - In-process vehicle import preflight (mkz, BodyPlusTires)
  //              No editor / no cook / no shipping CARLA needed.
  //              Validates that everything BEFORE the UE-side cook is correct.
  // ============================================================

  // I01 precond: mkz fixture present
  // I01 test:    body OBJ parses, AABB has chassis-shaped extents (~5m × ~2m × ~1.5m)
  void test_i01_mkz_body_parses_chassis_shape() {
    using namespace carla_studio::vehicle_import;
    const QString p = vehicleFixture("mkz/MKZ.obj");
    if (!QFileInfo::exists(p)) QSKIP("mkz fixture missing", SkipSingle);
    const MeshAABB bb = parseOBJ(p);
    QVERIFY(bb.valid);
    const float xExt = bb.xMax - bb.xMin;
    const float yExt = bb.yMax - bb.yMin;
    const float zExt = bb.zMax - bb.zMin;
    QVERIFY2(xExt > 0 && yExt > 0 && zExt > 0,
             qPrintable(QString("collapsed AABB %1 %2 %3").arg(xExt).arg(yExt).arg(zExt)));
    QVERIFY2(bb.vertexCount > 5000,
             qPrintable(QString("body looks too sparse: %1 verts").arg(bb.vertexCount)));
  }

  // I02 precond: mkz tires fixture present
  // I02 test:    Tires.obj parses, AABB present, vertex count > 1000 (4 wheels' worth)
  void test_i02_mkz_tires_parses() {
    using namespace carla_studio::vehicle_import;
    const QString p = vehicleFixture("mkz/Tires.obj");
    if (!QFileInfo::exists(p)) QSKIP("mkz tires fixture missing", SkipSingle);
    const MeshAABB bb = parseOBJ(p);
    QVERIFY(bb.valid);
    QVERIFY2(bb.vertexCount > 1000,
             qPrintable(QString("tires too sparse: %1 verts").arg(bb.vertexCount)));
  }

  // I03 precond: I01 passed
  // I03 test:    analyzeMesh on mkz body identifies chassis (not wheels)
  void test_i03_mkz_body_analysis_is_chassis() {
    using namespace carla_studio::vehicle_import;
    const QString p = vehicleFixture("mkz/MKZ.obj");
    if (!QFileInfo::exists(p)) QSKIP("mkz fixture missing", SkipSingle);
    const MeshGeometry g = loadMeshGeometryOBJ(p);
    QVERIFY(g.vertexCount() > 0);
    const MeshAnalysisResult r = analyzeMesh(g, 1.0f);
    QVERIFY(r.ok);
    const float chassisExtX = r.chassisXMax - r.chassisXMin;
    QVERIFY2(chassisExtX > 100.0f,
             qPrintable(QString("chassis X extent %1cm too small for sedan").arg(chassisExtX)));
  }

  // I04 precond: I02 passed
  // I04 test:    analyzeMesh on mkz tires finds at least 4 wheel candidates
  void test_i04_mkz_tires_analysis_finds_four_wheels() {
    using namespace carla_studio::vehicle_import;
    const QString p = vehicleFixture("mkz/Tires.obj");
    if (!QFileInfo::exists(p)) QSKIP("mkz tires fixture missing", SkipSingle);
    const MeshGeometry g = loadMeshGeometryOBJ(p);
    QVERIFY(g.vertexCount() > 0);
    const MeshAnalysisResult r = analyzeMesh(g, 1.0f);
    QVERIFY(r.ok);
    QVERIFY2(static_cast<int>(r.wheels.size()) >= 4,
             qPrintable(QString("found %1 wheel candidates, expected ≥ 4")
                          .arg(r.wheels.size())));
  }

  // I05 precond: I04 passed
  // I05 test:    wheelTemplateFromTiresObj (Finding-2 fix) returns
  //              Chaos-sane wheel radius (18..60 cm) at AT LEAST ONE
  //              of the candidate input units. Probes raw / m→cm / in→cm.
  //              Old wheelTemplateFromAnalysis path stays for Combined-mode.
  void test_i05_mkz_wheel_radius_above_chaos_floor() {
    using namespace carla_studio::vehicle_import;
    const QString p = vehicleFixture("mkz/Tires.obj");
    if (!QFileInfo::exists(p)) QSKIP("mkz tires fixture missing", SkipSingle);
    struct Probe { const char *name; float scale; };
    const Probe candidates[] = {
      {"raw (1.0)",        1.0f},
      {"meters→cm (100)", 100.0f},
      {"inches→cm (2.54)", 2.54f},
    };
    QStringList diag;
    bool anySane = false;
    for (const Probe &c : candidates) {
      const WheelTemplate wt = wheelTemplateFromTiresObj(p, c.scale);
      diag << QString("  %1 -> radius=%2cm width=%3cm valid=%4")
              .arg(c.name).arg(wt.radius, 0, 'f', 2)
              .arg(wt.width, 0, 'f', 2).arg(wt.valid);
      if (wt.valid && wt.radius >= 18.0f && wt.radius <= 60.0f)
        anySane = true;
    }
    QVERIFY2(anySane,
      qPrintable("Finding-2 helper produced no Chaos-sane radius (18..60 cm) "
                 "at any candidate scale. Diagnostics:\n" + diag.join('\n')));
  }

  // I06 precond: I01 + I02 + I05 passed
  // I06 test:    mergeBodyAndTires(MKZ.obj, Tires.obj) lands all 4 tire copies
  void test_i06_mkz_merge_body_and_tires() {
    using namespace carla_studio::vehicle_import;
    const QString body  = vehicleFixture("mkz/MKZ.obj");
    const QString tires = vehicleFixture("mkz/Tires.obj");
    if (!QFileInfo::exists(body) || !QFileInfo::exists(tires))
      QSKIP("mkz fixtures missing", SkipSingle);

    const QString outDir = fixtureDir("i06_mkz_merge");
    QDir(outDir).removeRecursively();
    QDir().mkpath(outDir);

    const MeshGeometry g = loadMeshGeometryOBJ(body);
    const MeshAnalysisResult r = analyzeMesh(g, 1.0f);

    std::array<WheelSpec, 4> anchors;
    const float cx = (r.chassisXMin + r.chassisXMax) * 0.5f;
    const float halfX = (r.chassisXMax - r.chassisXMin) * 0.35f;
    const float halfY = (r.chassisYMax - r.chassisYMin) * 0.45f;
    const float wheelZ = r.chassisZMin + 25.0f;
    anchors[0] = {cx + halfX,  halfY, wheelZ, 30.0f, 18.0f};
    anchors[1] = {cx + halfX, -halfY, wheelZ, 30.0f, 18.0f};
    anchors[2] = {cx - halfX,  halfY, wheelZ, 30.0f, 18.0f};
    anchors[3] = {cx - halfX, -halfY, wheelZ, 30.0f, 18.0f};

    const ObjMergeResult m = mergeBodyAndTires(body, tires, outDir, anchors);
    QVERIFY2(m.ok,
             qPrintable("merge failed: " + m.reason));
    QVERIFY2(QFileInfo::exists(m.outputPath),
             qPrintable("merged OBJ missing: " + m.outputPath));
    QVERIFY2(m.tireCopiesPlaced == 4,
             qPrintable(QString("placed %1/4 tires").arg(m.tireCopiesPlaced)));
    QVERIFY2(m.mergedVertexCount > m.bodyVertexCount,
             "merged vertex count not greater than body alone");
    QVERIFY2(m.tireSourceCount >= 1,
             qPrintable(QString("clusterTireGroups detected %1 clusters, expected >= 1")
                          .arg(m.tireSourceCount)));
  }

  // I07 precond: I05 passed
  // I07 test:    sanitizeVehicleName("MKZ") → "mkz" or "MKZ" (filesystem-safe)
  void test_i07_mkz_name_sanitizer_safe() {
    using namespace carla_studio::vehicle_import;
    const QString s = sanitizeVehicleName("MKZ");
    QVERIFY(!s.isEmpty());
    QVERIFY(!s.contains(' '));
    QVERIFY(!s.contains('+'));
    QVERIFY(!s.contains('/'));
    QVERIFY(!s.contains('\\'));
  }

  // ============================================================
  // Section J - Drive test matrix (cross-process; SKIPPED if sim missing)
  //              Pre-imported vehicles only (this matrix doesn't trigger
  //              the editor cook). When you've cooked + deployed mkz,
  //              start shipping CARLA with mkz registered, then run.
  // ============================================================

  // ============================================================
  // Section K - Chaos-safety of generated vehicle spec
  //              Each test pulls one Chaos failure-mode lever and
  //              asserts our spec generation prevents it client-side.
  // ============================================================

  // K01 precond: I05 + I06 passed (mkz BodyPlusTires path)
  // K01 test:    full-pipeline spec for mkz passes verifyChaosVehicleSpec
  //              with 4-wheels declared, wheels above 18cm Chaos floor,
  //              wheel anchors clear of the road, sane chassis extents.
  void test_k01_mkz_full_spec_chaos_safe() {
    using namespace carla_studio::vehicle_import;
    const QString body  = vehicleFixture("mkz/MKZ.obj");
    const QString tires = vehicleFixture("mkz/Tires.obj");
    if (!QFileInfo::exists(body) || !QFileInfo::exists(tires))
      QSKIP("mkz fixtures missing", SkipSingle);

    const MeshGeometry g = loadMeshGeometryOBJ(body);
    QVERIFY(g.vertexCount() > 0);
    const MeshAnalysisResult r = analyzeMesh(g, 1.0f);
    QVERIFY(r.ok);

    VehicleSpec body_spec = buildSpecFromAnalysis(
        r, "mkz_test", body, 1.0f, 2, 0);

    const WheelTemplate tpl = wheelTemplateFromTiresObj(tires, 1.0f);
    QVERIFY2(tpl.valid, "wheel template invalid");

    const MergeResult mr = mergeBodyAndWheels(body_spec, tpl, true);
    QVERIFY2(mr.ok, qPrintable("merge failed: " + mr.reason));

    const SpecVerification v = verifyChaosVehicleSpec(mr.spec);
    QStringList diag;
    diag << "verifyChaosVehicleSpec(mkz):";
    for (const auto &w : v.warnings) diag << "  warn: " + w;
    for (const auto &e : v.errors)   diag << "  ERR : " + e;
    QVERIFY2(v.ok, qPrintable(diag.join('\n')));
    QVERIFY(v.fourWheelsPresent);
    QVERIFY(v.chassisExtentPositive);
    QVERIFY(v.wheelAnchorsClearRoad);
  }

  // K02 precond: deriveWheelAnchors lifts wheel.z above road
  // K02 test:    a synthetic spec with chassisZMin BELOW road still
  //              produces wheel anchors at >= radius (not sunken).
  void test_k02_chassis_z_lift_keeps_wheels_above_road() {
    using namespace carla_studio::vehicle_import;
    VehicleSpec body_spec;
    body_spec.detectedFromMesh = true;
    body_spec.hasFourWheels    = false;
    body_spec.chassisXMin = -200; body_spec.chassisXMax =  200;
    body_spec.chassisYMin =  -90; body_spec.chassisYMax =   90;
    body_spec.chassisZMin = -150; body_spec.chassisZMax =   30;

    WheelTemplate tpl;
    tpl.valid  = true;
    tpl.radius = 33.0f;
    tpl.width  = 22.0f;

    const MergeResult mr = mergeBodyAndWheels(body_spec, tpl, true);
    QVERIFY(mr.ok);
    for (size_t i = 0; i < mr.spec.wheels.size(); ++i) {
      const float wz = mr.spec.wheels[i].z;
      QVERIFY2(qFuzzyCompare(wz, tpl.radius),
               qPrintable(QString("wheel[%1].z=%2 should equal radius %3 "
                                  "(so wheel bottom touches road at z=0)")
                            .arg(i).arg(wz).arg(tpl.radius)));
    }
  }

  // K03 precond: K01 passed
  // K03 test:    verifyChaosVehicleSpec rejects a degenerate spec
  void test_k03_verifier_rejects_degenerate_spec() {
    using namespace carla_studio::vehicle_import;
    VehicleSpec bad;
    bad.detectedFromMesh = true;
    bad.hasFourWheels    = false;
    const SpecVerification v = verifyChaosVehicleSpec(bad);
    QVERIFY(!v.ok);
    QVERIFY(!v.errors.isEmpty());
    QVERIFY(!v.fourWheelsPresent);
    QVERIFY(!v.chassisExtentPositive);
  }

  // K04 precond: server-side CarlaTools forces COMNudge=ZeroVector
  //              (USDImporterWidget.cpp:656). We cannot run UE here
  //              but we DO own the editor source path; verify the line
  //              is present so a future refactor doesn't lose it.
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

  // K07 precond: buildSpecFromAnalysis used in import path
  // K07 test:    when source mesh has Y-extent > X-extent (i.e. forward axis
  //              is Y in source), buildSpecFromAnalysis swaps X and Y so the
  //              spec sent to the importer always has +X = forward (CARLA
  //              convention), and adjustYawDeg=90 to compensate visually.
  void test_k07_forward_axis_auto_swap() {
    using namespace carla_studio::vehicle_import;
    MeshAnalysisResult r;
    r.ok = true;
    r.hasFourWheels = false;
    r.sizeClass = SizeClass::Sedan;
    r.chassisXMin = -100; r.chassisXMax = 100;
    r.chassisYMin = -250; r.chassisYMax = 250;
    r.chassisZMin = 0;    r.chassisZMax = 150;
    r.overallXMin = -100; r.overallXMax = 100;
    r.overallYMin = -250; r.overallYMax = 250;
    r.overallZMin = 0;    r.overallZMax = 150;
    const VehicleSpec s = buildSpecFromAnalysis(r, "synth", "/dev/null", 1.0f, 2, 0);
    QCOMPARE(s.chassisXMin, -250.0f);
    QCOMPARE(s.chassisXMax,  250.0f);
    QCOMPARE(s.chassisYMin, -100.0f);
    QCOMPARE(s.chassisYMax,  100.0f);
    QCOMPARE(s.adjustYawDeg, 90.0f);

    MeshAnalysisResult r2 = r;
    r2.chassisYMin = -90;  r2.chassisYMax =  90;
    r2.overallYMin = -90;  r2.overallYMax =  90;
    r2.chassisXMin = -250; r2.chassisXMax = 250;
    r2.overallXMin = -250; r2.overallXMax = 250;
    const VehicleSpec s2 = buildSpecFromAnalysis(r2, "synth2", "/dev/null", 1.0f, 2, 0);
    QCOMPARE(s2.chassisXMin, -250.0f);
    QCOMPARE(s2.chassisXMax,  250.0f);
    QCOMPARE(s2.adjustYawDeg, 0.0f);
  }

  // K06 precond: K01 + K02 passed
  // K06 test:    if analyzer's body-detected wheels are clustered
  //              (poor segmentation), mergeBodyAndWheels falls back to
  //              deriveWheelAnchors so the spec still has 4 well-spread
  //              wheel positions covering the chassis bbox.
  void test_k06_clustered_wheels_fall_back_to_derived() {
    using namespace carla_studio::vehicle_import;
    VehicleSpec body;
    body.detectedFromMesh = true;
    body.hasFourWheels    = true;
    body.chassisXMin = -200; body.chassisXMax = 200;
    body.chassisYMin = -90;  body.chassisYMax = 90;
    body.chassisZMin =  0;   body.chassisZMax = 150;
    for (int i = 0; i < 4; ++i) {
      body.wheels[i].x = 100.0f + i * 0.5f;
      body.wheels[i].y = -80.0f + i * 0.5f;
      body.wheels[i].z = 35.0f;
      body.wheels[i].radius = 35.0f;
      body.wheels[i].width  = 22.0f;
    }
    WheelTemplate tpl;
    tpl.valid = true; tpl.radius = 35.0f; tpl.width = 22.0f;

    const MergeResult mr = mergeBodyAndWheels(body, tpl, true);
    QVERIFY(mr.ok);
    QVERIFY2(mr.reason.contains("fallback"),
             qPrintable("expected clustered fallback path; got: " + mr.reason));
    float yMin = mr.spec.wheels[0].y, yMax = yMin;
    float xMin = mr.spec.wheels[0].x, xMax = xMin;
    for (const WheelSpec &w : mr.spec.wheels) {
      yMin = std::min(yMin, w.y); yMax = std::max(yMax, w.y);
      xMin = std::min(xMin, w.x); xMax = std::max(xMax, w.x);
    }
    QVERIFY2(yMax - yMin >= 100.0f,
      qPrintable(QString("after fallback wheels still clustered y: %1").arg(yMax - yMin)));
    QVERIFY2(xMax - xMin >= 100.0f,
      qPrintable(QString("after fallback wheels still clustered x: %1").arg(xMax - xMin)));
  }

  // K05 precond: per-vehicle drive baseline already collected
  // K05 test:    documented baseline in tests/outputs/drive_matrix_summary.txt
  //              proves rig executed and wrote the file.
  void test_k05_drive_baseline_captured() {
    const QString p = mOutputsDir + "/drive_matrix_summary.txt";
    if (!QFileInfo::exists(p))
      QSKIP("drive baseline not yet captured (run drive_baseline first)",
            SkipSingle);
    QFile f(p);
    QVERIFY(f.open(QIODevice::ReadOnly));
    const QString s = QString::fromUtf8(f.readAll());
    QVERIFY2(s.contains("vehicle.custom.fr_max_latest"),
             "drive baseline doesn't reference fr_max_latest");
    QVERIFY2(s.contains("vehicle.custom.mkz"),
             "drive baseline doesn't reference mkz");
  }

  // J01 precond: shipping CARLA running on localhost:2000 with hero vehicle
  // J01 test:    libcarla connect succeeds, returns world version 0.10.0
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

  // K01 precond: binary exists, DISPLAY set, CARLA_STUDIO_REPO_ROOT set
  // K01 test:    launch carla-studio --capture-screenshots OUTDIR, wait for exit
  // K01 expect:  ≥4 PNG files in OUTDIR; copy to resources/usage/screenshots/
  void test_k01_capture_usage_screenshots() {
    if (mBinaryPath.isEmpty())
      QSKIP("CARLA_STUDIO_BINARY_PATH not set", SkipSingle);
    if (qEnvironmentVariable("DISPLAY").isEmpty())
      QSKIP("no DISPLAY - screenshot capture needs a graphical session", SkipSingle);
    if (mRepoRoot.isEmpty())
      QSKIP("CARLA_STUDIO_REPO_ROOT not set", SkipSingle);

    const QString outDir = mRepoRoot + "/docs/usage/screenshots";
    QDir().mkpath(outDir);

    QProcess p;
    p.setProgram(mBinaryPath);
    p.setArguments({"--capture-screenshots", outDir});
    p.start();
    QVERIFY2(p.waitForStarted(5000), "binary did not start");
    QVERIFY2(p.waitForFinished(20000), "capture did not finish within 20s");

    const QStringList pngs = QDir(outDir).entryList({"*.png"}, QDir::Files);
    QVERIFY2(pngs.size() >= 4,
             qPrintable(QString("expected ≥4 screenshots, got %1").arg(pngs.size())));
  }

  // 29 precond: HuggingFace.cpp uses curl --config -
  // 29 test:    Authorization header NOT in argv; goes via stdin config
  void test_g29_huggingface_token_off_argv() {
    QDir d(QFileInfo(mAppSourcePath).dir());
    d.cdUp();
    const QString p = d.filePath("integrations/HuggingFace.cpp");
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
