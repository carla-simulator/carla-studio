// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later
//

#include "vehicle_import/CarlaSpawn.h"
#include "vehicle_import/ImporterClient.h"
#include "vehicle_import/VehicleSpec.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QRegularExpression>
#include <QString>
#include <QStringList>
#include <QStandardPaths>
#include <QTextStream>
#include <QThread>

#include <chrono>
#include <cstdio>
#include <cstdlib>

#ifdef CARLA_STUDIO_WITH_LIBCARLA
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/Actor.h>
#include <carla/client/ActorList.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Vehicle.h>
#include <carla/geom/Transform.h>
#include <cmath>
#endif

extern "C" int system(const char *);

namespace cs = carla_studio::vehicle_import;

namespace {

struct Fixture {
  QString name;
  QString meshPath;
};

QList<Fixture> loadFixtures()
{
  QList<Fixture> out;
  const QString root = qEnvironmentVariable("CARLA_VEHICLE_TEST_FIXTURES_DIR");
  if (root.isEmpty()) return out;
  QDir d(root);
  const QStringList exts = { "*.obj", "*.fbx", "*.glb", "*.gltf", "*.dae", "*.blend" };
  for (const QFileInfo &fi : d.entryInfoList(exts, QDir::Files, QDir::Name)) {
    out.push_back({ fi.completeBaseName().toLower().replace(QRegularExpression("[^a-z0-9_]+"), "_"),
                    fi.absoluteFilePath() });
  }
  return out;
}

QString tail(const QString &path, int lines)
{
  QProcess p;
  p.start("/bin/sh", QStringList() << "-c"
      << QString("tail -n %1 %2 2>/dev/null").arg(lines).arg(path));
  p.waitForFinished(2000);
  return QString::fromLocal8Bit(p.readAllStandardOutput());
}

bool portOpen(int port)
{
  QProcess p;
  p.start("/bin/sh", QStringList() << "-c"
      << QString("ss -tln 2>/dev/null | grep -q ':%1 '").arg(port));
  p.waitForFinished(1500);
  return p.exitCode() == 0;
}

qint64 launchEditor(int port, const QString &logPath)
{
  const QString engine  = qEnvironmentVariable("CARLA_UNREAL_ENGINE_PATH");
  const QString srcRoot = qEnvironmentVariable("CARLA_SRC_ROOT");
  if (engine.isEmpty() || srcRoot.isEmpty()) return -1;
  const QString editor = engine + "/Engine/Binaries/Linux/UnrealEditor";
  const QString uproject = srcRoot + "/Unreal/CarlaUnreal/CarlaUnreal.uproject";

  auto shEsc = [](const QString &s) {
    QString out = s; out.replace('\'', "'\\''"); return "'" + out + "'";
  };
  const QString cmd = QString("CARLA_VEHICLE_IMPORTER_PORT=%1 exec %2 %3 "
      "-unattended -RenderOffScreen -nosplash -nosound >>%4 2>&1 & echo $!")
      .arg(port).arg(shEsc(editor), shEsc(uproject), shEsc(logPath));
  QProcess p;
  p.start("/bin/sh", QStringList() << "-c" << cmd);
  p.waitForFinished(5000);
  bool ok = false;
  const qint64 pid = QString::fromLocal8Bit(p.readAllStandardOutput()).trimmed().toLongLong(&ok);
  return ok ? pid : -1;
}

bool waitPort(int port, int seconds)
{
  for (int i = 0; i < seconds; ++i) {
    if (portOpen(port)) return true;
    QThread::sleep(1);
  }
  return false;
}

void killProcess(qint64 pid)
{
  if (pid > 0) { ::system(QString("kill -9 %1 2>/dev/null").arg(pid).toLocal8Bit().constData()); }
}

QString findShippingScript(const QString &shippingRoot)
{
  for (const QString &name : { QStringLiteral("CarlaUnreal.sh"),
                               QStringLiteral("CarlaUE5.sh"),
                               QStringLiteral("CarlaUE4.sh") }) {
    const QString candidate = shippingRoot + "/" + name;
    if (QFileInfo(candidate).isFile()) return candidate;
  }
  return QString();
}

qint64 launchShippingCarla(const QString &script, int rpcPort, const QString &logPath,
                           bool visible)
{
  auto shEsc = [](const QString &s) {
    QString out = s; out.replace('\'', "'\\''"); return "'" + out + "'";
  };
  const QString modeFlag = QStringLiteral("-nullrhi");
  const QString cmd = QString("exec %1 %2 -nosound -quality-level=Low "
      "-carla-rpc-port=%3 -carla-streaming-port=%4 "
      ">>%5 2>&1 & echo $!")
      .arg(shEsc(script)).arg(modeFlag).arg(rpcPort).arg(rpcPort + 1).arg(shEsc(logPath));
  QProcess p;
  p.start("/bin/sh", QStringList() << "-c" << cmd);
  p.waitForFinished(5000);
  bool ok = false;
  const qint64 pid = QString::fromLocal8Bit(p.readAllStandardOutput()).trimmed().toLongLong(&ok);
  return ok ? pid : -1;
}

cs::VehicleSpec makeSpec(const Fixture &f)
{
  cs::VehicleSpec s;
  s.name          = QString(f.name);
  s.meshPath      = QString(f.meshPath);
  s.contentPath   = "/Game/Carla/Static/Vehicles/4Wheeled";
  s.baseVehicleBP = "/Game/Carla/Blueprints/Vehicles/BaseVehiclePawnNW";
  s.mass          = 1500.0f;
  s.suspDamping   = 0.55f;
  for (int i = 0; i < 4; ++i) {
    s.wheels[i].radius         = 35.f;
    s.wheels[i].width          = 22.f;
    s.wheels[i].maxSteerAngle  = (i < 2) ? 35.f : 0.f;
    s.wheels[i].maxBrakeTorque = 1500.f;
    s.wheels[i].suspMaxRaise   = 20.f;
    s.wheels[i].suspMaxDrop    = 50.f;
  }
  s.wheels[0].x =  140; s.wheels[0].y = -80; s.wheels[0].z = 71;
  s.wheels[1].x =  140; s.wheels[1].y =  80; s.wheels[1].z = 71;
  s.wheels[2].x = -140; s.wheels[2].y = -80; s.wheels[2].z = 71;
  s.wheels[3].x = -140; s.wheels[3].y =  80; s.wheels[3].z = 71;
  return s;
}

struct ImportRow {
  Fixture  fixture;
  bool     imported = false;
  QString  assetPath;
  QString  importDetail;
  bool     deployed = false;
  int      filesCopied = 0;
  int      cookedFiles = 0;
  QString  deployDetail;
  bool     spawned = false;
  QString  spawnDetail;
};

}  // namespace

int main(int argc, char **argv)
{
  QCoreApplication app(argc, argv);
  int  port         = 18584;
  int  carlaPort    = 3000;
  bool spawnEditor  = true;
  bool fullDrive    = false;
  bool spawnCarla   = true;
  bool visibleCarla = false;
  bool screenshots  = false;
  bool spawnOnly    = false;  // skip import+deploy, jump straight to CARLA spawn+drive
  QString screenshotsDir = "/tmp/cs_carla_shots";
  QString compareWith;  // e.g. "vehicle.taxi.ford" — spawn this 5 m to the right for visual ref
  for (int i = 1; i < argc; ++i) {
    const QString a = QString::fromLocal8Bit(argv[i]);
    if      (a == "--port"             && i + 1 < argc) port      = QString(argv[++i]).toInt();
    else if (a == "--carla-port"       && i + 1 < argc) carlaPort = QString(argv[++i]).toInt();
    else if (a == "--no-spawn-editor")                  spawnEditor = false;
    else if (a == "--no-spawn-carla")                   spawnCarla  = false;
    else if (a == "--full-drive")                       fullDrive   = true;
    else if (a == "--spawn-only")                     { spawnOnly = true; fullDrive = true; spawnEditor = false; }
    else if (a == "--visible")                          visibleCarla = true;
    else if (a == "--screenshots")                      screenshots = true;
    else if (a == "--shots-dir"        && i + 1 < argc) screenshotsDir = QString(argv[++i]);
    else if (a == "--compare-with"     && i + 1 < argc) compareWith = QString(argv[++i]);
  }
  if (screenshots) visibleCarla = true;  // screenshots imply visible carla
  qputenv("CARLA_VEHICLE_IMPORTER_PORT", QByteArray::number(port));

  QTextStream out(stdout);

  if (fullDrive && !spawnOnly) {
    const QStringList required = { "CARLA_SRC_ROOT", "CARLA_UNREAL_ENGINE_PATH",
                                   "CARLA_SHIPPING_ROOT", "CARLA_VEHICLE_TEST_FIXTURES_DIR" };
    QStringList missing;
    for (const QString &k : required)
      if (qEnvironmentVariable(k.toLocal8Bit().constData()).isEmpty()) missing << k;
    if (!missing.isEmpty()) {
      out << "--full-drive needs the following env vars set: " << missing.join(", ") << "\n";
      return 7;
    }
  }

  qint64 editorPid = -1;
  const QString editorLog = QString("/tmp/cs_test_editor_%1.log").arg(port);
  if (spawnEditor) {
    if (portOpen(port)) {
      out << "Port " << port << " already in use — pass --no-spawn-editor or pick another with --port.\n";
      return 2;
    }
    QFile::remove(editorLog);
    out << "Launching headless editor on port " << port << " (stdout→" << editorLog << ") …\n";
    editorPid = launchEditor(port, editorLog);
    if (editorPid <= 0) { out << "Editor launch failed.\n"; return 3; }
    out << "Editor pid: " << editorPid << "\n";
    out << "Waiting up to 240 s for port " << port << " …\n";
    if (!waitPort(port, 240)) {
      out << "Editor never opened port. Tail of log:\n" << tail(editorLog, 25) << "\n";
      killProcess(editorPid);
      return 4;
    }
    out << "Editor ready.\n";
  } else if (!spawnOnly && !portOpen(port)) {
    out << "No editor on port " << port << " and --no-spawn-editor passed. Aborting.\n";
    return 5;
  }

  const QList<Fixture> fixtures = loadFixtures();
  if (fixtures.isEmpty()) {
    out << "No fixtures found. Set CARLA_VEHICLE_TEST_FIXTURES_DIR to a "
           "directory containing .obj/.fbx/.glb/.gltf/.dae/.blend test meshes.\n";
    if (spawnEditor && editorPid > 0) killProcess(editorPid);
    return 6;
  }

  QList<ImportRow> rows;
  rows.reserve(fixtures.size());

  const QString srcRoot      = qEnvironmentVariable("CARLA_SRC_ROOT");
  const QString enginePath   = qEnvironmentVariable("CARLA_UNREAL_ENGINE_PATH");
  const QString shippingRoot = qEnvironmentVariable("CARLA_SHIPPING_ROOT");
  const QString uproject     = srcRoot + "/Unreal/CarlaUnreal/CarlaUnreal.uproject";
  const QString editorBinary = enginePath + "/Engine/Binaries/Linux/UnrealEditor";

  if (spawnOnly) {
    out << "(--spawn-only: skipping import + deploy, using pre-cooked assets)\n";
    for (const Fixture &f : fixtures) {
      ImportRow r; r.fixture = f;
      r.imported = true;
      r.deployed = true;
      r.assetPath = QString("/Game/Carla/Static/Vehicles/4Wheeled/%1/BP_%1").arg(f.name);
      rows.append(r);
    }
  } else {
    out << "\n=== IMPORT ===\n";
    for (const Fixture &f : fixtures) {
      ImportRow r; r.fixture = f;
      if (!QFile::exists(f.meshPath)) {
        r.importDetail = "mesh missing on disk";
        out << QString("[skip] %1 — mesh missing at %2\n").arg(f.name, f.meshPath);
        rows.append(r);
        continue;
      }
      cs::VehicleSpec s = makeSpec(f);
      QJsonObject json = cs::specToJson(s);
      QString resp = cs::sendJson(json);
      if (resp.isEmpty()) {
        r.importDetail = "no response from editor";
        out << QString("[FAIL] %1 — no response from editor\n").arg(f.name);
        rows.append(r);
        continue;
      }
      const QJsonObject obj = QJsonDocument::fromJson(resp.toUtf8()).object();
      r.imported     = obj.value("success").toBool();
      r.assetPath    = obj.value("asset_path").toString();
      r.importDetail = r.imported ? r.assetPath : obj.value("error").toString();
      out << QString(r.imported ? "[ OK ] %1 → %2\n" : "[FAIL] %1 — %2\n")
              .arg(f.name, r.importDetail);
      rows.append(r);
    }

    int importPasses = 0;
    for (const ImportRow &r : rows) if (r.imported) ++importPasses;
    out << importPasses << " / " << fixtures.size() << " imports succeeded.\n";

    if (!fullDrive) {
      if (spawnEditor && editorPid > 0) {
        out << "Killing editor pid " << editorPid << " …\n";
        killProcess(editorPid);
      }
      return importPasses == fixtures.size() ? 0 : 1;
    }

    if (editorPid > 0) {
      out << "\nKilling import editor pid " << editorPid << " before cook commandlet …\n";
      killProcess(editorPid);
      editorPid = -1;
      QThread::sleep(2);
    }

    out << "\n=== REGISTER + DEPLOY (cook) ===\n";
    out << "uproject     : " << uproject << "\n";
    out << "shippingRoot : " << shippingRoot << "\n";
    for (ImportRow &r : rows) {
      if (!r.imported) continue;
      cs::VehicleRegistration reg;
      reg.uprojectPath      = uproject;
      reg.shippingCarlaRoot = shippingRoot;
      reg.bpAssetPath       = r.assetPath;
      reg.make              = "Custom";
      reg.model             = r.fixture.name;
      reg.editorBinary      = editorBinary;

      out << "\n[" << r.fixture.name << "] registering …\n";
      const cs::RegisterResult rr = cs::registerVehicleInJson(reg);
      out << "  register: " << (rr.ok ? "OK" : "FAIL") << " — " << rr.detail << "\n";
      if (!rr.ok) { r.deployDetail = rr.detail; continue; }

      out << "[" << r.fixture.name << "] deploying (cook commandlet) — 2-3 min first cook …\n";
      const cs::DeployResult dr = cs::deployVehicleToShippingCarla(reg);
      r.deployed     = dr.ok;
      r.filesCopied  = dr.filesCopied;
      r.cookedFiles  = dr.cookedFiles;
      r.deployDetail = dr.detail;
      out << "  deploy: " << (dr.ok ? "OK" : "FAIL")
          << " — cooked=" << dr.cookedFiles
          << " copied=" << dr.filesCopied
          << " — " << dr.detail << "\n";
    }

    int deployPasses = 0;
    for (const ImportRow &r : rows) if (r.deployed) ++deployPasses;
    out << "\n" << deployPasses << "/" << importPasses << " deploys succeeded.\n";

    if (deployPasses == 0) {
      out << "No vehicles deployed; skipping spawn phase.\n";
      return 1;
    }
  }

  qint64 carlaPid = -1;
  const QString carlaLog = QString("/tmp/cs_test_carla_%1.log").arg(carlaPort);
  if (spawnCarla) {
    if (portOpen(carlaPort)) {
      out << "RPC port " << carlaPort
          << " already in use — pass --no-spawn-carla or pick --carla-port.\n";
      return 8;
    }
    const QString script = findShippingScript(shippingRoot);
    if (script.isEmpty()) {
      out << "Shipping CARLA launcher not found in " << shippingRoot
          << " (need CarlaUnreal.sh / CarlaUE5.sh / CarlaUE4.sh).\n";
      return 9;
    }
    QFile::remove(carlaLog);
    out << "\n=== LAUNCH SHIPPING CARLA ===\n";
    out << "script: " << script << " on RPC port " << carlaPort
        << " (stdout→" << carlaLog << ") …\n";
    carlaPid = launchShippingCarla(script, carlaPort, carlaLog, visibleCarla);
    if (carlaPid <= 0) {
      out << "CARLA launch failed.\n";
      return 10;
    }
    out << "CARLA pid: " << carlaPid << " — waiting up to 120 s for RPC port …\n";
    if (!waitPort(carlaPort, 120)) {
      out << "CARLA never opened RPC port. Tail of log:\n"
          << tail(carlaLog, 30) << "\n";
      killProcess(carlaPid);
      return 11;
    }
    out << "CARLA RPC ready on " << carlaPort << ".\n";
    QThread::sleep(5);  // give the world a moment to settle before spawn calls
  } else if (!portOpen(carlaPort)) {
    out << "No CARLA on RPC port " << carlaPort
        << " and --no-spawn-carla passed. Aborting.\n";
    return 12;
  }

  out << "\n=== SPAWN ===\n";
  QDir().mkpath(screenshotsDir);
  for (ImportRow &r : rows) {
    if (!r.deployed) continue;
    const cs::SpawnResult sr = cs::spawnInRunningCarla(
        "Custom", r.fixture.name, "localhost", carlaPort);
    r.spawned     = (sr.kind == cs::SpawnResult::Kind::Spawned);
    r.spawnDetail = sr.detail;
    out << QString(r.spawned ? "[ OK ] %1 — %2\n" : "[FAIL] %1 — %2\n")
            .arg(r.fixture.name, sr.detail);

#ifdef CARLA_STUDIO_WITH_LIBCARLA
    if (r.spawned) {
      try {
        carla::client::Client client("localhost", carlaPort);
        client.SetTimeout(std::chrono::seconds(20));
        auto world = client.GetWorld();
        std::shared_ptr<carla::client::Actor> ours;
        const std::string modelTail = "." + r.fixture.name.toLower().toStdString();
        auto all = world.GetActors();
        for (size_t i = 0; i < all->size(); ++i) {
          auto a = all->at(i);
          if (!a) continue;
          const std::string tid = a->GetTypeId();
          if (tid.size() > modelTail.size() &&
              tid.compare(tid.size() - modelTail.size(), modelTail.size(), modelTail) == 0) {
            if (!ours || a->GetId() > ours->GetId()) ours = a;
          }
        }
        if (ours) {
          if (auto v = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
            v->SetAutopilot(false);
            carla::rpc::VehicleControl brk0; brk0.brake = 1.f; brk0.throttle = 0.f;
            v->ApplyControl(brk0);
            v->SetSimulatePhysics(false);
          }
          QThread::msleep(500);
          ours->SetTargetVelocity(carla::geom::Vector3D(0.f, 0.f, 0.f));
          if (auto v = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
            v->SetSimulatePhysics(true);
          }
          QThread::sleep(2);  // let physics settle and TM release before recording t0
          auto spectator = world.GetSpectator();
          auto t0 = ours->GetTransform();
          if (auto v = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
            const auto &bb = v->GetBoundingBox();
            out << "  bbox extent (cm): X=" << bb.extent.x*100 << " Y=" << bb.extent.y*100 << " Z=" << bb.extent.z*100
                << "  spawn Z (m): " << t0.location.z << "  yaw: " << t0.rotation.yaw << "°\n";
            out.flush();
          }
          const double yawRad = double(t0.rotation.yaw) * (M_PI / 180.0);
          carla::geom::Transform sp;
          sp.location.x = t0.location.x - 8.0f * float(std::cos(yawRad));
          sp.location.y = t0.location.y - 8.0f * float(std::sin(yawRad));
          sp.location.z = t0.location.z + 4.0f;
          sp.rotation.pitch = -15.0f;
          sp.rotation.yaw   = t0.rotation.yaw;
          sp.rotation.roll  = 0.0f;
          spectator->SetTransform(sp);

          std::shared_ptr<carla::client::Actor> refActor;
          if (!compareWith.isEmpty()) {
            try {
              auto bps = world.GetBlueprintLibrary();
              auto refBp = bps->Find(compareWith.toStdString());
              if (refBp) {
                carla::geom::Transform refT = t0;
                const double yawR = double(t0.rotation.yaw) * (M_PI / 180.0);
                refT.location.x = t0.location.x + 5.0f * float(-std::sin(yawR));
                refT.location.y = t0.location.y + 5.0f * float( std::cos(yawR));
                refActor = world.TrySpawnActor(*refBp, refT);
                if (refActor) {
                  if (auto rv = std::dynamic_pointer_cast<carla::client::Vehicle>(refActor)) {
                    rv->SetAutopilot(false);
                    rv->SetSimulatePhysics(true);
                  }
                  out << QString("  ref: spawned %1 (id %2) 5 m right of %3\n")
                          .arg(compareWith).arg(refActor->GetId()).arg(r.fixture.name);
                  carla::geom::Transform mid;
                  mid.location.x = 0.5f * (t0.location.x + refT.location.x);
                  mid.location.y = 0.5f * (t0.location.y + refT.location.y);
                  mid.location.z = t0.location.z;
                  carla::geom::Transform sp2;
                  sp2.location.x = mid.location.x - 9.0f * float(std::cos(yawRad));
                  sp2.location.y = mid.location.y - 9.0f * float(std::sin(yawRad));
                  sp2.location.z = mid.location.z + 4.5f;
                  sp2.rotation.pitch = -18.0f;
                  sp2.rotation.yaw   = t0.rotation.yaw;
                  spectator->SetTransform(sp2);
                } else {
                  out << QString("  ref: TrySpawnActor returned null for %1 — point may "
                                 "be blocked or BP not in library\n").arg(compareWith);
                }
              } else {
                out << QString("  ref: blueprint '%1' not in library — skip\n").arg(compareWith);
              }
            } catch (const std::exception &e) {
              out << QString("  ref: spawn threw %1\n").arg(QString::fromUtf8(e.what()));
            } catch (...) {
              out << "  ref: spawn threw unknown\n";
            }
          }

          QThread::sleep(2);  // let the renderer catch up

          const QString shot1 = QString("%1/%2_01_at_spawn.png")
                                  .arg(screenshotsDir, r.fixture.name);
          ::system(QString("scrot %1 2>/dev/null").arg(shot1).toLocal8Bit().constData());
          out << QString("  shot → %1\n").arg(shot1);
          out.flush();

          struct DirResult { const char *label; float dist; float fwdDot; float yawDelta; };

          auto doDir = [&](const char *label, float throttle, float steer, bool reverse) -> DirResult {
            DirResult d; d.label = label;
            if (auto v2 = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
              v2->SetAutopilot(false);
              carla::rpc::VehicleControl brk; brk.brake = 1.f; brk.throttle = 0.f;
              v2->ApplyControl(brk);
              v2->SetSimulatePhysics(false);  // freeze physics so teleport has no velocity impulse
            }
            QThread::msleep(300);
            ours->SetTransform(t0);
            ours->SetTargetVelocity(carla::geom::Vector3D(0.f, 0.f, 0.f));
            QThread::msleep(200);
            if (auto v2 = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
              v2->SetSimulatePhysics(true);
            }
            QThread::msleep(1500);  // let physics settle before measuring

            auto ts = ours->GetTransform();
            const float yawS = ts.rotation.yaw;
            if (auto v2 = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
              carla::rpc::VehicleControl ctl;
              ctl.throttle = throttle; ctl.steer = steer;
              ctl.brake = 0.f; ctl.reverse = reverse;
              v2->ApplyControl(ctl);
            }
            for (int s = 0; s < 5; ++s) {
              QThread::sleep(1);
              try {
                auto t = ours->GetTransform();
                const double yr = double(t.rotation.yaw) * (M_PI / 180.0);
                carla::geom::Transform sp2;
                sp2.location.x = t.location.x - 8.f * float(std::cos(yr));
                sp2.location.y = t.location.y - 8.f * float(std::sin(yr));
                sp2.location.z = t.location.z + 4.f;
                sp2.rotation.pitch = -15.f; sp2.rotation.yaw = t.rotation.yaw;
                spectator->SetTransform(sp2);
              } catch (...) {}
            }
            auto te = ours->GetTransform();
            const double dx = te.location.x - ts.location.x;
            const double dy = te.location.y - ts.location.y;
            d.dist   = float(std::sqrt(dx*dx + dy*dy));
            d.fwdDot = float(dx * std::cos(yawRad) + dy * std::sin(yawRad));
            float dyaw = te.rotation.yaw - yawS;
            while (dyaw >  180.f) dyaw -= 360.f;
            while (dyaw < -180.f) dyaw += 360.f;
            d.yawDelta = dyaw;

            const QString shot = QString("%1/%2_%3.png")
                                   .arg(screenshotsDir, r.fixture.name,
                                        QString::fromUtf8(label).toLower());
            ::system(QString("scrot %1 2>/dev/null").arg(shot).toLocal8Bit().constData());
            out << QString("    shot → %1\n").arg(shot);
            out.flush();
            return d;
          };

          DirResult fwd  = doDir("FWD",  1.0f,  0.0f, false);
          DirResult rev  = doDir("REV",  1.0f,  0.0f, true );
          DirResult left = doDir("LEFT", 0.5f, -1.0f, false);
          DirResult rght = doDir("RGHT", 0.5f, +1.0f, false);

          if (auto v2 = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
            carla::rpc::VehicleControl brk; brk.brake = 1.f; brk.throttle = 0.f;
            v2->ApplyControl(brk);
          }

          out << QString("  drive matrix (5s each):\n");
          const DirResult allDirs[] = { fwd, rev, left, rght };
          for (const DirResult &d : allDirs) {
            out << QString("    %1  dist=%2m  fwd-dot=%3m  yaw=%4°\n")
                    .arg(QString::fromUtf8(d.label), -4)
                    .arg(d.dist,     6, 'f', 2)
                    .arg(d.fwdDot,   7, 'f', 2)
                    .arg(d.yawDelta, 6, 'f', 1);
          }
          out.flush();
          r.spawnDetail += QString(" | fwd=%1m rev=%2m L=%3° R=%4°")
              .arg(fwd.fwdDot, 0, 'f', 1).arg(rev.fwdDot, 0, 'f', 1)
              .arg(left.yawDelta, 0, 'f', 0).arg(rght.yawDelta, 0, 'f', 0);
        } else {
          out << QString("  (visual: could not find spawned actor with model tail %1)\n")
                  .arg(QString::fromStdString(modelTail));
        }
      } catch (const std::exception &e) {
        out << QString("  (visual: libcarla threw: %1)\n").arg(QString::fromUtf8(e.what()));
      } catch (...) {
        out << "  (visual: unknown exception)\n";
      }
    }
#endif
  }

  int spawnPasses = 0;
  for (const ImportRow &r : rows) if (r.spawned) ++spawnPasses;

  out << "\n=== SUMMARY ===\n";
  out << QString("%1 %2 %3 %4\n").arg("Vehicle", -22)
        .arg("IMPORT", -8).arg("DEPLOY", -8).arg("SPAWN");
  for (const ImportRow &r : rows) {
    out << QString("%1 %2 %3 %4\n")
            .arg(r.fixture.name, -22)
            .arg(r.imported ? "OK"   : "FAIL", -8)
            .arg(r.deployed ? "OK"   : "FAIL", -8)
            .arg(r.spawned  ? "OK"   : "FAIL");
  }
  int importPasses = 0, deployPasses = 0;
  for (const ImportRow &r : rows) {
    if (r.imported) ++importPasses;
    if (r.deployed) ++deployPasses;
  }
  out << "\nimport=" << importPasses << "/" << fixtures.size()
      << "  deploy=" << deployPasses << "/" << importPasses
      << "  spawn="  << spawnPasses  << "/" << deployPasses << "\n";

  if (spawnCarla && carlaPid > 0) {
    out << "Killing CARLA pid " << carlaPid << " …\n";
    killProcess(carlaPid);
  }

  return (importPasses == fixtures.size()
          && deployPasses == importPasses
          && spawnPasses  == deployPasses) ? 0 : 1;
}
