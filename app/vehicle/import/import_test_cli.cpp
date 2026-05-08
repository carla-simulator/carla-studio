// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later
//

#include "vehicle/import/carla_spawn.h"
#include "vehicle/import/importer_client.h"
#include "vehicle/import/import_pipeline.h"
#include "vehicle/import/merged_spec_builder.h"
#include "vehicle/import/mesh_analysis.h"
#include "vehicle/import/mesh_geometry.h"
#include "vehicle/import/vehicle_spec.h"

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

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

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

// Tracked PIDs of children we spawn directly. The signal handler reads these
// (atomic load) and SIGKILLs them, plus a pattern sweep, so Ctrl+C / SIGTERM
// of the CLI tears down everything it launched.
std::atomic<qint64> g_editorPid{-1};
std::atomic<qint64> g_carlaPid{-1};

void killAllChildrenAndExit(int sig)
{
  const qint64 ep = g_editorPid.load();
  const qint64 cp = g_carlaPid.load();
  if (ep > 0) ::kill(static_cast<pid_t>(ep), SIGKILL);
  if (cp > 0) ::kill(static_cast<pid_t>(cp), SIGKILL);
  // Sweep cook commandlet, calibration spinner, any forked UE/CARLA.
  ::system("pkill -KILL -f 'UnrealEditor.*CarlaUnreal' 2>/dev/null; "
           "pkill -KILL -f 'CarlaUnreal-Linux-Shipping' 2>/dev/null; "
           "pkill -KILL -f 'carla-studio-vehicle-preview --interactive' 2>/dev/null; true");
  std::_Exit(128 + sig);
}

void installCleanupHandlers()
{
  std::signal(SIGINT,  killAllChildrenAndExit);
  std::signal(SIGTERM, killAllChildrenAndExit);
  std::signal(SIGHUP,  killAllChildrenAndExit);
  std::atexit([]{
    ::system("pkill -KILL -f 'UnrealEditor.*CarlaUnreal' 2>/dev/null; "
             "pkill -KILL -f 'CarlaUnreal-Linux-Shipping' 2>/dev/null; "
             "pkill -KILL -f 'carla-studio-vehicle-preview --interactive' 2>/dev/null; true");
  });
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
  const QString modeFlag = visible ? QString() : QStringLiteral("-nullrhi");
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

struct UserKnobs {
  float mass             = 1500.f;
  float maxSteerAngle    = 70.f;
  float maxBrakeTorque   = 1500.f;
  float suspMaxRaise     = 10.f;
  float suspMaxDrop      = 10.f;
  float suspDamping      = 0.65f;
  cs::SizeClass sizeClass = cs::SizeClass::Sedan;
  QString sizeClassLabel  = "sedan";
};

cs::VehicleSpec makeSpec(const Fixture &f, const UserKnobs &k,
                         const QString &tiresPath = QString())
{
  // Use the SAME spec builder as the GUI's VehicleImportPage so the two
  // paths emit byte-identical specs. Mesh-derived wheel anchors when
  // analysis succeeds; if the body has no detected wheel clusters (common
  // when tires are a separate file), derive anchors from the chassis bbox
  // via mergeBodyAndWheels - so tires don't stack at origin.
  cs::VehicleSpec s;
  cs::MeshGeometry g = cs::loadMeshGeometry(f.meshPath);
  // Detect OBJ units (mm / m / cm) from raw extents - same heuristic
  // VehiclePreviewPage::updateMeshBounds + CalibrationCli use.
  float scaleToCm = 1.0f;
  if (g.valid) {
    float xn=1e30f, xx=-1e30f, yn=1e30f, yx=-1e30f, zn=1e30f, zx=-1e30f;
    for (int i = 0; i < g.vertexCount(); ++i) {
      float x, y, z; g.vertex(i, x, y, z);
      if (x < xn) xn = x; if (x > xx) xx = x;
      if (y < yn) yn = y; if (y > yx) yx = y;
      if (z < zn) zn = z; if (z > zx) zx = z;
    }
    const float maxExt = std::max({xx-xn, yx-yn, zx-zn});
    if      (maxExt >= 200.f && maxExt <= 800.f)        scaleToCm = 1.f;     // cm
    else if (maxExt >= 2.f   && maxExt <= 8.f)          scaleToCm = 100.f;   // meters
    else if (maxExt >= 2000.f && maxExt <= 8000.f)      scaleToCm = 0.1f;    // mm
    else if (maxExt > 0.f && maxExt < 2.f)              scaleToCm = 450.f / std::max(0.01f, maxExt); // squashed → target ~4.5m
  }
  if (g.valid) {
    const cs::MeshAnalysisResult ar = cs::analyzeMesh(g, scaleToCm);
    if (ar.ok) {
      s = cs::buildSpecFromAnalysis(ar, f.name, f.meshPath, scaleToCm, 2, 0);
    }
  }
  if (s.name.isEmpty()) {
    // Mesh-analysis failed entirely - last-ditch sedan fallback.
    s.name     = QString(f.name);
    s.meshPath = QString(f.meshPath);
    for (int i = 0; i < 4; ++i) {
      s.wheels[i].radius = 35.f;
      s.wheels[i].width  = 22.f;
    }
    s.wheels[0].x =  140; s.wheels[0].y = -80; s.wheels[0].z = 35;
    s.wheels[1].x =  140; s.wheels[1].y =  80; s.wheels[1].z = 35;
    s.wheels[2].x = -140; s.wheels[2].y = -80; s.wheels[2].z = 35;
    s.wheels[3].x = -140; s.wheels[3].y =  80; s.wheels[3].z = 35;
  }
  // If wheel anchors are degenerate (all-zero, OR clustered on one axis
  // because analyzeMesh latched onto body detail instead of real wheels,
  // OR detected radius is sub-tire-size), derive them DIRECTLY from the
  // raw OBJ bbox. We can't reuse buildSpecFromAnalysis' chassisXMin/Max
  // here because those are post-swapXY (vehicle-frame), while
  // mergeBodyAndTires expects raw-OBJ-frame anchors. We also force the
  // re-derivation whenever a separate --tires file is provided, since the
  // body's "auto-detected wheels" are unreliable when wheels live in a
  // separate file (no body-side tire geometry exists for analyzeMesh to
  // find).
  bool wheelsDegenerate =
      (s.wheels[0].x == 0 && s.wheels[1].x == 0 &&
       s.wheels[2].x == 0 && s.wheels[3].x == 0)
      || (!tiresPath.isEmpty() && QFileInfo(tiresPath).isFile());
  // Sub-tire radius (< 5 cm at any sensible scale) → analyzeMesh found
  // body fragments, not wheels.
  for (int i = 0; i < 4; ++i)
    if (s.wheels[i].radius * scaleToCm > 0 && s.wheels[i].radius * scaleToCm < 5.f)
      wheelsDegenerate = true;
  if (!wheelsDegenerate && g.valid) {
    // Compute body bbox extents to check that anchors actually span it
    // - same 40% spread threshold mergeBodyAndWheels uses internally.
    float xMin = 1e30f, xMax = -1e30f, yMin = 1e30f, yMax = -1e30f;
    for (int i = 0; i < g.vertexCount(); ++i) {
      float x, y, z; g.vertex(i, x, y, z);
      if (x < xMin) xMin = x; if (x > xMax) xMax = x;
      if (y < yMin) yMin = y; if (y > yMax) yMax = y;
    }
    float wXMin = 1e30f, wXMax = -1e30f, wYMin = 1e30f, wYMax = -1e30f;
    for (int i = 0; i < 4; ++i) {
      if (s.wheels[i].x < wXMin) wXMin = s.wheels[i].x;
      if (s.wheels[i].x > wXMax) wXMax = s.wheels[i].x;
      if (s.wheels[i].y < wYMin) wYMin = s.wheels[i].y;
      if (s.wheels[i].y > wYMax) wYMax = s.wheels[i].y;
    }
    const float bodyExtent  = std::max(xMax - xMin, yMax - yMin);
    const float wheelSpread = std::max(wXMax - wXMin, wYMax - wYMin);
    if (wheelSpread < bodyExtent * 0.40f) wheelsDegenerate = true;
  }
  if (wheelsDegenerate && g.valid) {
    cs::WheelTemplate tpl;
    if (!tiresPath.isEmpty() && QFileInfo(tiresPath).isFile()
        && QFileInfo(tiresPath).suffix().toLower() == "obj") {
      tpl = cs::wheelTemplateFromTiresObj(tiresPath, 1.0f);
    }
    if (!tpl.valid) { tpl.radius = 35.f; tpl.width = 22.f; tpl.valid = true; }

    float xMin = 1e30f, xMax = -1e30f;
    float yMin = 1e30f, yMax = -1e30f;
    float zMin = 1e30f, zMax = -1e30f;
    for (int i = 0; i < g.vertexCount(); ++i) {
      float x, y, z; g.vertex(i, x, y, z);
      if (x < xMin) xMin = x; if (x > xMax) xMax = x;
      if (y < yMin) yMin = y; if (y > yMax) yMax = y;
      if (z < zMin) zMin = z; if (z > zMax) zMax = z;
    }
    // Detect which OBJ axis is forward (longest), up (typically shortest of
    // the remaining two - vehicles are taller than they are wide is rare;
    // height is usually the smallest dim), and lateral (third).
    const float ext[3] = { xMax - xMin, yMax - yMin, zMax - zMin };
    auto longestAxis = [&]() {
      int a = 0;
      for (int i = 1; i < 3; ++i) if (ext[i] > ext[a]) a = i;
      return a;
    }();
    int upAxis;
    {
      int rem[2], k = 0;
      for (int i = 0; i < 3; ++i) if (i != longestAxis) rem[k++] = i;
      upAxis = (ext[rem[0]] <= ext[rem[1]]) ? rem[0] : rem[1];
    }
    const int latAxisIdx = 3 - longestAxis - upAxis;
    const float fwdMinV = (longestAxis == 0) ? xMin : (longestAxis == 1 ? yMin : zMin);
    const float fwdMaxV = (longestAxis == 0) ? xMax : (longestAxis == 1 ? yMax : zMax);
    const float latMinV = (latAxisIdx == 0) ? xMin : (latAxisIdx == 1 ? yMin : zMin);
    const float latMaxV = (latAxisIdx == 0) ? xMax : (latAxisIdx == 1 ? yMax : zMax);
    const float upMinV  = (upAxis == 0) ? xMin : (upAxis == 1 ? yMin : zMin);
    const float fwdInset = (fwdMaxV - fwdMinV) * 0.21f;
    // Lateral inset: real wheels sit just inside the body's outer fender
    // line. Aim for track width ≈ 73% of body width (typical for sedans /
    // SUVs - gives MKZ track 155cm vs body 213cm, GLS track 168cm vs body
    // 207cm). Floor at one tire-half-width so we don't bury the tire mesh.
    const float latExt   = latMaxV - latMinV;
    const float tireHalf = std::max(tpl.width * 0.5f, 1.0f) /
                           std::max(scaleToCm, 1e-3f);
    const float latInset = std::max(latExt * 0.135f, tireHalf);
    const float wheelUp   = upMinV + std::max(tpl.radius / std::max(scaleToCm, 1e-3f), 1.0f / std::max(scaleToCm, 1e-3f));
    const float fwdC = 0.5f * (fwdMinV + fwdMaxV);
    const float latC = 0.5f * (latMinV + latMaxV);
    const float fwdAxisVals[4] = { fwdMaxV - fwdInset, fwdMaxV - fwdInset,
                                   fwdMinV + fwdInset, fwdMinV + fwdInset };
    const float latAxisVals[4] = { latMinV + latInset, latMaxV - latInset,
                                   latMinV + latInset, latMaxV - latInset };
    for (int i = 0; i < 4; ++i) {
      float c[3] = { 0, 0, 0 };
      c[longestAxis] = fwdAxisVals[i];
      c[latAxisIdx]  = latAxisVals[i];
      c[upAxis]      = wheelUp;
      s.wheels[i].x = (c[longestAxis] - fwdC)   * scaleToCm;
      s.wheels[i].y = (c[latAxisIdx]  - latC)   * scaleToCm;
      s.wheels[i].z = (c[upAxis]      - upMinV) * scaleToCm;
      s.wheels[i].radius = tpl.radius;
      s.wheels[i].width  = tpl.width;
    }
  }
  s.contentPath   = "/Game/Carla/Static/Vehicles/4Wheeled";
  s.baseVehicleBP = "/Game/Carla/Blueprints/Vehicles/Mustang/BP_Mustang";
  s.mass          = k.mass;
  s.suspDamping   = k.suspDamping;
  s.sizeClass     = k.sizeClass;
  for (int i = 0; i < 4; ++i) {
    s.wheels[i].maxSteerAngle  = (i < 2) ? k.maxSteerAngle : 0.f;
    s.wheels[i].maxBrakeTorque = k.maxBrakeTorque;
    s.wheels[i].suspMaxRaise   = k.suspMaxRaise;
    s.wheels[i].suspMaxDrop    = k.suspMaxDrop;
  }
  return s;
}

UserKnobs detectKnobs(const QString &meshPath)
{
  UserKnobs k;
  cs::SizeClass cls = cs::classifyByName(QFileInfo(meshPath).completeBaseName());
  if (cls == cs::SizeClass::Unknown) {
    cs::MeshGeometry g = cs::loadMeshGeometry(meshPath);
    if (g.valid) {
      const cs::MeshAnalysisResult ar = cs::analyzeMesh(g, 1.0f);
      if (ar.ok) cls = ar.sizeClass;
    }
  }
  if (cls == cs::SizeClass::Unknown) cls = cs::SizeClass::Sedan;
  const cs::SizePreset p = cs::presetForSizeClass(cls);
  k.mass           = p.mass;
  k.maxSteerAngle  = p.maxSteerAngle;
  k.maxBrakeTorque = p.maxBrakeTorque;
  k.suspMaxRaise   = p.suspMaxRaise;
  k.suspMaxDrop    = p.suspMaxDrop;
  k.suspDamping    = p.suspDamping;
  k.sizeClass      = cls;
  k.sizeClassLabel = cs::sizeClassName(cls);
  return k;
}

float promptFloat(QTextStream &out, const QString &label, float def,
                  const QString &unit)
{
  if (!isatty(fileno(stdin))) return def;
  const QString prompt = QString("  %1 [%2%3]: ")
                            .arg(label)
                            .arg(QString::number(def, 'f', 2))
                            .arg(unit.isEmpty() ? QString() : QString(" %1").arg(unit));
  std::fputs(prompt.toLocal8Bit().constData(), stdout);
  std::fflush(stdout);
  char buf[256] = {0};
  if (!fgets(buf, sizeof(buf), stdin)) return def;
  QString line = QString::fromLocal8Bit(buf).trimmed();
  if (line.isEmpty()) return def;
  bool ok = false;
  float v = line.toFloat(&ok);
  return ok ? v : def;
}

void waitForEnter(QTextStream &out, const QString &msg)
{
  if (!isatty(fileno(stdin))) return;
  out << msg;
  out.flush();
  char buf[8] = {0};
  (void)fgets(buf, sizeof(buf), stdin);
}

QString workspaceRoot()
{
  const QString root = QDir::current().absoluteFilePath(".cs_import");
  QDir().mkpath(root);
  return root;
}

// If --mtl was provided, drop a copy alongside the (possibly centered/merged)
// OBJ so its `mtllib FOO.mtl` line resolves. The OBJ may reference textures
// next to the .mtl (PNG/JPG); those need to live next to the centered copy too,
// so we mirror the .mtl's whole parent dir recursively (small in practice).
void stageMaterialsBeside(const QString &objOnDisk, const QString &mtl);

// Find the OBJ's mtllib reference (e.g. `mtllib MKZ.mtl`) and stage that
// .mtl + sibling textures next to the target OBJ. Without this, merged /
// centered copies of an OBJ that lives in /home/.../mkz/ have a dangling
// mtllib pointer and Qt3D's QSceneLoader falls back to flat shading.
void autoStageBodyMaterials(const QString &bodyObj, const QString &targetObj) {
  if (bodyObj.isEmpty() || targetObj.isEmpty()) return;
  if (!QFileInfo(bodyObj).isFile() || !QFileInfo(targetObj).isFile()) return;
  QFile in(bodyObj);
  if (!in.open(QIODevice::ReadOnly | QIODevice::Text)) return;
  QTextStream ts(&in);
  QString mtlName;
  int linesScanned = 0;
  while (!ts.atEnd() && linesScanned < 400) {
    const QString line = ts.readLine();
    ++linesScanned;
    if (line.startsWith("mtllib ") || line.startsWith("mtllib\t")) {
      mtlName = line.section(QRegularExpression("\\s+"), 1).trimmed();
      break;
    }
    if (line.startsWith("v ") || line.startsWith("v\t")) break;
  }
  if (mtlName.isEmpty()) return;
  // Resolve through symlinks - the CLI's "fixture_*" symlink dir doesn't
  // contain the .mtl, but the original source dir does. Without canonical
  // path, the mtllib lookup lands in the symlink dir and bails.
  const QString sourceObj = QFileInfo(bodyObj).canonicalFilePath();
  const QString sourceDir = sourceObj.isEmpty()
      ? QFileInfo(bodyObj).absolutePath()
      : QFileInfo(sourceObj).absolutePath();
  const QString mtlPath = QDir(sourceDir).absoluteFilePath(mtlName);
  if (!QFileInfo(mtlPath).isFile()) return;
  stageMaterialsBeside(targetObj, mtlPath);
}

void stageMaterialsBeside(const QString &objOnDisk, const QString &mtl) {
  if (mtl.isEmpty() || !QFileInfo(mtl).isFile()) return;
  const QString objDir = QFileInfo(objOnDisk).absolutePath();
  const QString mtlDir = QFileInfo(mtl).absolutePath();
  if (QFileInfo(objDir).canonicalFilePath()
      == QFileInfo(mtlDir).canonicalFilePath()) return;  // already next to obj
  // Mirror everything in the mtl's directory (mtl + textures) into objDir.
  // Skip files that would clobber the OBJ itself.
  const QString objBase = QFileInfo(objOnDisk).fileName();
  for (const QFileInfo &fi : QDir(mtlDir).entryInfoList(
           QDir::Files | QDir::NoSymLinks | QDir::NoDotAndDotDot)) {
    if (fi.fileName() == objBase) continue;
    const QString dst = QDir(objDir).absoluteFilePath(fi.fileName());
    QFile::remove(dst);
    QFile::copy(fi.absoluteFilePath(), dst);
  }
}

// Produce a centered copy of an OBJ (XY bbox centered on origin, Z=0 at the
// bottom of the body) so the Qt3D calibration view shows the vehicle on top
// of the grid origin without the user having to click "Recenter". For other
// mesh types (.fbx/.glb/.gltf/.dae/.blend) we leave them as-is.
QString centerObjOnDisk(const QString &srcPath, const QString &outDir)
{
  if (QFileInfo(srcPath).suffix().toLower() != "obj") return srcPath;
  QFile in(srcPath);
  if (!in.open(QIODevice::ReadOnly | QIODevice::Text)) return srcPath;
  QStringList lines;
  double xmin =  1e300, xmax = -1e300;
  double ymin =  1e300, ymax = -1e300;
  double zmin =  1e300, zmax = -1e300;
  {
    QTextStream ts(&in);
    while (!ts.atEnd()) {
      const QString line = ts.readLine();
      lines.append(line);
      if (line.startsWith("v ") || line.startsWith("v\t")) {
        const QStringList toks = line.split(QRegularExpression("\\s+"),
                                            Qt::SkipEmptyParts);
        if (toks.size() >= 4) {
          bool ox=false, oy=false, oz=false;
          const double x = toks[1].toDouble(&ox);
          const double y = toks[2].toDouble(&oy);
          const double z = toks[3].toDouble(&oz);
          if (ox && oy && oz) {
            if (x < xmin) xmin = x; if (x > xmax) xmax = x;
            if (y < ymin) ymin = y; if (y > ymax) ymax = y;
            if (z < zmin) zmin = z; if (z > zmax) zmax = z;
          }
        }
      }
    }
  }
  if (xmin > xmax) return srcPath;  // no vertices found

  // Detect axes: longest = forward, shortest = up (vehicles are longer
  // than wide, taller than narrow rarely; height is usually shortest).
  // Then transform every "v" line so the output is canonical:
  //   - bbox centered on origin in lateral + forward
  //   - rests on ground (up_min becomes z=0)
  //   - up axis remapped to Z (Qt3D world's up direction)
  //   - scaled to cm so the calibration view's auto-scale picks "cm" mode
  //     and the camera frames the vehicle correctly.
  const double extX = xmax - xmin;
  const double extY = ymax - ymin;
  const double extZ = zmax - zmin;
  const double maxExt = std::max({extX, extY, extZ});

  enum Axis { AX_X, AX_Y, AX_Z };
  Axis fwdAxis = AX_X, upAxis = AX_Z, latAxis = AX_Y;
  // forward = longest extent
  if      (extX >= extY && extX >= extZ) fwdAxis = AX_X;
  else if (extY >= extX && extY >= extZ) fwdAxis = AX_Y;
  else                                    fwdAxis = AX_Z;
  // up = shortest extent (typical vehicle height)
  if      (extZ <= extX && extZ <= extY) upAxis = AX_Z;
  else if (extY <= extX && extY <= extZ) upAxis = AX_Y;
  else                                    upAxis = AX_X;
  if (upAxis == fwdAxis) {
    // Edge case: forward and up tied. Pick remaining axis as up.
    upAxis = (fwdAxis == AX_X) ? AX_Y : (fwdAxis == AX_Y ? AX_Z : AX_X);
  }
  latAxis = static_cast<Axis>(3 - fwdAxis - upAxis);

  // Per-axis bbox centers/floors for translation BEFORE remapping to Z-up.
  auto axMin = [&](Axis a) { return a == AX_X ? xmin : a == AX_Y ? ymin : zmin; };
  auto axMax = [&](Axis a) { return a == AX_X ? xmax : a == AX_Y ? ymax : zmax; };
  const double fwdMin  = axMin(fwdAxis), fwdMax = axMax(fwdAxis);
  const double latMin  = axMin(latAxis), latMax = axMax(latAxis);
  const double upMin   = axMin(upAxis);
  const double fwdC    = 0.5 * (fwdMin + fwdMax);
  const double latC    = 0.5 * (latMin + latMax);
  // Detect units: aim for ~5 m typical vehicle. If maxExt is in meters
  // (~1-10), scale ×100 to cm. If already in cm (200-800), ×1. If mm
  // (2000-8000), ×0.1. Else fall back to "fit ~450 cm".
  double unitScale = 1.0;
  if      (maxExt >= 200.0 && maxExt <= 800.0) unitScale = 1.0;
  else if (maxExt >= 2.0   && maxExt <= 8.0)   unitScale = 100.0;
  else if (maxExt >= 2000.0 && maxExt <= 8000.0) unitScale = 0.1;
  else if (maxExt > 0.0 && maxExt < 2.0)       unitScale = 450.0 / maxExt;

  QDir().mkpath(outDir);
  const QString outPath = QDir(outDir).filePath(
      QFileInfo(srcPath).completeBaseName() + "_centered.obj");
  QFile out(outPath);
  if (!out.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    return srcPath;
  QTextStream os(&out);
  // Per-vertex transform:
  //   1. Translate so detected (lat, fwd, up) → (0, 0, up_min) inside the
  //      raw-OBJ frame.
  //   2. Remap raw axes (lat, fwd, up) to canonical (X, Y, Z) - Y forward,
  //      X lateral (-X = left), Z up - so the Qt3D viewer (Z-up) shows the
  //      vehicle right-side-up regardless of source convention.
  //   3. Multiply by unitScale to lift sub-meter OBJs into the cm range
  //      that the calibration view's auto-scaler expects.
  auto pickFromAxis = [](Axis a, double x, double y, double z) {
    return a == AX_X ? x : (a == AX_Y ? y : z);
  };
  for (const QString &line : lines) {
    if (line.startsWith("v ") || line.startsWith("v\t")) {
      const QStringList toks = line.split(QRegularExpression("\\s+"),
                                          Qt::SkipEmptyParts);
      if (toks.size() >= 4) {
        bool ox=false, oy=false, oz=false;
        const double x = toks[1].toDouble(&ox);
        const double y = toks[2].toDouble(&oy);
        const double z = toks[3].toDouble(&oz);
        if (ox && oy && oz) {
          // Step 1 - pull raw (lat, fwd, up) values.
          const double rawLat = pickFromAxis(latAxis, x, y, z);
          const double rawFwd = pickFromAxis(fwdAxis, x, y, z);
          const double rawUp  = pickFromAxis(upAxis,  x, y, z);
          // Step 2 - translate (center lateral & forward, ground the up).
          const double centeredLat = rawLat - latC;
          const double centeredFwd = rawFwd - fwdC;
          const double centeredUp  = rawUp  - upMin;
          // Step 3 - remap to canonical axes (X = lateral, Y = forward,
          //          Z = up) and scale to cm.
          const double newX = centeredLat * unitScale;
          const double newY = centeredFwd * unitScale;
          const double newZ = centeredUp  * unitScale;
          os << "v " << QString::number(newX, 'f', 6)
             << " "  << QString::number(newY, 'f', 6)
             << " "  << QString::number(newZ, 'f', 6);
          for (int i = 4; i < toks.size(); ++i) os << " " << toks[i];
          os << "\n";
          continue;
        }
      }
    }
    os << line << "\n";
  }
  return outPath;
}

void openCalibrationViews(QTextStream &out, const QString &meshPath,
                          const cs::VehicleSpec *spec = nullptr)
{
  const QString previewBin = QCoreApplication::applicationDirPath()
                           + "/carla-studio-vehicle-preview";
  if (!QFileInfo(previewBin).isExecutable()) {
    out << "  (calibration preview binary not found at " << previewBin << ")\n";
    return;
  }
  const bool hasDisplay = !qEnvironmentVariable("DISPLAY").isEmpty()
                       || !qEnvironmentVariable("WAYLAND_DISPLAY").isEmpty();
  if (hasDisplay) {
    out << "  opening Qt3D calibration viewer - orbit/drag to inspect, close window to continue…\n";
    out.flush();
    if (spec) {
      // Pass spec wheel anchors (cm) so the calibration window's wheel
      // markers land on the actual import positions, not the off-screen
      // default. Helps the user verify wheelbase + track at a glance.
      std::array<float, 12> w{};
      for (int i = 0; i < 4; ++i) {
        w[i*3 + 0] = spec->wheels[i].x;
        w[i*3 + 1] = spec->wheels[i].y;
        w[i*3 + 2] = spec->wheels[i].z;
      }
      cs::openCalibrationWindowWithWheels(meshPath, w);
    } else {
      cs::openCalibrationWindow(meshPath);
    }
    return;
  }
  const QString stem = QFileInfo(meshPath).completeBaseName();
  const QString outDir = workspaceRoot() + "/preview_" + stem;
  QDir(outDir).removeRecursively();
  QProcess preview;
  preview.start(previewBin, QStringList() << meshPath << outDir);
  preview.waitForFinished(45000);
  out << "  no display - wrote calibration PNGs (manual viewer):\n"
      << "    " << outDir << "/top.png\n"
      << "    " << outDir << "/side.png\n"
      << "    " << outDir << "/front.png\n";
}

void promptKnobs(QTextStream &out, UserKnobs &k, const QString &meshPath,
                 const cs::VehicleSpec *spec = nullptr)
{
  out << "\n=== CALIBRATION ===\n";
  out << "  mesh        : " << meshPath << "\n";
  out << "  size class  : " << k.sizeClassLabel
      << " (filename + chassis-length heuristic)\n";
  openCalibrationViews(out, meshPath, spec);
  if (!isatty(fileno(stdin))) {
    out << "  (stdin not a TTY - using class defaults; skipping prompts)\n";
    return;
  }
  out << "  calibration window closed. Now let's build Physics\n"
         "  Press ENTER to keep the prefilled value, or type a new number.\n"
         "  fields with * are critical for you to review as these might have "
         "driveability impact. Know that if the vehicle doesn't drive on road, "
         "you will have to re-do this step.\n\n";
  out.flush();
  k.mass           = promptFloat(out, "Vehicle Mass             *", k.mass,           "kg");
  k.maxSteerAngle  = promptFloat(out, "Max Steer Angle           ", k.maxSteerAngle,  "deg");
  k.maxBrakeTorque = promptFloat(out, "Max Brake Torque         *", k.maxBrakeTorque, "Nm");
  k.suspMaxRaise   = promptFloat(out, "Suspension Max Raise     *", k.suspMaxRaise,   "cm");
  k.suspMaxDrop    = promptFloat(out, "Suspension Max Drop      *", k.suspMaxDrop,    "cm");
  k.suspDamping    = promptFloat(out, "Suspension Damping       *", k.suspDamping,    "");
  out << "  using mass=" << k.mass << "kg  brake=" << k.maxBrakeTorque
      << "Nm  susp=(" << k.suspMaxRaise << "/" << k.suspMaxDrop << "cm @ "
      << k.suspDamping << ")  steer=" << k.maxSteerAngle << "deg\n";
  out.flush();
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

int runVehicleImportCli(int argc, char **argv)
{
  QCoreApplication app(argc, argv);
  installCleanupHandlers();
  int  port         = 18584;
  int  carlaPort    = 3000;
  bool spawnEditor  = true;
  bool fullDrive    = false;
  bool spawnCarla   = true;
  bool visibleCarla = true;
  bool screenshots  = false;
  bool spawnOnly    = false;  // skip import+deploy, jump straight to CARLA spawn+drive
  bool manualDrive  = false;  // --manual_drive: interactive matrix; default = SAE L5 autopilot for 60s then exit
  int  autoDriveSeconds = 60;
  QString tiresPath;          // --tires: optional separate tire/wheel mesh, merged into body before import
  QString mtlPath;            // --mtl: optional Wavefront .mtl materials file (resolved alongside the OBJ)
  QString screenshotsDir = workspaceRoot() + "/shots";
  QString compareWith;  // e.g. "vehicle.taxi.ford" - spawn this 5 m to the right for visual ref
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
    else if (a == "--manual_drive")                     manualDrive = true;
    else if (a == "--manual-drive")                     manualDrive = true;
    else if (a == "--auto-seconds"     && i + 1 < argc) autoDriveSeconds = QString(argv[++i]).toInt();
    else if (a == "--tires"            && i + 1 < argc) tiresPath = QString(argv[++i]);
    else if (a == "--mtl"              && i + 1 < argc) mtlPath   = QString(argv[++i]);
    else if (a == "--debug-merge-only") {
      // Diagnostic mode: run only the calibration-side pre-merge + spec build
      // for the first --mesh, write the merged.obj to .cs_import/, print the
      // 4 derived wheel anchors, and exit. No editor, no CARLA, no prompts.
      qputenv("CARLA_VEHICLE_TEST_FIXTURES_DIR_DEBUG_MERGE_ONLY", "1");
    }
    else if (a == "--mesh"             && i + 1 < argc) {
      const QString meshPath = QString(argv[++i]);
      if (!QFileInfo::exists(meshPath)) {
        QTextStream(stderr) << "--mesh: file not found: " << meshPath << "\n";
        return 8;
      }
      QFileInfo fi(meshPath);
      const QString supported[] = { "obj","fbx","glb","gltf","dae","blend" };
      const QString suffix = fi.suffix().toLower();
      bool ok = false;
      for (const auto &e : supported) if (suffix == e) { ok = true; break; }
      if (!ok) {
        QTextStream(stderr) << "--mesh: unsupported extension ." << suffix
                            << " (supported: obj/fbx/glb/gltf/dae/blend)\n";
        return 8;
      }
      const QString stem = fi.completeBaseName().toLower()
                             .replace(QRegularExpression("[^a-z0-9_]+"), "_");
      const QString tmpDir = workspaceRoot() + "/fixture_" + stem;
      QDir(tmpDir).removeRecursively();
      QDir().mkpath(tmpDir);
      const QString link = tmpDir + "/" + stem + "." + suffix;
      QFile::link(fi.absoluteFilePath(), link);
      qputenv("CARLA_VEHICLE_TEST_FIXTURES_DIR", tmpDir.toLocal8Bit());
      fullDrive = true;
    }
  }
  if (screenshots) visibleCarla = true;  // screenshots imply visible carla
  qputenv("CARLA_VEHICLE_IMPORTER_PORT", QByteArray::number(port));

  QTextStream out(stdout);

  if (qEnvironmentVariableIsSet("CARLA_VEHICLE_TEST_FIXTURES_DIR_DEBUG_MERGE_ONLY")) {
    const QList<Fixture> fxts = loadFixtures();
    if (fxts.isEmpty()) {
      out << "--debug-merge-only: no fixture loaded (set --mesh first)\n";
      return 11;
    }
    const Fixture &f = fxts.first();
    UserKnobs knobs = detectKnobs(f.meshPath);
    cs::VehicleSpec s = makeSpec(f, knobs, tiresPath);
    out << "[debug-merge-only] derived wheel anchors (cm) for " << f.name << ":\n";
    for (int i = 0; i < 4; ++i) {
      out << QString("  wheel[%1] x=%2 y=%3 z=%4 r=%5 w=%6\n")
              .arg(i).arg(s.wheels[i].x, 0, 'f', 1).arg(s.wheels[i].y, 0, 'f', 1)
              .arg(s.wheels[i].z, 0, 'f', 1)
              .arg(s.wheels[i].radius, 0, 'f', 1).arg(s.wheels[i].width, 0, 'f', 1);
    }
    if (!tiresPath.isEmpty()
        && QFileInfo(f.meshPath).suffix().toLower() == "obj"
        && QFileInfo(tiresPath).isFile()) {
      const QString outDir = workspaceRoot() + "/merged_" + f.name;
      QDir(outDir).removeRecursively();
      QDir().mkpath(outDir);
      const cs::ObjMergeResult mr = cs::mergeBodyAndTires(
          f.meshPath, tiresPath, outDir, s.wheels);
      out << "[debug-merge-only] merge: " << (mr.ok ? "OK" : "FAIL")
          << " - " << mr.reason << "\n";
      if (mr.ok) {
        out << "[debug-merge-only] output: " << mr.outputPath << "\n";
        autoStageBodyMaterials(f.meshPath, mr.outputPath);
        const QString centered = centerObjOnDisk(
            mr.outputPath, workspaceRoot() + "/centered_" + f.name + "_merged");
        if (centered != mr.outputPath) {
          autoStageBodyMaterials(f.meshPath, centered);
          out << "[debug-merge-only] centered output: " << centered << "\n";
        }
      }
    } else if (QFileInfo(f.meshPath).suffix().toLower() == "obj") {
      // Body-only path - same on-disk centering as the live import flow.
      const QString centered = centerObjOnDisk(
          f.meshPath, workspaceRoot() + "/centered_" + f.name);
      if (centered != f.meshPath) {
        autoStageBodyMaterials(f.meshPath, centered);
        out << "[debug-merge-only] centered output: " << centered << "\n";
      }
    }
    return 0;
  }

  if (portOpen(port) || portOpen(carlaPort)) {
    out << "[cleanup] reclaiming ports - killing leftover UnrealEditor / "
           "CarlaUnreal-Linux-Shipping from a prior run …\n";
    QProcess::execute("/bin/sh", QStringList() << "-c"
        << "pkill -KILL -f 'UnrealEditor.*CarlaUnreal' 2>/dev/null; "
           "pkill -KILL -f 'CarlaUnreal-Linux-Shipping' 2>/dev/null; "
           "pkill -KILL -f 'carla-studio-vehicle-preview --interactive' 2>/dev/null; true");
    QThread::sleep(3);
  }

  if (visibleCarla) {
    const QString display = qEnvironmentVariable("DISPLAY");
    const QString wayland = qEnvironmentVariable("WAYLAND_DISPLAY");
    const QString sshConn = qEnvironmentVariable("SSH_CONNECTION");
    const bool noDisplay  = display.isEmpty() && wayland.isEmpty();
    const bool sshNoX     = !sshConn.isEmpty() && display.isEmpty();
    if (noDisplay || sshNoX) {
      if (noDisplay) {
        out << "\n[!]  No display detected (DISPLAY and WAYLAND_DISPLAY are unset).\n";
      } else {
        out << "\n[!]  Detected SSH session (SSH_CONNECTION=" << sshConn
            << ") without forwarded display.\n";
      }
      out << "     Calibration image viewers and the visible CARLA window need\n"
             "     a graphical session. This is common on Docker / remote servers.\n\n"
             "     Proceed in HEADLESS mode (no calibration viewers, no visible\n"
             "     CARLA - runs import + cook + zip, prints the .tar.gz path)?\n"
             "     [Y]es / [n]o: ";
      out.flush();
      QString ans;
      if (isatty(fileno(stdin))) {
        char buf[8] = {0};
        if (fgets(buf, sizeof(buf), stdin))
          ans = QString::fromLocal8Bit(buf).trimmed().toLower();
      } else {
        ans = "y";
        out << "(stdin not a TTY - assuming yes)\n";
      }
      if (ans == "n" || ans == "no") {
        out << "Aborted by user.\n";
        return 9;
      }
      visibleCarla = false;
      out << "==> Headless mode: skipping calibration viewers and visible CARLA.\n\n";
      out.flush();
    }
  }

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
  const QString editorLog = workspaceRoot() + QString("/editor_%1.log").arg(port);
  if (spawnEditor) {
    if (portOpen(port)) {
      out << "Port " << port << " already in use - pass --no-spawn-editor or pick another with --port.\n";
      return 2;
    }
    QFile::remove(editorLog);
    out << "Launching headless editor on port " << port << " (stdout→" << editorLog << ") …\n";
    editorPid = launchEditor(port, editorLog);
    g_editorPid.store(editorPid);
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
        out << QString("[skip] %1 - mesh missing at %2\n").arg(f.name, f.meshPath);
        rows.append(r);
        continue;
      }
      UserKnobs knobs = detectKnobs(f.meshPath);
      // Pre-merge body+tires (if --tires given) BEFORE calibration so the Qt3D
      // viewer + the editor see the assembled vehicle, not the chassis alone.
      QString viewMeshPath = f.meshPath;
      QString mergedPath;
      if (!tiresPath.isEmpty()
          && QFileInfo(f.meshPath).suffix().toLower() == "obj"
          && QFileInfo(tiresPath).suffix().toLower() == "obj"
          && QFileInfo(tiresPath).isFile()) {
        const cs::VehicleSpec preSpec = makeSpec(f, knobs, tiresPath);
        const QString outDir = workspaceRoot() + "/merged_" + f.name;
        QDir(outDir).removeRecursively();
        QDir().mkpath(outDir);
        const cs::ObjMergeResult mr = cs::mergeBodyAndTires(
            f.meshPath, tiresPath, outDir, preSpec.wheels);
        if (mr.ok) {
          mergedPath = mr.outputPath;
          viewMeshPath = mergedPath;
          autoStageBodyMaterials(f.meshPath, mergedPath);
          // Remap axes to canonical Z-up (some kits use Y-up - RR's source
          // has X=fwd, Y=up, Z=lat) and ground-rest the result. Without
          // this the calibration viewer renders bodies on their side.
          const QString centered = centerObjOnDisk(
              mergedPath, workspaceRoot() + "/centered_" + f.name + "_merged");
          if (centered != mergedPath) {
            autoStageBodyMaterials(f.meshPath, centered);
            viewMeshPath = centered;
          }
          out << QString("  merged body+tires (%1 verts body + %2 verts tires "
                         "→ %3 verts; %4/%5 tire copies placed) → %6\n")
                  .arg(mr.bodyVertexCount).arg(mr.tireVertexCount)
                  .arg(mr.mergedVertexCount).arg(mr.tireCopiesPlaced)
                  .arg(mr.tireSourceCount).arg(mr.outputPath);
        } else {
          out << QString("  body+tires merge failed: %1 - calibration will show "
                         "chassis only.\n").arg(mr.reason);
        }
      } else if (!tiresPath.isEmpty()) {
        out << "  --tires given but path is not a readable .obj - ignoring.\n";
      }
      // No tires merge → still produce a centered copy of the body so the
      // calibration view shows the vehicle ON the grid origin, not floating
      // off-center because the source OBJ has its origin at one corner.
      if (mergedPath.isEmpty()
          && QFileInfo(f.meshPath).suffix().toLower() == "obj") {
        const QString centered = centerObjOnDisk(
            f.meshPath, workspaceRoot() + "/centered_" + f.name);
        if (centered != f.meshPath) {
          autoStageBodyMaterials(f.meshPath, centered);
          viewMeshPath = centered;
          out << "  centered body OBJ for calibration view → " << centered << "\n";
        }
      }
      if (!mtlPath.isEmpty()) {
        if (!QFileInfo(mtlPath).isFile()) {
          out << "  --mtl path not a file - ignoring: " << mtlPath << "\n";
        } else {
          stageMaterialsBeside(viewMeshPath, mtlPath);
          out << "  materials staged: " << QFileInfo(mtlPath).fileName()
              << " + sibling textures copied next to "
              << QFileInfo(viewMeshPath).fileName() << "\n"
              << "  (calibration view is flat-shaded; the editor's Interchange "
                 "importer will pick up materials at cook time.)\n";
        }
      }
      // Build spec FIRST so the calibration window can render real wheel
      // markers at the actual import positions instead of leaving them at
      // the off-screen default.
      cs::VehicleSpec s = makeSpec(f, knobs, tiresPath);
      promptKnobs(out, knobs, viewMeshPath, &s);
      if (!mergedPath.isEmpty()) s.meshPath = mergedPath;
      QJsonObject json = cs::specToJson(s);
      out << "\n=== IMPORT (5-stage UE pipeline; 30-90 s blocking) ===\n"
             "  → sending spec to editor on port " << port << "\n"
             "    [stage 1/5] preflight (canonicalize OBJ axes & scale)\n"
             "    [stage 2/5] interchange (UE5 mesh import)\n"
             "    [stage 3/5] build SK (skeletal mesh from template)\n"
             "    [stage 4/5] build BP (vehicle blueprint + 4 wheel BPs)\n"
             "    [stage 5/5] persist (SaveAsset to disk)\n"
             "  detailed log: " << editorLog
          << " (tail -f to watch progress live)\n";
      out.flush();
      QString resp = cs::sendJson(json);
      if (resp.isEmpty()) {
        r.importDetail = "no response from editor";
        out << QString("[FAIL] %1 - no response from editor\n").arg(f.name);
        rows.append(r);
        continue;
      }
      const QJsonObject obj = QJsonDocument::fromJson(resp.toUtf8()).object();
      r.imported     = obj.value("success").toBool();
      r.assetPath    = obj.value("asset_path").toString();
      r.importDetail = r.imported ? r.assetPath : obj.value("error").toString();
      out << QString(r.imported ? "[ OK ] %1 → %2\n" : "[FAIL] %1 - %2\n")
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
      out << "  register: " << (rr.ok ? "OK" : "FAIL") << " - " << rr.detail << "\n";
      if (!rr.ok) { r.deployDetail = rr.detail; continue; }

      out << "[" << r.fixture.name << "] deploying (UE cook commandlet - 2-3 min first cook, "
                                       "<30 s on subsequent cooks; "
                                       "watch " + srcRoot + "/Unreal/CarlaUnreal/"
                                       "Saved/Logs/ for live cook output) …\n";
      out.flush();
      const cs::DeployResult dr = cs::deployVehicleToShippingCarla(reg);
      r.deployed     = dr.ok;
      r.filesCopied  = dr.filesCopied;
      r.cookedFiles  = dr.cookedFiles;
      r.deployDetail = dr.detail;
      out << "  deploy: " << (dr.ok ? "OK" : "FAIL")
          << " - cooked=" << dr.cookedFiles
          << " copied=" << dr.filesCopied
          << " - " << dr.detail << "\n";
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
  const QString carlaLog = workspaceRoot() + QString("/carla_%1.log").arg(carlaPort);
  if (spawnCarla) {
    if (portOpen(carlaPort)) {
      out << "[cleanup] RPC port " << carlaPort
          << " busy - sweeping leftover CarlaUnreal-Linux-Shipping…\n";
      QProcess::execute("/bin/sh", QStringList() << "-c"
          << "pkill -KILL -f 'CarlaUnreal-Linux-Shipping' 2>/dev/null; "
             "fuser -k " + QString::number(carlaPort) + "/tcp 2>/dev/null; true");
      QThread::sleep(3);
      if (portOpen(carlaPort)) {
        out << "RPC port " << carlaPort
            << " still busy after sweep - pass --no-spawn-carla or pick --carla-port.\n";
        return 8;
      }
      out << "[cleanup] port " << carlaPort << " freed; continuing.\n";
    }
    const QString script = findShippingScript(shippingRoot);
    if (script.isEmpty()) {
      out << "Shipping CARLA launcher not found in " << shippingRoot
          << " (need CarlaUnreal.sh / CarlaUE5.sh / CarlaUE4.sh).\n";
      return 9;
    }
    QFile::remove(carlaLog);
    out << "\n=== LAUNCH SHIPPING CARLA "
        << (visibleCarla ? "(visible RHI window)" : "(headless / -nullrhi)") << " ===\n";
    out << "script: " << script << " on RPC port " << carlaPort
        << " (stdout→" << carlaLog << ") …\n";
    out.flush();
    carlaPid = launchShippingCarla(script, carlaPort, carlaLog, visibleCarla);
    g_carlaPid.store(carlaPid);
    if (carlaPid <= 0) {
      out << "CARLA launch failed.\n";
      return 10;
    }
    out << "CARLA pid: " << carlaPid << " - Unreal is loading content "
        << "(window appears in 30-60 s once shaders/Maps finish; until then it "
        << "looks like nothing happens). Polling RPC port " << carlaPort
        << " every 1 s, up to 120 s …\n";
    out.flush();
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
    out << QString(r.spawned ? "[ OK ] %1 - %2\n" : "[FAIL] %1 - %2\n")
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
          QThread::sleep(2);
          if (auto v = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
            carla::rpc::VehicleControl warm; warm.throttle = 0.3f;
            v->ApplyControl(warm);
            QThread::msleep(800);
            carla::rpc::VehicleControl brk; brk.brake = 1.f; brk.throttle = 0.f;
            v->ApplyControl(brk);
            QThread::sleep(1);
          }
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
                  out << QString("  ref: TrySpawnActor returned null for %1 - point may "
                                 "be blocked or BP not in library\n").arg(compareWith);
                }
              } else {
                out << QString("  ref: blueprint '%1' not in library - skip\n").arg(compareWith);
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

          if (!manualDrive) {
            out << QString("  SAE L5 autopilot (Traffic Manager) - driving for %1 s …\n")
                    .arg(autoDriveSeconds);
            out.flush();
            if (auto v2 = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
              v2->SetAutopilot(true);
            }
            const auto p0 = ours->GetTransform().location;
            const int  ticks = autoDriveSeconds;
            for (int s = 1; s <= ticks; ++s) {
              QThread::sleep(1);
              try {
                const auto p = ours->GetTransform().location;
                const auto t = ours->GetTransform();
                const double yr = double(t.rotation.yaw) * (M_PI / 180.0);
                carla::geom::Transform sp2;
                sp2.location.x = p.x - 8.f * float(std::cos(yr));
                sp2.location.y = p.y - 8.f * float(std::sin(yr));
                sp2.location.z = p.z + 4.f;
                sp2.rotation.pitch = -15.f; sp2.rotation.yaw = t.rotation.yaw;
                spectator->SetTransform(sp2);
                if (s % 5 == 0) {
                  float vmag = 0.f;
                  if (auto v2 = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
                    auto vel = v2->GetVelocity();
                    vmag = float(std::sqrt(double(
                        vel.x*vel.x + vel.y*vel.y + vel.z*vel.z)));
                  }
                  const float dist = float(std::sqrt(double(
                      (p.x-p0.x)*(p.x-p0.x) + (p.y-p0.y)*(p.y-p0.y))));
                  out << QString("    t=%1s  d=%2m  v=%3m/s\n")
                          .arg(s).arg(dist,0,'f',1).arg(vmag,0,'f',2);
                  out.flush();
                }
              } catch (...) {}
            }
            const auto pe = ours->GetTransform().location;
            const float total = float(std::sqrt(double(
                (pe.x-p0.x)*(pe.x-p0.x) + (pe.y-p0.y)*(pe.y-p0.y))));
            out << QString("  autopilot total displacement=%1 m over %2 s\n")
                    .arg(total,0,'f',1).arg(autoDriveSeconds);
            r.spawnDetail += QString(" | autopilot d=%1m/%2s")
                                .arg(total,0,'f',1).arg(autoDriveSeconds);
            if (auto v2 = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
              v2->SetAutopilot(false);
              carla::rpc::VehicleControl brk; brk.brake = 1.f; brk.throttle = 0.f;
              v2->ApplyControl(brk);
            }
          } else {
            if (auto v2 = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
              v2->SetAutopilot(false);
              try {
                auto pc = v2->GetPhysicsControl();
                out << QString("  diag-PC: max_torque=%1 max_rpm=%2 final_ratio=%3 "
                              "fwd_gears=%4 rev_gears=%5 mass=%6\n")
                        .arg(pc.max_torque, 0,'f',1)
                        .arg(pc.max_rpm,    0,'f',0)
                        .arg(pc.final_ratio,0,'f',2)
                        .arg(int(pc.forward_gear_ratios.size()))
                        .arg(int(pc.reverse_gear_ratios.size()))
                        .arg(pc.mass,       0,'f',0);
                pc.max_torque = 700.f;
                pc.max_rpm    = 6000.f;
                pc.idle_rpm   = 1200.f;
                pc.final_ratio = 3.5f;
                pc.use_automatic_gears = true;
                pc.forward_gear_ratios = { 4.0f, 2.5f, 1.6f, 1.2f, 1.0f };
                pc.reverse_gear_ratios = { 4.0f };
                pc.change_up_rpm   = 4500.f;
                pc.change_down_rpm = 2000.f;
                pc.differential_type = 0;
                pc.front_rear_split  = 0.5f;
                pc.transmission_efficiency = 0.9f;
                v2->ApplyPhysicsControl(pc);
                out << QString("  diag-PC: applied forced physics control\n");
              } catch (const std::exception &e) {
                out << QString("  diag-PC: %1\n").arg(QString::fromUtf8(e.what()));
              } catch (...) {
                out << "  diag-PC: unknown throw\n";
              }
              QThread::msleep(500);
              carla::rpc::VehicleControl ctl;
              ctl.throttle = 1.0f; ctl.steer = 0.f; ctl.brake = 0.f; ctl.reverse = false;
              ctl.manual_gear_shift = true;
              ctl.gear = 1;
              v2->ApplyControl(ctl);
            }
            auto pa = ours->GetTransform().location;
            out << QString("  diag: pure-throttle hold (no teleport, no autopilot)\n");
            for (int s = 1; s <= 8; ++s) {
              QThread::sleep(1);
              auto p = ours->GetTransform().location;
              float vmag = 0.f;
              if (auto v2 = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
                auto vel = v2->GetVelocity();
                vmag = float(std::sqrt(double(vel.x*vel.x + vel.y*vel.y + vel.z*vel.z)));
              }
              const float dist = float(std::sqrt(double(
                  (p.x-pa.x)*(p.x-pa.x) + (p.y-pa.y)*(p.y-pa.y))));
              out << QString("    t=%1s pos=(%2,%3,%4) d=%5m v=%6m/s\n")
                      .arg(s).arg(p.x,0,'f',2).arg(p.y,0,'f',2).arg(p.z,0,'f',2)
                      .arg(dist,0,'f',2).arg(vmag,0,'f',2);
              out.flush();
            }
          }
          if (manualDrive) {
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
          }
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
    if (manualDrive) {
      waitForEnter(out, "\nClose the CARLA window (or press ENTER here) "
                        "to stop the simulator and zip the cooked vehicle…\n");
    } else {
      out << "\nAutopilot run finished - closing CARLA and zipping…\n";
    }
    out << "Killing CARLA pid " << carlaPid << " …\n";
    killProcess(carlaPid);
    QThread::sleep(2);
  }

  for (const ImportRow &r : rows) {
    if (!r.deployed) continue;
    const QString cookedSrc = QFileInfo(uproject).absolutePath() + "/Saved/Cooked/Linux/"
                              "CarlaUnreal/Content/Carla/Static/Vehicles/4Wheeled/"
                            + r.fixture.name;
    if (!QFileInfo(cookedSrc).isDir()) continue;
    const QString tarPath = QDir::current().absoluteFilePath(
        QString("%1_cooked_%2.tar.gz")
          .arg(r.fixture.name)
          .arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss")));
    QProcess tar;
    tar.start("/bin/sh", QStringList()
              << "-c"
              << QString("tar -C %1 -czf %2 %3")
                   .arg(QFileInfo(cookedSrc).absolutePath(), tarPath, r.fixture.name));
    tar.waitForFinished(120000);
    if (tar.exitCode() != 0 || !QFileInfo(tarPath).isFile()) {
      out << "  zip [" << r.fixture.name << "] FAILED - tar exit "
          << tar.exitCode() << "\n";
      continue;
    }
    const QString hyperlink = QString("\033]8;;file://%1\033\\%1\033]8;;\033\\")
                                .arg(tarPath);
    out << "\n[" << r.fixture.name << "] cooked vehicle at:\n  "
        << hyperlink << "\n  (Ctrl+click in supported terminals to open in your "
                        "filemanager / archive viewer.)\n";
  }
  out.flush();

  return (importPasses == fixtures.size()
          && deployPasses == importPasses
          && spawnPasses  == deployPasses) ? 0 : 1;
}
