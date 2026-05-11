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
  QString mesh_path;
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
  if (pid > 0) { (void)::system(QString("kill -9 %1 2>/dev/null").arg(pid).toLocal8Bit().constData()); }
}




std::atomic<qint64> g_editorPid{-1};
std::atomic<qint64> g_carlaPid{-1};

void killAllChildrenAndExit(int sig)
{
  const qint64 ep = g_editorPid.load();
  const qint64 cp = g_carlaPid.load();
  if (ep > 0) ::kill(static_cast<pid_t>(ep), SIGKILL);
  if (cp > 0) ::kill(static_cast<pid_t>(cp), SIGKILL);

  (void)::system("pkill -KILL -f 'UnrealEditor.*CarlaUnreal' 2>/dev/null; "
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
    (void)::system("pkill -KILL -f 'UnrealEditor.*CarlaUnreal' 2>/dev/null; "
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
  float max_steer_angle    = 70.f;
  float max_brake_torque   = 1500.f;
  float susp_max_raise     = 10.f;
  float susp_max_drop      = 10.f;
  float susp_damping      = 0.65f;
  cs::SizeClass size_class = cs::SizeClass::Sedan;
  QString sizeClassLabel  = "sedan";
};

cs::VehicleSpec makeSpec(const Fixture &f, const UserKnobs &k,
                         const QString &tires_path = QString())
{





  cs::VehicleSpec s;
  cs::MeshGeometry g = cs::load_mesh_geometry(f.mesh_path);


  float scale_to_cm = 1.0f;
  if (g.valid) {
    float xn=1e30f, xx=-1e30f, yn=1e30f, yx=-1e30f, zn=1e30f, zx=-1e30f;
    for (int i = 0; i < g.vertex_count(); ++i) {
      float x, y, z; g.vertex(i, x, y, z);
      if (x < xn) xn = x;
      if (x > xx) xx = x;
      if (y < yn) yn = y;
      if (y > yx) yx = y;
      if (z < zn) zn = z;
      if (z > zx) zx = z;
    }
    const float maxExt = std::max({xx-xn, yx-yn, zx-zn});
    if      (maxExt >= 200.f && maxExt <= 800.f)        scale_to_cm = 1.f;
    else if (maxExt >= 2.f   && maxExt <= 8.f)          scale_to_cm = 100.f;
    else if (maxExt >= 2000.f && maxExt <= 8000.f)      scale_to_cm = 0.1f;
    else if (maxExt > 0.f && maxExt < 2.f)              scale_to_cm = 450.f / std::max(0.01f, maxExt);
  }
  if (g.valid) {
    const cs::MeshAnalysisResult ar = cs::analyze_mesh(g, scale_to_cm);
    if (ar.ok) {
      s = cs::build_spec_from_analysis(ar, f.name, f.mesh_path, scale_to_cm, 2, 0);
    }
  }
  if (s.name.isEmpty()) {

    s.name     = QString(f.name);
    s.mesh_path = QString(f.mesh_path);
    for (size_t i = 0; i < 4; ++i) {
      s.wheels[i].radius = 35.f;
      s.wheels[i].width  = 22.f;
    }
    s.wheels[0].x =  140; s.wheels[0].y = -80; s.wheels[0].z = 35;
    s.wheels[1].x =  140; s.wheels[1].y =  80; s.wheels[1].z = 35;
    s.wheels[2].x = -140; s.wheels[2].y = -80; s.wheels[2].z = 35;
    s.wheels[3].x = -140; s.wheels[3].y =  80; s.wheels[3].z = 35;
  }










  bool wheelsDegenerate =
      (s.wheels[0].x == 0 && s.wheels[1].x == 0 &&
       s.wheels[2].x == 0 && s.wheels[3].x == 0)
      || (!tires_path.isEmpty() && QFileInfo(tires_path).isFile());


  for (size_t i = 0; i < 4; ++i)
    if (s.wheels[i].radius * scale_to_cm > 0 && s.wheels[i].radius * scale_to_cm < 5.f)
      wheelsDegenerate = true;
  if (!wheelsDegenerate && g.valid) {


    float x_min = 1e30f, x_max = -1e30f, y_min = 1e30f, y_max = -1e30f;
    for (int i = 0; i < g.vertex_count(); ++i) {
      float x, y, z; g.vertex(i, x, y, z);
      if (x < x_min) x_min = x;
      if (x > x_max) x_max = x;
      if (y < y_min) y_min = y;
      if (y > y_max) y_max = y;
    }
    float wXMin = 1e30f, wXMax = -1e30f, wYMin = 1e30f, wYMax = -1e30f;
    for (size_t i = 0; i < 4; ++i) {
      if (s.wheels[i].x < wXMin) wXMin = s.wheels[i].x;
      if (s.wheels[i].x > wXMax) wXMax = s.wheels[i].x;
      if (s.wheels[i].y < wYMin) wYMin = s.wheels[i].y;
      if (s.wheels[i].y > wYMax) wYMax = s.wheels[i].y;
    }
    const float bodyExtent  = std::max(x_max - x_min, y_max - y_min);
    const float wheelSpread = std::max(wXMax - wXMin, wYMax - wYMin);
    if (wheelSpread < bodyExtent * 0.40f) wheelsDegenerate = true;
  }
  if (wheelsDegenerate && g.valid) {
    cs::WheelTemplate tpl;
    if (!tires_path.isEmpty() && QFileInfo(tires_path).isFile()
        && QFileInfo(tires_path).suffix().toLower() == "obj") {
      tpl = cs::wheel_template_from_tires_obj(tires_path, 1.0f);
    }
    if (!tpl.valid) { tpl.radius = 35.f; tpl.width = 22.f; tpl.valid = true; }

    float x_min = 1e30f, x_max = -1e30f;
    float y_min = 1e30f, y_max = -1e30f;
    float z_min = 1e30f, z_max = -1e30f;
    for (int i = 0; i < g.vertex_count(); ++i) {
      float x, y, z; g.vertex(i, x, y, z);
      if (x < x_min) x_min = x;
      if (x > x_max) x_max = x;
      if (y < y_min) y_min = y;
      if (y > y_max) y_max = y;
      if (z < z_min) z_min = z;
      if (z > z_max) z_max = z;
    }



    const float ext[3] = { x_max - x_min, y_max - y_min, z_max - z_min };
    auto longestAxis = [&]() {
      int a = 0;
      for (int i = 1; i < 3; ++i) if (ext[i] > ext[a]) a = i;
      return a;
    }();
    int up_axis;
    {
      int rem[2], idx = 0;
      for (int i = 0; i < 3; ++i) if (i != longestAxis) rem[idx++] = i;
      up_axis = (ext[rem[0]] <= ext[rem[1]]) ? rem[0] : rem[1];
    }
    const int latAxisIdx = 3 - longestAxis - up_axis;
    const float fwdMinV = (longestAxis == 0) ? x_min : (longestAxis == 1 ? y_min : z_min);
    const float fwdMaxV = (longestAxis == 0) ? x_max : (longestAxis == 1 ? y_max : z_max);
    const float latMinV = (latAxisIdx == 0) ? x_min : (latAxisIdx == 1 ? y_min : z_min);
    const float latMaxV = (latAxisIdx == 0) ? x_max : (latAxisIdx == 1 ? y_max : z_max);
    const float upMinV  = (up_axis == 0) ? x_min : (up_axis == 1 ? y_min : z_min);
    const float fwdInset = (fwdMaxV - fwdMinV) * 0.21f;




    const float latExt   = latMaxV - latMinV;
    const float tire_half = std::max(tpl.width * 0.5f, 1.0f) /
                           std::max(scale_to_cm, 1e-3f);
    const float latInset = std::max(latExt * 0.135f, tire_half);
    const float wheelUp   = upMinV + std::max(tpl.radius / std::max(scale_to_cm, 1e-3f), 1.0f / std::max(scale_to_cm, 1e-3f));
    const float fwdC = 0.5f * (fwdMinV + fwdMaxV);
    const float latC = 0.5f * (latMinV + latMaxV);
    const float fwdAxisVals[4] = { fwdMaxV - fwdInset, fwdMaxV - fwdInset,
                                   fwdMinV + fwdInset, fwdMinV + fwdInset };
    const float latAxisVals[4] = { latMinV + latInset, latMaxV - latInset,
                                   latMinV + latInset, latMaxV - latInset };
    for (size_t i = 0; i < 4; ++i) {
      float c[3] = { 0, 0, 0 };
      c[longestAxis] = fwdAxisVals[i];
      c[latAxisIdx]  = latAxisVals[i];
      c[up_axis]      = wheelUp;
      s.wheels[i].x = (c[longestAxis] - fwdC)   * scale_to_cm;
      s.wheels[i].y = (c[latAxisIdx]  - latC)   * scale_to_cm;
      s.wheels[i].z = (c[up_axis]      - upMinV) * scale_to_cm;
      s.wheels[i].radius = tpl.radius;
      s.wheels[i].width  = tpl.width;
    }
  }
  s.content_path   = "/Game/Carla/Static/Vehicles/4Wheeled";
  s.base_vehicle_bp = "/Game/Carla/Blueprints/Vehicles/Mustang/BP_Mustang";
  s.mass          = k.mass;
  s.susp_damping   = k.susp_damping;
  s.size_class     = k.size_class;
  for (size_t i = 0; i < 4; ++i) {
    s.wheels[i].max_steer_angle  = (i < 2) ? k.max_steer_angle : 0.f;
    s.wheels[i].max_brake_torque = k.max_brake_torque;
    s.wheels[i].susp_max_raise   = k.susp_max_raise;
    s.wheels[i].susp_max_drop    = k.susp_max_drop;
  }
  return s;
}

UserKnobs detectKnobs(const QString &mesh_path)
{
  UserKnobs k;
  cs::SizeClass cls = cs::classify_by_name(QFileInfo(mesh_path).completeBaseName());
  if (cls == cs::SizeClass::Unknown) {
    cs::MeshGeometry g = cs::load_mesh_geometry(mesh_path);
    if (g.valid) {
      const cs::MeshAnalysisResult ar = cs::analyze_mesh(g, 1.0f);
      if (ar.ok) cls = ar.size_class;
    }
  }
  if (cls == cs::SizeClass::Unknown) cls = cs::SizeClass::Sedan;
  const cs::SizePreset p = cs::preset_for_size_class(cls);
  k.mass           = p.mass;
  k.max_steer_angle  = p.max_steer_angle;
  k.max_brake_torque = p.max_brake_torque;
  k.susp_max_raise   = p.susp_max_raise;
  k.susp_max_drop    = p.susp_max_drop;
  k.susp_damping    = p.susp_damping;
  k.size_class      = cls;
  k.sizeClassLabel = cs::size_class_name(cls);
  return k;
}

float promptFloat(QTextStream &, const QString &label, float def,
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





void stageMaterialsBeside(const QString &objOnDisk, const QString &mtl);





void autoStageBodyMaterials(const QString &body_obj, const QString &targetObj) {
  if (body_obj.isEmpty() || targetObj.isEmpty()) return;
  if (!QFileInfo(body_obj).isFile() || !QFileInfo(targetObj).isFile()) return;
  QFile in(body_obj);
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



  const QString sourceObj = QFileInfo(body_obj).canonicalFilePath();
  const QString sourceDir = sourceObj.isEmpty()
      ? QFileInfo(body_obj).absolutePath()
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
      == QFileInfo(mtlDir).canonicalFilePath()) return;


  const QString objBase = QFileInfo(objOnDisk).fileName();
  for (const QFileInfo &fi : QDir(mtlDir).entryInfoList(
           QDir::Files | QDir::NoSymLinks | QDir::NoDotAndDotDot)) {
    if (fi.fileName() == objBase) continue;
    const QString dst = QDir(objDir).absoluteFilePath(fi.fileName());
    QFile::remove(dst);
    QFile::copy(fi.absoluteFilePath(), dst);
  }
}





QString centerObjOnDisk(const QString &srcPath, const QString &out_dir)
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
            if (x < xmin) xmin = x;
            if (x > xmax) xmax = x;
            if (y < ymin) ymin = y;
            if (y > ymax) ymax = y;
            if (z < zmin) zmin = z;
            if (z > zmax) zmax = z;
          }
        }
      }
    }
  }
  if (xmin > xmax) return srcPath;









  const double extX = xmax - xmin;
  const double extY = ymax - ymin;
  const double extZ = zmax - zmin;
  const double maxExt = std::max({extX, extY, extZ});

  enum Axis { AX_X, AX_Y, AX_Z };
  Axis fwdAxis = AX_X, up_axis = AX_Z, latAxis = AX_Y;

  if      (extX >= extY && extX >= extZ) fwdAxis = AX_X;
  else if (extY >= extX && extY >= extZ) fwdAxis = AX_Y;
  else                                    fwdAxis = AX_Z;

  if      (extZ <= extX && extZ <= extY) up_axis = AX_Z;
  else if (extY <= extX && extY <= extZ) up_axis = AX_Y;
  else                                    up_axis = AX_X;
  if (up_axis == fwdAxis) {

    up_axis = (fwdAxis == AX_X) ? AX_Y : (fwdAxis == AX_Y ? AX_Z : AX_X);
  }
  latAxis = static_cast<Axis>(3 - fwdAxis - up_axis);


  auto axMin = [&](Axis a) { return a == AX_X ? xmin : a == AX_Y ? ymin : zmin; };
  auto axMax = [&](Axis a) { return a == AX_X ? xmax : a == AX_Y ? ymax : zmax; };
  const double fwdMin  = axMin(fwdAxis), fwdMax = axMax(fwdAxis);
  const double latMin  = axMin(latAxis), latMax = axMax(latAxis);
  const double upMin   = axMin(up_axis);
  const double fwdC    = 0.5 * (fwdMin + fwdMax);
  const double latC    = 0.5 * (latMin + latMax);



  double unitScale = 1.0;
  if      (maxExt >= 200.0 && maxExt <= 800.0) unitScale = 1.0;
  else if (maxExt >= 2.0   && maxExt <= 8.0)   unitScale = 100.0;
  else if (maxExt >= 2000.0 && maxExt <= 8000.0) unitScale = 0.1;
  else if (maxExt > 0.0 && maxExt < 2.0)       unitScale = 450.0 / maxExt;

  QDir().mkpath(out_dir);
  const QString outPath = QDir(out_dir).filePath(
      QFileInfo(srcPath).completeBaseName() + "_centered.obj");
  QFile out(outPath);
  if (!out.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    return srcPath;
  QTextStream os(&out);








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

          const double rawLat = pickFromAxis(latAxis, x, y, z);
          const double rawFwd = pickFromAxis(fwdAxis, x, y, z);
          const double rawUp  = pickFromAxis(up_axis,  x, y, z);

          const double centeredLat = rawLat - latC;
          const double centeredFwd = rawFwd - fwdC;
          const double centeredUp  = rawUp  - upMin;


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

void openCalibrationViews(QTextStream &out, const QString &mesh_path,
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



      std::array<float, 12> w{};
      for (size_t i = 0; i < 4; ++i) {
        w[i*3 + 0] = spec->wheels[i].x;
        w[i*3 + 1] = spec->wheels[i].y;
        w[i*3 + 2] = spec->wheels[i].z;
      }
      cs::open_calibration_window_with_wheels(mesh_path, w);
    } else {
      cs::open_calibration_window(mesh_path);
    }
    return;
  }
  const QString stem = QFileInfo(mesh_path).completeBaseName();
  const QString out_dir = workspaceRoot() + "/preview_" + stem;
  QDir(out_dir).removeRecursively();
  QProcess preview;
  preview.start(previewBin, QStringList() << mesh_path << out_dir);
  preview.waitForFinished(45000);
  out << "  no display - wrote calibration PNGs (manual viewer):\n"
      << "    " << out_dir << "/top.png\n"
      << "    " << out_dir << "/side.png\n"
      << "    " << out_dir << "/front.png\n";
}

void promptKnobs(QTextStream &out, UserKnobs &k, const QString &mesh_path,
                 const cs::VehicleSpec *spec = nullptr)
{
  out << "\n=== CALIBRATION ===\n";
  out << "  mesh        : " << mesh_path << "\n";
  out << "  size class  : " << k.sizeClassLabel
      << " (filename + chassis-length heuristic)\n";
  openCalibrationViews(out, mesh_path, spec);
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
  k.max_steer_angle  = promptFloat(out, "Max Steer Angle           ", k.max_steer_angle,  "deg");
  k.max_brake_torque = promptFloat(out, "Max Brake Torque         *", k.max_brake_torque, "Nm");
  k.susp_max_raise   = promptFloat(out, "Suspension Max Raise     *", k.susp_max_raise,   "cm");
  k.susp_max_drop    = promptFloat(out, "Suspension Max Drop      *", k.susp_max_drop,    "cm");
  k.susp_damping    = promptFloat(out, "Suspension Damping       *", k.susp_damping,    "");
  out << "  using mass=" << k.mass << "kg  brake=" << k.max_brake_torque
      << "Nm  susp=(" << k.susp_max_raise << "/" << k.susp_max_drop << "cm @ "
      << k.susp_damping << ")  steer=" << k.max_steer_angle << "deg\n";
  out.flush();
}

struct ImportRow {
  Fixture  fixture;
  bool     imported = false;
  QString  asset_path;
  QString  importDetail;
  bool     deployed = false;
  int      files_copied = 0;
  int      cooked_files = 0;
  QString  deployDetail;
  bool     spawned = false;
  QString  spawnDetail;
};

}

int runVehicleImportCli(int argc, char **argv)
{
  QCoreApplication app(argc, argv);
  installCleanupHandlers();
  int  port         = 18584;
  int  carla_port    = 3000;
  bool spawnEditor  = true;
  bool fullDrive    = false;
  bool spawnCarla   = true;
  bool visible_carla = true;
  bool screenshots  = false;
  bool spawnOnly    = false;
  bool manual_drive  = false;
#ifdef CARLA_STUDIO_WITH_LIBCARLA
  int  auto_drive_seconds = 60;
#endif
  QString tires_path;
  QString mtlPath;
  QString screenshotsDir = workspaceRoot() + "/shots";
  QString compareWith;
  for (int i = 1; i < argc; ++i) {
    const QString a = QString::fromLocal8Bit(argv[i]);
    if      (a == "--port"             && i + 1 < argc) port      = QString(argv[++i]).toInt();
    else if (a == "--carla-port"       && i + 1 < argc) carla_port = QString(argv[++i]).toInt();
    else if (a == "--no-spawn-editor")                  spawnEditor = false;
    else if (a == "--no-spawn-carla")                   spawnCarla  = false;
    else if (a == "--full-drive")                       fullDrive   = true;
    else if (a == "--spawn-only")                     { spawnOnly = true; fullDrive = true; spawnEditor = false; }
    else if (a == "--visible")                          visible_carla = true;
    else if (a == "--screenshots")                      screenshots = true;
    else if (a == "--shots-dir"        && i + 1 < argc) screenshotsDir = QString(argv[++i]);
    else if (a == "--compare-with"     && i + 1 < argc) compareWith = QString(argv[++i]);
    else if (a == "--manual_drive")                     manual_drive = true;
    else if (a == "--manual-drive")                     manual_drive = true;
#ifdef CARLA_STUDIO_WITH_LIBCARLA
    else if (a == "--auto-seconds"     && i + 1 < argc) auto_drive_seconds = QString(argv[++i]).toInt();
#endif
    else if (a == "--tires"            && i + 1 < argc) tires_path = QString(argv[++i]);
    else if (a == "--mtl"              && i + 1 < argc) mtlPath   = QString(argv[++i]);
    else if (a == "--debug-merge-only") {



      qputenv("CARLA_VEHICLE_TEST_FIXTURES_DIR_DEBUG_MERGE_ONLY", "1");
    }
    else if (a == "--mesh"             && i + 1 < argc) {
      const QString mesh_path = QString(argv[++i]);
      if (!QFileInfo::exists(mesh_path)) {
        QTextStream(stderr) << "--mesh: file not found: " << mesh_path << "\n";
        return 8;
      }
      QFileInfo fi(mesh_path);
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
  if (screenshots) visible_carla = true;
  qputenv("CARLA_VEHICLE_IMPORTER_PORT", QByteArray::number(port));

  QTextStream out(stdout);

  if (qEnvironmentVariableIsSet("CARLA_VEHICLE_TEST_FIXTURES_DIR_DEBUG_MERGE_ONLY")) {
    const QList<Fixture> fxts = loadFixtures();
    if (fxts.isEmpty()) {
      out << "--debug-merge-only: no fixture loaded (set --mesh first)\n";
      return 11;
    }
    const Fixture &f = fxts.first();
    UserKnobs knobs = detectKnobs(f.mesh_path);
    cs::VehicleSpec s = makeSpec(f, knobs, tires_path);
    out << "[debug-merge-only] derived wheel anchors (cm) for " << f.name << ":\n";
    for (size_t i = 0; i < 4; ++i) {
      out << QString("  wheel[%1] x=%2 y=%3 z=%4 r=%5 w=%6\n")
              .arg(i).arg(s.wheels[i].x, 0, 'f', 1).arg(s.wheels[i].y, 0, 'f', 1)
              .arg(s.wheels[i].z, 0, 'f', 1)
              .arg(s.wheels[i].radius, 0, 'f', 1).arg(s.wheels[i].width, 0, 'f', 1);
    }
    if (!tires_path.isEmpty()
        && QFileInfo(f.mesh_path).suffix().toLower() == "obj"
        && QFileInfo(tires_path).isFile()) {
      const QString out_dir = workspaceRoot() + "/merged_" + f.name;
      QDir(out_dir).removeRecursively();
      QDir().mkpath(out_dir);
      const cs::ObjMergeResult mr = cs::merge_body_and_tires(
          f.mesh_path, tires_path, out_dir, s.wheels);
      out << "[debug-merge-only] merge: " << (mr.ok ? "OK" : "FAIL")
          << " - " << mr.reason << "\n";
      if (mr.ok) {
        out << "[debug-merge-only] output: " << mr.output_path << "\n";
        autoStageBodyMaterials(f.mesh_path, mr.output_path);
        const QString centered = centerObjOnDisk(
            mr.output_path, workspaceRoot() + "/centered_" + f.name + "_merged");
        if (centered != mr.output_path) {
          autoStageBodyMaterials(f.mesh_path, centered);
          out << "[debug-merge-only] centered output: " << centered << "\n";
        }
      }
    } else if (QFileInfo(f.mesh_path).suffix().toLower() == "obj") {

      const QString centered = centerObjOnDisk(
          f.mesh_path, workspaceRoot() + "/centered_" + f.name);
      if (centered != f.mesh_path) {
        autoStageBodyMaterials(f.mesh_path, centered);
        out << "[debug-merge-only] centered output: " << centered << "\n";
      }
    }
    return 0;
  }

  if (portOpen(port) || portOpen(carla_port)) {
    out << "[cleanup] reclaiming ports - killing leftover UnrealEditor / "
           "CarlaUnreal-Linux-Shipping from a prior run …\n";
    QProcess::execute("/bin/sh", QStringList() << "-c"
        << "pkill -KILL -f 'UnrealEditor.*CarlaUnreal' 2>/dev/null; "
           "pkill -KILL -f 'CarlaUnreal-Linux-Shipping' 2>/dev/null; "
           "pkill -KILL -f 'carla-studio-vehicle-preview --interactive' 2>/dev/null; true");
    QThread::sleep(3);
  }

  if (visible_carla) {
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
      visible_carla = false;
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

  qint64 editor_pid = -1;
  const QString editor_log = workspaceRoot() + QString("/editor_%1.log").arg(port);
  if (spawnEditor) {
    if (portOpen(port)) {
      out << "Port " << port << " already in use - pass --no-spawn-editor or pick another with --port.\n";
      return 2;
    }
    QFile::remove(editor_log);
    out << "Launching headless editor on port " << port << " (stdout→" << editor_log << ") …\n";
    editor_pid = launchEditor(port, editor_log);
    g_editorPid.store(editor_pid);
    if (editor_pid <= 0) { out << "Editor launch failed.\n"; return 3; }
    out << "Editor pid: " << editor_pid << "\n";
    out << "Waiting up to 240 s for port " << port << " …\n";
    if (!waitPort(port, 240)) {
      out << "Editor never opened port. Tail of log:\n" << tail(editor_log, 25) << "\n";
      killProcess(editor_pid);
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
    if (spawnEditor && editor_pid > 0) killProcess(editor_pid);
    return 6;
  }

  QList<ImportRow> rows;
  rows.reserve(fixtures.size());

  const QString srcRoot      = qEnvironmentVariable("CARLA_SRC_ROOT");
  const QString engine_path   = qEnvironmentVariable("CARLA_UNREAL_ENGINE_PATH");
  const QString shippingRoot = qEnvironmentVariable("CARLA_SHIPPING_ROOT");
  const QString uproject     = srcRoot + "/Unreal/CarlaUnreal/CarlaUnreal.uproject";
  const QString editor_binary = engine_path + "/Engine/Binaries/Linux/UnrealEditor";

  if (spawnOnly) {
    out << "(--spawn-only: skipping import + deploy, using pre-cooked assets)\n";
    for (const Fixture &f : fixtures) {
      ImportRow r; r.fixture = f;
      r.imported = true;
      r.deployed = true;
      r.asset_path = QString("/Game/Carla/Static/Vehicles/4Wheeled/%1/BP_%1").arg(f.name);
      rows.append(r);
    }
  } else {
    out << "\n=== IMPORT ===\n";
    for (const Fixture &f : fixtures) {
      ImportRow r; r.fixture = f;
      if (!QFile::exists(f.mesh_path)) {
        r.importDetail = "mesh missing on disk";
        out << QString("[skip] %1 - mesh missing at %2\n").arg(f.name, f.mesh_path);
        rows.append(r);
        continue;
      }
      UserKnobs knobs = detectKnobs(f.mesh_path);


      QString viewMeshPath = f.mesh_path;
      QString mergedPath;
      if (!tires_path.isEmpty()
          && QFileInfo(f.mesh_path).suffix().toLower() == "obj"
          && QFileInfo(tires_path).suffix().toLower() == "obj"
          && QFileInfo(tires_path).isFile()) {
        const cs::VehicleSpec preSpec = makeSpec(f, knobs, tires_path);
        const QString out_dir = workspaceRoot() + "/merged_" + f.name;
        QDir(out_dir).removeRecursively();
        QDir().mkpath(out_dir);
        const cs::ObjMergeResult mr = cs::merge_body_and_tires(
            f.mesh_path, tires_path, out_dir, preSpec.wheels);
        if (mr.ok) {
          mergedPath = mr.output_path;
          viewMeshPath = mergedPath;
          autoStageBodyMaterials(f.mesh_path, mergedPath);



          const QString centered = centerObjOnDisk(
              mergedPath, workspaceRoot() + "/centered_" + f.name + "_merged");
          if (centered != mergedPath) {
            autoStageBodyMaterials(f.mesh_path, centered);
            viewMeshPath = centered;
          }
          out << QString("  merged body+tires (%1 verts body + %2 verts tires "
                         "→ %3 verts; %4/%5 tire copies placed) → %6\n")
                  .arg(mr.body_vertex_count).arg(mr.tire_vertex_count)
                  .arg(mr.merged_vertex_count).arg(mr.tire_copies_placed)
                  .arg(mr.tire_source_count).arg(mr.output_path);
        } else {
          out << QString("  body+tires merge failed: %1 - calibration will show "
                         "chassis only.\n").arg(mr.reason);
        }
      } else if (!tires_path.isEmpty()) {
        out << "  --tires given but path is not a readable .obj - ignoring.\n";
      }



      if (mergedPath.isEmpty()
          && QFileInfo(f.mesh_path).suffix().toLower() == "obj") {
        const QString centered = centerObjOnDisk(
            f.mesh_path, workspaceRoot() + "/centered_" + f.name);
        if (centered != f.mesh_path) {
          autoStageBodyMaterials(f.mesh_path, centered);
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



      cs::VehicleSpec s = makeSpec(f, knobs, tires_path);
      promptKnobs(out, knobs, viewMeshPath, &s);
      if (!mergedPath.isEmpty()) s.mesh_path = mergedPath;
      QJsonObject json = cs::spec_to_json(s);
      out << "\n=== IMPORT (5-stage UE pipeline; 30-90 s blocking) ===\n"
             "  → sending spec to editor on port " << port << "\n"
             "    [stage 1/5] preflight (canonicalize OBJ axes & scale)\n"
             "    [stage 2/5] interchange (UE5 mesh import)\n"
             "    [stage 3/5] build SK (skeletal mesh from template)\n"
             "    [stage 4/5] build BP (vehicle blueprint + 4 wheel BPs)\n"
             "    [stage 5/5] persist (SaveAsset to disk)\n"
             "  detailed log: " << editor_log
          << " (tail -f to watch progress live)\n";
      out.flush();
      QString resp = cs::send_json(json);
      if (resp.isEmpty()) {
        r.importDetail = "no response from editor";
        out << QString("[FAIL] %1 - no response from editor\n").arg(f.name);
        rows.append(r);
        continue;
      }
      const QJsonObject obj = QJsonDocument::fromJson(resp.toUtf8()).object();
      r.imported     = obj.value("success").toBool();
      r.asset_path    = obj.value("asset_path").toString();
      r.importDetail = r.imported ? r.asset_path : obj.value("error").toString();
      out << QString(r.imported ? "[ OK ] %1 → %2\n" : "[FAIL] %1 - %2\n")
              .arg(f.name, r.importDetail);
      rows.append(r);
    }

    int importPasses = 0;
    for (const ImportRow &r : rows) if (r.imported) ++importPasses;
    out << importPasses << " / " << fixtures.size() << " imports succeeded.\n";

    if (!fullDrive) {
      if (spawnEditor && editor_pid > 0) {
        out << "Killing editor pid " << editor_pid << " …\n";
        killProcess(editor_pid);
      }
      return importPasses == fixtures.size() ? 0 : 1;
    }

    if (editor_pid > 0) {
      out << "\nKilling import editor pid " << editor_pid << " before cook commandlet …\n";
      killProcess(editor_pid);
      editor_pid = -1;
      QThread::sleep(2);
    }

    out << "\n=== REGISTER + DEPLOY (cook) ===\n";
    out << "uproject     : " << uproject << "\n";
    out << "shippingRoot : " << shippingRoot << "\n";
    for (ImportRow &r : rows) {
      if (!r.imported) continue;
      cs::VehicleRegistration reg;
      reg.uproject_path      = uproject;
      reg.shipping_carla_root = shippingRoot;
      reg.bp_asset_path       = r.asset_path;
      reg.make              = "Custom";
      reg.model             = r.fixture.name;
      reg.editor_binary      = editor_binary;

      out << "\n[" << r.fixture.name << "] registering …\n";
      const cs::RegisterResult rr = cs::register_vehicle_in_json(reg);
      out << "  register: " << (rr.ok ? "OK" : "FAIL") << " - " << rr.detail << "\n";
      if (!rr.ok) { r.deployDetail = rr.detail; continue; }

      out << "[" << r.fixture.name << "] deploying (UE cook commandlet - 2-3 min first cook, "
                                       "<30 s on subsequent cooks; "
                                       "watch " + srcRoot + "/Unreal/CarlaUnreal/"
                                       "Saved/Logs/ for live cook output) …\n";
      out.flush();
      const cs::DeployResult dr = cs::deploy_vehicle_to_shipping_carla(reg);
      r.deployed     = dr.ok;
      r.files_copied  = dr.files_copied;
      r.cooked_files  = dr.cooked_files;
      r.deployDetail = dr.detail;
      out << "  deploy: " << (dr.ok ? "OK" : "FAIL")
          << " - cooked=" << dr.cooked_files
          << " copied=" << dr.files_copied
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

  qint64 carla_pid = -1;
  const QString carla_log = workspaceRoot() + QString("/carla_%1.log").arg(carla_port);
  if (spawnCarla) {
    if (portOpen(carla_port)) {
      out << "[cleanup] RPC port " << carla_port
          << " busy - sweeping leftover CarlaUnreal-Linux-Shipping…\n";
      QProcess::execute("/bin/sh", QStringList() << "-c"
          << "pkill -KILL -f 'CarlaUnreal-Linux-Shipping' 2>/dev/null; "
             "fuser -k " + QString::number(carla_port) + "/tcp 2>/dev/null; true");
      QThread::sleep(3);
      if (portOpen(carla_port)) {
        out << "RPC port " << carla_port
            << " still busy after sweep - pass --no-spawn-carla or pick --carla-port.\n";
        return 8;
      }
      out << "[cleanup] port " << carla_port << " freed; continuing.\n";
    }
    const QString script = findShippingScript(shippingRoot);
    if (script.isEmpty()) {
      out << "Shipping CARLA launcher not found in " << shippingRoot
          << " (need CarlaUnreal.sh / CarlaUE5.sh / CarlaUE4.sh).\n";
      return 9;
    }
    QFile::remove(carla_log);
    out << "\n=== LAUNCH SHIPPING CARLA "
        << (visible_carla ? "(visible RHI window)" : "(headless / -nullrhi)") << " ===\n";
    out << "script: " << script << " on RPC port " << carla_port
        << " (stdout→" << carla_log << ") …\n";
    out.flush();
    carla_pid = launchShippingCarla(script, carla_port, carla_log, visible_carla);
    g_carlaPid.store(carla_pid);
    if (carla_pid <= 0) {
      out << "CARLA launch failed.\n";
      return 10;
    }
    out << "CARLA pid: " << carla_pid << " - Unreal is loading content "
        << "(window appears in 30-60 s once shaders/Maps finish; until then it "
        << "looks like nothing happens). Polling RPC port " << carla_port
        << " every 1 s, up to 120 s …\n";
    out.flush();
    if (!waitPort(carla_port, 120)) {
      out << "CARLA never opened RPC port. Tail of log:\n"
          << tail(carla_log, 30) << "\n";
      killProcess(carla_pid);
      return 11;
    }
    out << "CARLA RPC ready on " << carla_port << ".\n";
    QThread::sleep(5);
  } else if (!portOpen(carla_port)) {
    out << "No CARLA on RPC port " << carla_port
        << " and --no-spawn-carla passed. Aborting.\n";
    return 12;
  }

  out << "\n=== SPAWN ===\n";
  QDir().mkpath(screenshotsDir);
  for (ImportRow &r : rows) {
    if (!r.deployed) continue;
    const cs::SpawnResult sr = cs::spawn_in_running_carla(
        "Custom", r.fixture.name, "localhost", carla_port);
    r.spawned     = (sr.kind == cs::SpawnResult::Kind::Spawned);
    r.spawnDetail = sr.detail;
    out << QString(r.spawned ? "[ OK ] %1 - %2\n" : "[FAIL] %1 - %2\n")
            .arg(r.fixture.name, sr.detail);

#ifdef CARLA_STUDIO_WITH_LIBCARLA
    if (r.spawned) {
      try {
        carla::client::Client client("localhost", carla_port);
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

          QThread::sleep(2);

          const QString shot1 = QString("%1/%2_01_at_spawn.png")
                                  .arg(screenshotsDir, r.fixture.name);
          (void)::system(QString("scrot %1 2>/dev/null").arg(shot1).toLocal8Bit().constData());
          out << QString("  shot → %1\n").arg(shot1);
          out.flush();

          struct DirResult { const char *label; float dist; float fwdDot; float yawDelta; };

          auto doDir = [&](const char *label, float throttle, float steer, bool reverse) -> DirResult {
            DirResult d; d.label = label;
            if (auto v2 = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
              v2->SetAutopilot(false);
              carla::rpc::VehicleControl brk; brk.brake = 1.f; brk.throttle = 0.f;
              v2->ApplyControl(brk);
              v2->SetSimulatePhysics(false);
            }
            QThread::msleep(300);
            ours->SetTransform(t0);
            ours->SetTargetVelocity(carla::geom::Vector3D(0.f, 0.f, 0.f));
            QThread::msleep(200);
            if (auto v2 = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
              v2->SetSimulatePhysics(true);
            }
            QThread::msleep(1500);

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
            (void)::system(QString("scrot %1 2>/dev/null").arg(shot).toLocal8Bit().constData());
            out << QString("    shot → %1\n").arg(shot);
            out.flush();
            return d;
          };

          if (!manual_drive) {
            out << QString("  SAE L5 autopilot (Traffic Manager) - driving for %1 s …\n")
                    .arg(auto_drive_seconds);
            out.flush();
            if (auto v2 = std::dynamic_pointer_cast<carla::client::Vehicle>(ours)) {
              v2->SetAutopilot(true);
            }
            const auto p0 = ours->GetTransform().location;
            const int  ticks = auto_drive_seconds;
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
                    .arg(total,0,'f',1).arg(auto_drive_seconds);
            r.spawnDetail += QString(" | autopilot d=%1m/%2s")
                                .arg(total,0,'f',1).arg(auto_drive_seconds);
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
          if (manual_drive) {
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

  if (spawnCarla && carla_pid > 0) {
    if (manual_drive) {
      waitForEnter(out, "\nClose the CARLA window (or press ENTER here) "
                        "to stop the simulator and zip the cooked vehicle…\n");
    } else {
      out << "\nAutopilot run finished - closing CARLA and zipping…\n";
    }
    out << "Killing CARLA pid " << carla_pid << " …\n";
    killProcess(carla_pid);
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
