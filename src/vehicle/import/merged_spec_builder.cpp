// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/merged_spec_builder.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace carla_studio::vehicle_import {

WheelTemplate wheel_template_from_analysis(const MeshAnalysisResult &r) {
  WheelTemplate t;
  if (!r.ok || r.wheels.empty()) return t;
  float sumR = 0, sumW = 0;
  for (const WheelCandidate &w : r.wheels) { sumR += w.radius; sumW += w.width; }
  const float n = static_cast<float>(r.wheels.size());
  t.radius = sumR / n;
  t.width  = sumW / n;
  t.valid  = t.radius > 0;
  return t;
}

WheelTemplate wheel_template_from_tires_obj(const QString &tires_obj_path,
                                        float scale_to_cm) {
  WheelTemplate t;
  QFile f(tires_obj_path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return t;
  QTextStream in(&f);
  double bbMin[3] = {  std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max() };
  double bbMax[3] = { -std::numeric_limits<double>::max(),
                      -std::numeric_limits<double>::max(),
                      -std::numeric_limits<double>::max() };
  bool any = false;
  while (!in.atEnd()) {
    const QString line = in.readLine();
    if (!line.startsWith("v ")) continue;
    const QStringList parts = line.split(' ', Qt::SkipEmptyParts);
    if (parts.size() < 4) continue;
    bool okx = false, oky = false, okz = false;
    const double x = parts[1].toDouble(&okx);
    const double y = parts[2].toDouble(&oky);
    const double z = parts[3].toDouble(&okz);
    if (!okx || !oky || !okz) continue;
    bbMin[0] = std::min(bbMin[0], x); bbMax[0] = std::max(bbMax[0], x);
    bbMin[1] = std::min(bbMin[1], y); bbMax[1] = std::max(bbMax[1], y);
    bbMin[2] = std::min(bbMin[2], z); bbMax[2] = std::max(bbMax[2], z);
    any = true;
  }
  if (!any) return t;
  const double dx = (bbMax[0] - bbMin[0]) * static_cast<double>(scale_to_cm);
  const double dy = (bbMax[1] - bbMin[1]) * static_cast<double>(scale_to_cm);
  const double dz = (bbMax[2] - bbMin[2]) * static_cast<double>(scale_to_cm);
  t.radius = static_cast<float>(std::max(dx, dz) * 0.5);
  t.width  = static_cast<float>(dy);
  t.valid  = t.radius > 0;
  return t;
}

namespace {

constexpr float kSmallVehicleWheelRadiusCm = 18.0f;

void deriveWheelAnchors(const VehicleSpec &body, float radius, float width,
                        std::array<WheelSpec, 4> &out) {
  const float x_min = body.chassis_x_min, x_max = body.chassis_x_max;
  const float y_min = body.chassis_y_min, y_max = body.chassis_y_max;
  const float wheelInsetX = (x_max - x_min) * 0.18f;
  const std::array<float, 4> xs = { x_min + wheelInsetX, x_max - wheelInsetX,
                                    x_min + wheelInsetX, x_max - wheelInsetX };
  const std::array<float, 4> ysSigned = { y_min, y_min, y_max, y_max };
  const float wheelZ = std::max(radius, 1.0f);
  for (size_t i = 0; i < 4; ++i) {
    out[i].x = xs[i];
    out[i].y = ysSigned[i];
    out[i].z = wheelZ;
    out[i].radius = radius;
    out[i].width  = width;
  }
}

}

MergeResult merge_body_and_wheels(const VehicleSpec &body_spec,
                               const WheelTemplate &wheels_tpl,
                               bool wheels_file_provided) {
  MergeResult r;
  r.spec = body_spec;
  if (!body_spec.detected_from_mesh) {
    r.reason = "Body spec is not mesh-detected; merge requires analysis output.";
    return r;
  }

  if (body_spec.has_four_wheels && !wheels_file_provided) {
    r.mode = IngestionMode::Combined;
    r.ok = true;
    r.reason = "Combined: body already includes 4 wheels.";
    return r;
  }

  if (!wheels_tpl.valid && wheels_file_provided) {
    r.reason = "Wheels file provided but template is invalid.";
    return r;
  }
  if (!wheels_tpl.valid && !body_spec.has_four_wheels) {
    r.reason = "Body has no wheels and no wheels file was provided - vehicle would not be drivable.";
    return r;
  }

  if (body_spec.has_four_wheels && wheels_file_provided) {
    float y_min = std::numeric_limits<float>::max();
    float y_max = -std::numeric_limits<float>::max();
    float x_min = std::numeric_limits<float>::max();
    float x_max = -std::numeric_limits<float>::max();
    for (const WheelSpec &w : r.spec.wheels) {
      y_min = std::min(y_min, w.y); y_max = std::max(y_max, w.y);
      x_min = std::min(x_min, w.x); x_max = std::max(x_max, w.x);
    }
    const float chassisX = body_spec.chassis_x_max - body_spec.chassis_x_min;
    const float chassisY = body_spec.chassis_y_max - body_spec.chassis_y_min;
    const bool ySpreadOk = (y_max - y_min) >= chassisY * 0.40f;
    const bool xSpreadOk = (x_max - x_min) >= chassisX * 0.40f;
    const bool detectedSane = ySpreadOk && xSpreadOk;

    if (detectedSane) {
      r.mode = IngestionMode::TireSwap;
      for (WheelSpec &w : r.spec.wheels) {
        w.radius = wheels_tpl.radius;
        w.width  = wheels_tpl.width;
        w.z      = std::max(body_spec.chassis_z_min + wheels_tpl.radius,
                            wheels_tpl.radius * 1.05f);
      }
      r.spec.small_vehicle_needs_wheel_shape =
          wheels_tpl.radius < kSmallVehicleWheelRadiusCm;
      r.ok = true;
      r.reason = QString("Tire-swap: replacing wheels with radius=%1cm, width=%2cm.")
                   .arg(wheels_tpl.radius, 0, 'f', 1).arg(wheels_tpl.width, 0, 'f', 1);
      return r;
    }
    r.mode = IngestionMode::BodyPlusTires;
    deriveWheelAnchors(r.spec, wheels_tpl.radius, wheels_tpl.width, r.spec.wheels);
    r.spec.small_vehicle_needs_wheel_shape =
        wheels_tpl.radius < kSmallVehicleWheelRadiusCm;
    r.ok = true;
    r.reason = QString(
        "Tire-swap fallback: analyzer's wheels were clustered "
        "(y_spread=%1 of %2; x_spread=%3 of %4). Re-derived 4 anchors from chassis bbox.")
        .arg(y_max - y_min, 0, 'f', 0).arg(chassisY, 0, 'f', 0)
        .arg(x_max - x_min, 0, 'f', 0).arg(chassisX, 0, 'f', 0);
    return r;
  }

  r.mode = IngestionMode::BodyPlusTires;
  deriveWheelAnchors(r.spec, wheels_tpl.radius, wheels_tpl.width, r.spec.wheels);
  r.spec.has_four_wheels = true;
  r.spec.small_vehicle_needs_wheel_shape =
      wheels_tpl.radius < kSmallVehicleWheelRadiusCm;
  r.ok = true;
  r.reason = QString("Body+Tires: derived 4 wheel anchors from chassis bbox; radius=%1cm.")
               .arg(wheels_tpl.radius, 0, 'f', 1);
  return r;
}

namespace {

struct ObjVertex { double x = 0, y = 0, z = 0; };

struct ObjGroup {
  QString name;
  int first_vertex = 0;
  int vertex_count = 0;
  double cx = 0, cy = 0, cz = 0;
};

struct ParsedObj {
  std::vector<ObjVertex>  vertices;
  std::vector<QString>    rawLines;
  std::vector<int>        vertexLineIndex;
  std::vector<ObjGroup>   groups;
  bool ok = false;
};

ParsedObj parseObjFile(const QString &path) {
  ParsedObj p;
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return p;
  QTextStream in(&f);
  ObjGroup currentGroup;
  bool haveGroup = false;
  int vertexIndex = 0;
  while (!in.atEnd()) {
    const QString line = in.readLine();
    p.rawLines.push_back(line);
    if (line.startsWith("v ")) {
      const QStringList parts = line.split(' ', Qt::SkipEmptyParts);
      if (parts.size() >= 4) {
        ObjVertex v;
        v.x = parts[1].toDouble();
        v.y = parts[2].toDouble();
        v.z = parts[3].toDouble();
        p.vertices.push_back(v);
        p.vertexLineIndex.push_back(static_cast<int>(p.rawLines.size()) - 1);
        ++currentGroup.vertex_count;
        ++vertexIndex;
      }
    } else if (line.startsWith("o ") || line.startsWith("g ")) {
      if (haveGroup && currentGroup.vertex_count > 0) {
        p.groups.push_back(currentGroup);
      }
      currentGroup = ObjGroup{};
      currentGroup.name = line.mid(2).trimmed();
      currentGroup.first_vertex = vertexIndex;
      haveGroup = true;
    }
  }
  if (haveGroup && currentGroup.vertex_count > 0) {
    p.groups.push_back(currentGroup);
  }
  for (ObjGroup &g : p.groups) {
    double sx = 0, sy = 0, sz = 0;
    const int end = g.first_vertex + g.vertex_count;
    for (int i = g.first_vertex; i < end && i < static_cast<int>(p.vertices.size()); ++i) {
      sx += p.vertices[static_cast<size_t>(i)].x;
      sy += p.vertices[static_cast<size_t>(i)].y;
      sz += p.vertices[static_cast<size_t>(i)].z;
    }
    if (g.vertex_count > 0) {
      g.cx = sx / g.vertex_count;
      g.cy = sy / g.vertex_count;
      g.cz = sz / g.vertex_count;
    }
  }
  p.ok = !p.vertices.empty();
  return p;
}

struct WheelCluster {
  double cx = 0, cy = 0, cz = 0;
  int first_vertex = 0;
  int vertex_count = 0;
  std::vector<int> groupIndices;
};

std::vector<WheelCluster> clusterTireGroups(const ParsedObj &tires) {
  std::vector<WheelCluster> clusters;
  if (tires.groups.empty()) {
    if (!tires.vertices.empty()) {
      WheelCluster c;
      c.first_vertex = 0;
      c.vertex_count = static_cast<int>(tires.vertices.size());
      double sx = 0, sy = 0, sz = 0;
      for (const ObjVertex &v : tires.vertices) { sx += v.x; sy += v.y; sz += v.z; }
      c.cx = sx / c.vertex_count; c.cy = sy / c.vertex_count; c.cz = sz / c.vertex_count;
      clusters.push_back(c);
    }
    return clusters;
  }
  double bbMin[3] = { std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max() };
  double bbMax[3] = { -std::numeric_limits<double>::max(),
                      -std::numeric_limits<double>::max(),
                      -std::numeric_limits<double>::max() };
  for (const ObjVertex &v : tires.vertices) {
    bbMin[0] = std::min(bbMin[0], v.x); bbMax[0] = std::max(bbMax[0], v.x);
    bbMin[1] = std::min(bbMin[1], v.y); bbMax[1] = std::max(bbMax[1], v.y);
    bbMin[2] = std::min(bbMin[2], v.z); bbMax[2] = std::max(bbMax[2], v.z);
  }
  const double extent = std::max({ bbMax[0] - bbMin[0],
                                   bbMax[1] - bbMin[1],
                                   bbMax[2] - bbMin[2] });
  if (extent < 150.0) {
    WheelCluster c;
    c.first_vertex = 0;
    c.vertex_count = static_cast<int>(tires.vertices.size());
    double sx = 0, sy = 0, sz = 0;
    for (const ObjVertex &v : tires.vertices) { sx += v.x; sy += v.y; sz += v.z; }
    if (c.vertex_count > 0) {
      c.cx = sx / c.vertex_count; c.cy = sy / c.vertex_count; c.cz = sz / c.vertex_count;
    }
    c.groupIndices.reserve(tires.groups.size());
    for (size_t gi = 0; gi < tires.groups.size(); ++gi) c.groupIndices.push_back(static_cast<int>(gi));
    clusters.push_back(c);
    return clusters;
  }
  const double mergeRadius = std::max(0.01, extent * 0.25);
  std::vector<int> groupCluster(tires.groups.size(), -1);
  for (size_t gi = 0; gi < tires.groups.size(); ++gi) {
    if (groupCluster[gi] >= 0) continue;
    const int cIdx = static_cast<int>(clusters.size());
    WheelCluster c;
    c.groupIndices.push_back(static_cast<int>(gi));
    groupCluster[gi] = cIdx;
    for (size_t gj = gi + 1; gj < tires.groups.size(); ++gj) {
      if (groupCluster[gj] >= 0) continue;
      const double dx = tires.groups[gi].cx - tires.groups[gj].cx;
      const double dy = tires.groups[gi].cy - tires.groups[gj].cy;
      const double dz = tires.groups[gi].cz - tires.groups[gj].cz;
      if (std::sqrt(dx*dx + dy*dy + dz*dz) < mergeRadius) {
        groupCluster[gj] = cIdx;
        c.groupIndices.push_back(static_cast<int>(gj));
      }
    }
    clusters.push_back(c);
  }
  for (WheelCluster &c : clusters) {
    int firstV = std::numeric_limits<int>::max();
    int totalV = 0;
    double sx = 0, sy = 0, sz = 0;
    for (int gi : c.groupIndices) {
      const ObjGroup &g = tires.groups[static_cast<size_t>(gi)];
      firstV = std::min(firstV, g.first_vertex);
      totalV += g.vertex_count;
      const int end = g.first_vertex + g.vertex_count;
      for (int i = g.first_vertex; i < end && i < static_cast<int>(tires.vertices.size()); ++i) {
        sx += tires.vertices[static_cast<size_t>(i)].x;
        sy += tires.vertices[static_cast<size_t>(i)].y;
        sz += tires.vertices[static_cast<size_t>(i)].z;
      }
    }
    c.first_vertex = (firstV == std::numeric_limits<int>::max()) ? 0 : firstV;
    c.vertex_count = totalV;
    if (totalV > 0) { c.cx = sx / totalV; c.cy = sy / totalV; c.cz = sz / totalV; }
  }
  return clusters;
}

void writeBodyLinesScaledTranslated(QTextStream &out, const ParsedObj &body,
                                    double sx, double sy, double sz,
                                    double tx, double ty, double tz) {
  for (const QString &line : body.rawLines) {
    if (line.startsWith(QStringLiteral("v "))
        || line.startsWith(QStringLiteral("v\t"))) {
      const QStringList toks = line.split(QRegularExpression("\\s+"),
                                          Qt::SkipEmptyParts);
      if (toks.size() >= 4) {
        bool ox=false, oy=false, oz=false;
        const double x = toks[1].toDouble(&ox);
        const double y = toks[2].toDouble(&oy);
        const double z = toks[3].toDouble(&oz);
        if (ox && oy && oz) {
          out << "v " << QString::number(x * sx - tx, 'f', 6)
              << " "  << QString::number(y * sy - ty, 'f', 6)
              << " "  << QString::number(z * sz - tz, 'f', 6);
          for (int i = 4; i < toks.size(); ++i) out << " " << toks[i];
          out << "\n";
          continue;
        }
      }
    }
    out << line << "\n";
  }
}

void writeTranslatedTireCopy(QTextStream &out,
                             const ParsedObj &tires,
                             double tx, double ty, double tz,
                             const QString &copyName,
                             int globalVertOffset,
                             double sx = 1.0, double sy = 1.0,
                             double sz = 1.0,
                             int permX = 0, int permY = 1,
                             int permZ = 2,
                             bool mirrorAxial = false) {
  out << "o " << copyName << "\n";

  const double m_ax = mirrorAxial ? -1.0 : 1.0;
  for (const ObjVertex &v : tires.vertices) {
    const double comp[3] = { v.x * sx * m_ax, v.y * sy, v.z * sz };
    out << "v " << QString::number(comp[permX] + tx, 'f', 6)
        << " "  << QString::number(comp[permY] + ty, 'f', 6)
        << " "  << QString::number(comp[permZ] + tz, 'f', 6) << "\n";
  }

  for (const QString &line : tires.rawLines) {
    if (!line.startsWith(QStringLiteral("f "))
        && !line.startsWith(QStringLiteral("f\t"))) continue;
    const QStringList toks = line.split(QRegularExpression("\\s+"),
                                        Qt::SkipEmptyParts);
    if (toks.size() < 4) continue;
    out << "f";
    bool any = false;
    for (int i = 1; i < toks.size(); ++i) {
      const QString head = toks[i].section('/', 0, 0);
      bool ok = false;
      int   idx = head.toInt(&ok);
      if (!ok) { any = false; break; }
      const int srcN = static_cast<int>(tires.vertices.size());
      if (idx < 0) idx = srcN + idx + 1;
      const int newIdx = idx + globalVertOffset;
      out << " " << newIdx;
      any = true;
    }
    out << "\n";
    (void)any;
  }
}

}

IpgHeader parse_ipg_header(const QString &obj_path) {
  IpgHeader h;
  QFile f(obj_path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return h;
  QTextStream in(&f);
  int linesScanned = 0;
  while (!in.atEnd() && linesScanned < 200) {
    const QString line = in.readLine();
    ++linesScanned;
    if (line.startsWith(QStringLiteral("v ")) || line.startsWith(QStringLiteral("v\t"))
        || line.startsWith(QStringLiteral("f ")) || line.startsWith(QStringLiteral("f\t"))) {
      break;
    }
    if (line.startsWith(QStringLiteral("# Scale"))) {
      const QStringList toks = line.split(QRegularExpression("\\s+"),
                                          Qt::SkipEmptyParts);
      if (toks.size() >= 5) {
        bool ox=false, oy=false, oz=false;
        const double x = toks[2].toDouble(&ox);
        const double y = toks[3].toDouble(&oy);
        const double z = toks[4].toDouble(&oz);
        if (ox && oy && oz && x != 0 && y != 0 && z != 0) {
          h.sx = x; h.sy = y; h.sz = z; h.has_scale = true;
        }
      }
      continue;
    }
    if (line.startsWith(QStringLiteral("# Wheel."))) {
      const QStringList toks = line.split(QRegularExpression("\\s+"),
                                          Qt::SkipEmptyParts);
      if (toks.size() >= 5) {
        const QString tag = toks[1].toLower();
        int idx = -1;
        if      (tag.startsWith("wheel.fl")) idx = 0;
        else if (tag.startsWith("wheel.fr")) idx = 1;
        else if (tag.startsWith("wheel.rl")) idx = 2;
        else if (tag.startsWith("wheel.rr")) idx = 3;
        if (idx >= 0) {
          bool ox=false, oy=false, oz=false;
          const double x = toks[2].toDouble(&ox);
          const double y = toks[3].toDouble(&oy);
          const double z = toks[4].toDouble(&oz);
          if (ox && oy && oz) {
            h.wheels[static_cast<size_t>(idx*3 + 0)] = x;
            h.wheels[static_cast<size_t>(idx*3 + 1)] = y;
            h.wheels[static_cast<size_t>(idx*3 + 2)] = z;
            h.has_wheels = true;
          }
        }
      }
    }
  }
  return h;
}

ObjMergeResult merge_body_and_tires(const QString &body_obj_path,
                                 const QString &tires_obj_path,
                                 const QString &out_dir,
                                 const std::array<WheelSpec, 4> &wheel_anchors) {
  ObjMergeResult r;
  if (!QFileInfo(body_obj_path).isFile()) {
    r.reason = "Body OBJ not found.";
    return r;
  }
  if (!QFileInfo(tires_obj_path).isFile()) {
    r.reason = "Tires OBJ not found.";
    return r;
  }
  const ParsedObj body = parseObjFile(body_obj_path);
  if (!body.ok) { r.reason = "Failed to parse body OBJ."; return r; }
  const ParsedObj tires = parseObjFile(tires_obj_path);
  if (!tires.ok) { r.reason = "Failed to parse tires OBJ."; return r; }
  r.body_vertex_count = static_cast<int>(body.vertices.size());
  r.tire_vertex_count = static_cast<int>(tires.vertices.size());

  const IpgHeader body_ipg  = parse_ipg_header(body_obj_path);
  const IpgHeader tire_ipg  = parse_ipg_header(tires_obj_path);
  const double bsx = body_ipg.has_scale ? body_ipg.sx : 1.0;
  const double bsy = body_ipg.has_scale ? body_ipg.sy : 1.0;
  const double bsz = body_ipg.has_scale ? body_ipg.sz : 1.0;
  const double tsx = tire_ipg.has_scale ? tire_ipg.sx : 1.0;
  const double tsy = tire_ipg.has_scale ? tire_ipg.sy : 1.0;
  const double tsz = tire_ipg.has_scale ? tire_ipg.sz : 1.0;

  const std::vector<WheelCluster> clusters = clusterTireGroups(tires);
  r.tire_source_count = static_cast<int>(clusters.size());

  QDir().mkpath(out_dir);
  const QString stem = QFileInfo(body_obj_path).completeBaseName();
  const QString outPath = QDir(out_dir).filePath(stem + "_merged.obj");
  QFile out(outPath);
  if (!out.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
    r.reason = "Could not open output OBJ for writing: " + outPath;
    return r;
  }

  double bxMin = 1e300, bxMax = -1e300;
  double byMin = 1e300, byMax = -1e300;
  double bzMin = 1e300, bzMax = -1e300;
  for (const ObjVertex &v : body.vertices) {
    const double mx = v.x * bsx, my = v.y * bsy, mz = v.z * bsz;
    if (mx < bxMin) bxMin = mx;
    if (mx > bxMax) bxMax = mx;
    if (my < byMin) byMin = my;
    if (my > byMax) byMax = my;
    if (mz < bzMin) bzMin = mz;
    if (mz > bzMax) bzMax = mz;
  }
  const double bcx = 0.5 * (bxMin + bxMax);
  const double bcy = 0.5 * (byMin + byMax);
  const double bcz = bzMin;

  QTextStream os(&out);
  writeBodyLinesScaledTranslated(os, body, bsx, bsy, bsz, bcx, bcy, bcz);

  const double bextX = bxMax - bxMin;
  const double bextY = byMax - byMin;
  const double bextZ = bzMax - bzMin;
  int bFwdAxis = 0;
  if      (bextX >= bextY && bextX >= bextZ) bFwdAxis = 0;
  else if (bextY >= bextX && bextY >= bextZ) bFwdAxis = 1;
  else                                       bFwdAxis = 2;
  int bUpAxis = 2;
  if      (bextZ <= bextX && bextZ <= bextY) bUpAxis = 2;
  else if (bextY <= bextX && bextY <= bextZ) bUpAxis = 1;
  else                                       bUpAxis = 0;
  if (bUpAxis == bFwdAxis)
    bUpAxis = (bFwdAxis == 0) ? 1 : (bFwdAxis == 1 ? 2 : 0);
  const int bLatAxis = 3 - bFwdAxis - bUpAxis;

  int tire_perm[3];
  tire_perm[bLatAxis] = 0;
  tire_perm[bFwdAxis] = 1;
  tire_perm[bUpAxis]  = 2;

  const char *kCopyNames[4] = { "studio_tire_FL", "studio_tire_FR",
                                "studio_tire_RL", "studio_tire_RR" };

  double anchorM[4][3];
  for (size_t i = 0; i < 4; ++i) {
    anchorM[i][0] = static_cast<double>(wheel_anchors[i].x) * bsx;
    anchorM[i][1] = static_cast<double>(wheel_anchors[i].y) * bsy;
    anchorM[i][2] = static_cast<double>(wheel_anchors[i].z) * bsz;
  }

  int vertOffset = static_cast<int>(body.vertices.size());
  if (clusters.size() == 1) {
    const WheelCluster &src = clusters.front();

    const double centM[3] = { src.cx * tsx, src.cy * tsy, src.cz * tsz };
    for (size_t i = 0; i < 4; ++i) {
      const double tx = (anchorM[i][0] - bcx) - centM[tire_perm[0]];
      const double ty = (anchorM[i][1] - bcy) - centM[tire_perm[1]];
      const double tz = (anchorM[i][2] - bcz) - centM[tire_perm[2]];

      const bool isRight = (i == 1) || (i == 3);
      writeTranslatedTireCopy(os, tires, tx, ty, tz, kCopyNames[i],
                              vertOffset, tsx, tsy, tsz,
                              tire_perm[0], tire_perm[1], tire_perm[2],
                              isRight);
      vertOffset += static_cast<int>(tires.vertices.size());
      ++r.tire_copies_placed;
    }
  } else {
    std::array<int, 4> assignment{ -1, -1, -1, -1 };
    std::vector<bool> used(clusters.size(), false);
    for (size_t i = 0; i < 4; ++i) {
      double bestD = std::numeric_limits<double>::max();
      int bestC = -1;
      for (size_t c = 0; c < clusters.size(); ++c) {
        if (used[c]) continue;
        const double dx = clusters[c].cx * tsx - anchorM[i][0];
        const double dy = clusters[c].cy * tsy - anchorM[i][1];
        const double dz = clusters[c].cz * tsz - anchorM[i][2];
        const double d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < bestD) { bestD = d2; bestC = static_cast<int>(c); }
      }
      assignment[i] = bestC;
      if (bestC >= 0) used[static_cast<size_t>(bestC)] = true;
    }
    for (size_t i = 0; i < 4; ++i) {
      const int cIdxI = assignment[i] >= 0 ? assignment[i] : static_cast<int>(i % clusters.size());
      const WheelCluster &src = clusters[static_cast<size_t>(cIdxI)];
      const double centM[3] = { src.cx * tsx, src.cy * tsy, src.cz * tsz };
      const double tx = (anchorM[i][0] - bcx) - centM[tire_perm[0]];
      const double ty = (anchorM[i][1] - bcy) - centM[tire_perm[1]];
      const double tz = (anchorM[i][2] - bcz) - centM[tire_perm[2]];
      ParsedObj subset;
      subset.vertices.reserve(static_cast<size_t>(src.vertex_count));
      const int end = src.first_vertex + src.vertex_count;
      for (int v = src.first_vertex; v < end && v < static_cast<int>(tires.vertices.size()); ++v) {
        subset.vertices.push_back(tires.vertices[static_cast<size_t>(v)]);
      }
      const bool isRight = (i == 1) || (i == 3);
      writeTranslatedTireCopy(os, subset, tx, ty, tz, kCopyNames[i],
                              vertOffset, tsx, tsy, tsz,
                              tire_perm[0], tire_perm[1], tire_perm[2],
                              isRight);
      vertOffset += static_cast<int>(subset.vertices.size());
      ++r.tire_copies_placed;
    }
  }

  out.close();
  r.merged_vertex_count = r.body_vertex_count +
      (clusters.size() == 1 ? r.tire_vertex_count * 4
                            : [&]() {
                                int total = 0;
                                for (int i = 0; i < 4; ++i) {
                                  const int cIdx = i < static_cast<int>(clusters.size()) ? i : 0;
                                  total += clusters[static_cast<size_t>(cIdx)].vertex_count;
                                }
                                return total;
                              }());
  r.output_path = outPath;
  r.ok = true;
  r.reason = QString("Merged: %1 body verts + %2 tire copies (%3 source tire(s)) → %4 total.")
               .arg(r.body_vertex_count).arg(r.tire_copies_placed).arg(r.tire_source_count).arg(r.merged_vertex_count);
  return r;
}

}
