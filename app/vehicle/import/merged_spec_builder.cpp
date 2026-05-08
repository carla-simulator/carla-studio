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

WheelTemplate wheelTemplateFromAnalysis(const MeshAnalysisResult &r) {
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

WheelTemplate wheelTemplateFromTiresObj(const QString &tiresObjPath,
                                        float scaleToCm) {
  WheelTemplate t;
  QFile f(tiresObjPath);
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
  const double dx = (bbMax[0] - bbMin[0]) * static_cast<double>(scaleToCm);
  const double dy = (bbMax[1] - bbMin[1]) * static_cast<double>(scaleToCm);
  const double dz = (bbMax[2] - bbMin[2]) * static_cast<double>(scaleToCm);
  t.radius = static_cast<float>(std::max(dx, dz) * 0.5);
  t.width  = static_cast<float>(dy);
  t.valid  = t.radius > 0;
  return t;
}

namespace {

constexpr float kSmallVehicleWheelRadiusCm = 18.0f;

void deriveWheelAnchors(const VehicleSpec &body, float radius, float width,
                        std::array<WheelSpec, 4> &out) {
  const float xMin = body.chassisXMin, xMax = body.chassisXMax;
  const float yMin = body.chassisYMin, yMax = body.chassisYMax;
  const float zMin = body.chassisZMin;
  const float wheelInsetX = (xMax - xMin) * 0.18f;
  const float wheelInsetY = std::max(2.0f, width * 0.5f);
  const std::array<float, 4> xs = { xMin + wheelInsetX, xMax - wheelInsetX,
                                    xMin + wheelInsetX, xMax - wheelInsetX };
  const std::array<float, 4> ys = { yMin - wheelInsetY * 0.0f, yMin - wheelInsetY * 0.0f,
                                    yMax + wheelInsetY * 0.0f, yMax + wheelInsetY * 0.0f };
  const std::array<float, 4> ysSigned = { yMin, yMin, yMax, yMax };
  const float wheelZ = std::max(radius, 1.0f);
  for (int i = 0; i < 4; ++i) {
    out[i].x = xs[i];
    out[i].y = ysSigned[i];
    out[i].z = wheelZ;
    out[i].radius = radius;
    out[i].width  = width;
  }
}

}  // namespace

MergeResult mergeBodyAndWheels(const VehicleSpec &bodySpec,
                               const WheelTemplate &wheelsTpl,
                               bool wheelsFileProvided) {
  MergeResult r;
  r.spec = bodySpec;
  if (!bodySpec.detectedFromMesh) {
    r.reason = "Body spec is not mesh-detected; merge requires analysis output.";
    return r;
  }

  if (bodySpec.hasFourWheels && !wheelsFileProvided) {
    r.mode = IngestionMode::Combined;
    r.ok = true;
    r.reason = "Combined: body already includes 4 wheels.";
    return r;
  }

  if (!wheelsTpl.valid && wheelsFileProvided) {
    r.reason = "Wheels file provided but template is invalid.";
    return r;
  }
  if (!wheelsTpl.valid && !bodySpec.hasFourWheels) {
    r.reason = "Body has no wheels and no wheels file was provided - vehicle would not be drivable.";
    return r;
  }

  if (bodySpec.hasFourWheels && wheelsFileProvided) {
    float yMin = std::numeric_limits<float>::max();
    float yMax = -std::numeric_limits<float>::max();
    float xMin = std::numeric_limits<float>::max();
    float xMax = -std::numeric_limits<float>::max();
    for (const WheelSpec &w : r.spec.wheels) {
      yMin = std::min(yMin, w.y); yMax = std::max(yMax, w.y);
      xMin = std::min(xMin, w.x); xMax = std::max(xMax, w.x);
    }
    const float chassisX = bodySpec.chassisXMax - bodySpec.chassisXMin;
    const float chassisY = bodySpec.chassisYMax - bodySpec.chassisYMin;
    const bool ySpreadOk = (yMax - yMin) >= chassisY * 0.40f;
    const bool xSpreadOk = (xMax - xMin) >= chassisX * 0.40f;
    const bool detectedSane = ySpreadOk && xSpreadOk;

    if (detectedSane) {
      r.mode = IngestionMode::TireSwap;
      for (WheelSpec &w : r.spec.wheels) {
        w.radius = wheelsTpl.radius;
        w.width  = wheelsTpl.width;
        w.z      = std::max(bodySpec.chassisZMin + wheelsTpl.radius,
                            wheelsTpl.radius * 1.05f);
      }
      r.spec.smallVehicleNeedsWheelShape =
          wheelsTpl.radius < kSmallVehicleWheelRadiusCm;
      r.ok = true;
      r.reason = QString("Tire-swap: replacing wheels with radius=%1cm, width=%2cm.")
                   .arg(wheelsTpl.radius, 0, 'f', 1).arg(wheelsTpl.width, 0, 'f', 1);
      return r;
    }
    r.mode = IngestionMode::BodyPlusTires;
    deriveWheelAnchors(r.spec, wheelsTpl.radius, wheelsTpl.width, r.spec.wheels);
    r.spec.smallVehicleNeedsWheelShape =
        wheelsTpl.radius < kSmallVehicleWheelRadiusCm;
    r.ok = true;
    r.reason = QString(
        "Tire-swap fallback: analyzer's wheels were clustered "
        "(y_spread=%1 of %2; x_spread=%3 of %4). Re-derived 4 anchors from chassis bbox.")
        .arg(yMax - yMin, 0, 'f', 0).arg(chassisY, 0, 'f', 0)
        .arg(xMax - xMin, 0, 'f', 0).arg(chassisX, 0, 'f', 0);
    return r;
  }

  r.mode = IngestionMode::BodyPlusTires;
  deriveWheelAnchors(r.spec, wheelsTpl.radius, wheelsTpl.width, r.spec.wheels);
  r.spec.hasFourWheels = true;
  r.spec.smallVehicleNeedsWheelShape =
      wheelsTpl.radius < kSmallVehicleWheelRadiusCm;
  r.ok = true;
  r.reason = QString("Body+Tires: derived 4 wheel anchors from chassis bbox; radius=%1cm.")
               .arg(wheelsTpl.radius, 0, 'f', 1);
  return r;
}

namespace {

struct ObjVertex { double x = 0, y = 0, z = 0; };

struct ObjGroup {
  QString name;
  int firstVertex = 0;
  int vertexCount = 0;
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
        ++currentGroup.vertexCount;
        ++vertexIndex;
      }
    } else if (line.startsWith("o ") || line.startsWith("g ")) {
      if (haveGroup && currentGroup.vertexCount > 0) {
        p.groups.push_back(currentGroup);
      }
      currentGroup = ObjGroup{};
      currentGroup.name = line.mid(2).trimmed();
      currentGroup.firstVertex = vertexIndex;
      haveGroup = true;
    }
  }
  if (haveGroup && currentGroup.vertexCount > 0) {
    p.groups.push_back(currentGroup);
  }
  for (ObjGroup &g : p.groups) {
    double sx = 0, sy = 0, sz = 0;
    const int end = g.firstVertex + g.vertexCount;
    for (int i = g.firstVertex; i < end && i < static_cast<int>(p.vertices.size()); ++i) {
      sx += p.vertices[i].x; sy += p.vertices[i].y; sz += p.vertices[i].z;
    }
    if (g.vertexCount > 0) {
      g.cx = sx / g.vertexCount;
      g.cy = sy / g.vertexCount;
      g.cz = sz / g.vertexCount;
    }
  }
  p.ok = !p.vertices.empty();
  return p;
}

struct WheelCluster {
  double cx = 0, cy = 0, cz = 0;
  int firstVertex = 0;
  int vertexCount = 0;
  std::vector<int> groupIndices;
};

std::vector<WheelCluster> clusterTireGroups(const ParsedObj &tires) {
  std::vector<WheelCluster> clusters;
  if (tires.groups.empty()) {
    if (!tires.vertices.empty()) {
      WheelCluster c;
      c.firstVertex = 0;
      c.vertexCount = static_cast<int>(tires.vertices.size());
      double sx = 0, sy = 0, sz = 0;
      for (const ObjVertex &v : tires.vertices) { sx += v.x; sy += v.y; sz += v.z; }
      c.cx = sx / c.vertexCount; c.cy = sy / c.vertexCount; c.cz = sz / c.vertexCount;
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
    c.firstVertex = 0;
    c.vertexCount = static_cast<int>(tires.vertices.size());
    double sx = 0, sy = 0, sz = 0;
    for (const ObjVertex &v : tires.vertices) { sx += v.x; sy += v.y; sz += v.z; }
    if (c.vertexCount > 0) {
      c.cx = sx / c.vertexCount; c.cy = sy / c.vertexCount; c.cz = sz / c.vertexCount;
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
      const ObjGroup &g = tires.groups[gi];
      firstV = std::min(firstV, g.firstVertex);
      totalV += g.vertexCount;
      const int end = g.firstVertex + g.vertexCount;
      for (int i = g.firstVertex; i < end && i < static_cast<int>(tires.vertices.size()); ++i) {
        sx += tires.vertices[i].x; sy += tires.vertices[i].y; sz += tires.vertices[i].z;
      }
    }
    c.firstVertex = (firstV == std::numeric_limits<int>::max()) ? 0 : firstV;
    c.vertexCount = totalV;
    if (totalV > 0) { c.cx = sx / totalV; c.cy = sy / totalV; c.cz = sz / totalV; }
  }
  return clusters;
}

void writeBodyLines(QTextStream &out, const ParsedObj &body) {
  for (const QString &line : body.rawLines) out << line << "\n";
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

// Translate every "v X Y Z" line in body.rawLines by (-tx, -ty, -tz) and
// write the result. Non-vertex lines (faces, groups, materials, comments)
// pass through unchanged. This centers the body on the calibration grid
// without losing face/material structure.
void writeBodyLinesTranslated(QTextStream &out, const ParsedObj &body,
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
          out << "v " << QString::number(x - tx, 'f', 6)
              << " "  << QString::number(y - ty, 'f', 6)
              << " "  << QString::number(z - tz, 'f', 6);
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
  // permX/Y/Z select which scaled tire-vert component maps to the output
  // X/Y/Z columns. Default identity = (0,1,2) keeps tire frame unchanged.
  // For host bodies whose lateral axis isn't X (e.g. rr.obj where X=fwd),
  // the tire's axial (raw X) must be permuted onto body's lat so wheels
  // spin around the right axis. Faces stay valid since vertex *count* and
  // ordering don't change - only the per-vertex component shuffle.
  //
  // mirrorAxial flips the tire's axial axis (raw X) so right-side wheels
  // (FR, RR) face outward instead of being a same-handed copy of the left
  // wheels - without this, rims and brake calipers visibly point the wrong
  // way on the passenger side.
  const double mAx = mirrorAxial ? -1.0 : 1.0;
  for (const ObjVertex &v : tires.vertices) {
    const double comp[3] = { v.x * sx * mAx, v.y * sy, v.z * sz };
    out << "v " << QString::number(comp[permX] + tx, 'f', 6)
        << " "  << QString::number(comp[permY] + ty, 'f', 6)
        << " "  << QString::number(comp[permZ] + tz, 'f', 6) << "\n";
  }
  // Re-emit tire faces with rebased vertex indices so the merged OBJ
  // actually has triangles for the tire copies - without this the merged
  // file has tire verts only (no faces), so calibration / Interchange
  // see no tire geometry.
  //
  // For each "f a b c ..." line in tires.rawLines, rewrite each token's
  // vertex index. OBJ faces support 1-based positive indices and negative
  // (relative-to-end) indices; we handle both. Texture/normal indices
  // ("a/b/c") are dropped since the tire copies don't carry per-copy UVs.
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
      if (idx < 0) idx = srcN + idx + 1;   // OBJ relative-to-end → 1-based
      const int newIdx = idx + globalVertOffset;
      out << " " << newIdx;
      any = true;
    }
    out << "\n";
    (void)any;
  }
}

}  // namespace

IpgHeader parseIpgHeader(const QString &objPath) {
  IpgHeader h;
  QFile f(objPath);
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
          h.sx = x; h.sy = y; h.sz = z; h.hasScale = true;
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
            h.wheels[idx*3 + 0] = x;
            h.wheels[idx*3 + 1] = y;
            h.wheels[idx*3 + 2] = z;
            h.hasWheels = true;
          }
        }
      }
    }
  }
  return h;
}

ObjMergeResult mergeBodyAndTires(const QString &bodyObjPath,
                                 const QString &tiresObjPath,
                                 const QString &outDir,
                                 const std::array<WheelSpec, 4> &wheelAnchors) {
  ObjMergeResult r;
  if (!QFileInfo(bodyObjPath).isFile()) {
    r.reason = "Body OBJ not found.";
    return r;
  }
  if (!QFileInfo(tiresObjPath).isFile()) {
    r.reason = "Tires OBJ not found.";
    return r;
  }
  const ParsedObj body = parseObjFile(bodyObjPath);
  if (!body.ok) { r.reason = "Failed to parse body OBJ."; return r; }
  const ParsedObj tires = parseObjFile(tiresObjPath);
  if (!tires.ok) { r.reason = "Failed to parse tires OBJ."; return r; }
  r.bodyVertexCount = static_cast<int>(body.vertices.size());
  r.tireVertexCount = static_cast<int>(tires.vertices.size());

  const IpgHeader bodyIpg  = parseIpgHeader(bodyObjPath);
  const IpgHeader tireIpg  = parseIpgHeader(tiresObjPath);
  const double bsx = bodyIpg.hasScale ? bodyIpg.sx : 1.0;
  const double bsy = bodyIpg.hasScale ? bodyIpg.sy : 1.0;
  const double bsz = bodyIpg.hasScale ? bodyIpg.sz : 1.0;
  const double tsx = tireIpg.hasScale ? tireIpg.sx : 1.0;
  const double tsy = tireIpg.hasScale ? tireIpg.sy : 1.0;
  const double tsz = tireIpg.hasScale ? tireIpg.sz : 1.0;

  const std::vector<WheelCluster> clusters = clusterTireGroups(tires);
  r.tireSourceCount = static_cast<int>(clusters.size());

  QDir().mkpath(outDir);
  const QString stem = QFileInfo(bodyObjPath).completeBaseName();
  const QString outPath = QDir(outDir).filePath(stem + "_merged.obj");
  QFile out(outPath);
  if (!out.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
    r.reason = "Could not open output OBJ for writing: " + outPath;
    return r;
  }
  // Compute body bbox so we can center the merged OBJ on origin (XY = bbox
  // center, Z = bbox bottom). Without this the merged mesh inherits whatever
  // origin offset the source OBJ had - causing the calibration view + the
  // cooked SkeletalMesh actor to spawn off-center.
  // Body bbox in METER (post-scale) space - IPG headers carry per-axis
  // scales that convert raw verts to meters. Without applying them, mkz
  // (raw cm × 0.01 = m) and tires (raw cm × 0.01 = m) merge consistently
  // by accident, but rr (raw × 90 = m) collapses to a 0.05-unit dot next
  // to a 22-unit-wide tire and the body vanishes. Compute centering in
  // meter space so the merged file lives in unified meter coords.
  double bxMin = 1e300, bxMax = -1e300;
  double byMin = 1e300, byMax = -1e300;
  double bzMin = 1e300, bzMax = -1e300;
  for (const ObjVertex &v : body.vertices) {
    const double mx = v.x * bsx, my = v.y * bsy, mz = v.z * bsz;
    if (mx < bxMin) bxMin = mx; if (mx > bxMax) bxMax = mx;
    if (my < byMin) byMin = my; if (my > byMax) byMax = my;
    if (mz < bzMin) bzMin = mz; if (mz > bzMax) bzMax = mz;
  }
  const double bcx = 0.5 * (bxMin + bxMax);
  const double bcy = 0.5 * (byMin + byMax);
  const double bcz = bzMin;  // ground-rest the body - chassis bottom = z=0

  QTextStream os(&out);
  writeBodyLinesScaledTranslated(os, body, bsx, bsy, bsz, bcx, bcy, bcz);

  // Detect body axes from METER-space extents so we can permute the tire
  // mesh's axial (raw X) to land on body's lateral. Without this, kits
  // with X=forward (rr.obj) get tires spinning around the forward axis -
  // visible in calibration as wheels facing outward like saw blades.
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
  // Tire raw frame: X=axial, Y/Z=radial. Permutation maps tire (X,Y,Z)
  // onto body's (lat, fwd, up) raw axes - so that after merge & remap,
  // tire's axial axis = canonical lateral.
  int tirePerm[3];
  tirePerm[bLatAxis] = 0;  // tire X (axial)  → body lat
  tirePerm[bFwdAxis] = 1;  // tire Y (radial) → body fwd
  tirePerm[bUpAxis]  = 2;  // tire Z (radial) → body up

  const char *kCopyNames[4] = { "studio_tire_FL", "studio_tire_FR",
                                "studio_tire_RL", "studio_tire_RR" };

  // Wheel-anchor positions in METER space. Anchors come in body's RAW
  // frame (whatever units the bbox-derivation used); multiply by body's
  // IPG scale so they live in the same coords as the merged body verts.
  double anchorM[4][3];
  for (int i = 0; i < 4; ++i) {
    anchorM[i][0] = wheelAnchors[i].x * bsx;
    anchorM[i][1] = wheelAnchors[i].y * bsy;
    anchorM[i][2] = wheelAnchors[i].z * bsz;
  }

  // Track running vertex count so each tire copy's faces reference the
  // right indices in the merged file. The body's verts come first.
  int vertOffset = static_cast<int>(body.vertices.size());
  if (clusters.size() == 1) {
    const WheelCluster &src = clusters.front();
    // Tire cluster centroid in METER space too (cluster is computed in raw
    // tire-OBJ coords; convert via tire IPG scale before subtracting from
    // anchor position). After permutation the centroid components shuffle
    // alongside the verts, so we index by tirePerm[outputAxis].
    const double centM[3] = { src.cx * tsx, src.cy * tsy, src.cz * tsz };
    for (int i = 0; i < 4; ++i) {
      const double tx = (anchorM[i][0] - bcx) - centM[tirePerm[0]];
      const double ty = (anchorM[i][1] - bcy) - centM[tirePerm[1]];
      const double tz = (anchorM[i][2] - bcz) - centM[tirePerm[2]];
      // Right-side tires (FR=1, RR=3) are mirrored across the body's
      // lateral axis so the rim's outward face points outward, not into
      // the wheel well. Tires.obj is a single LH wheel; we get a proper
      // RH wheel by negating the axial axis.
      const bool isRight = (i == 1) || (i == 3);
      writeTranslatedTireCopy(os, tires, tx, ty, tz, kCopyNames[i],
                              vertOffset, tsx, tsy, tsz,
                              tirePerm[0], tirePerm[1], tirePerm[2],
                              isRight);
      vertOffset += static_cast<int>(tires.vertices.size());
      ++r.tireCopiesPlaced;
    }
  } else {
    std::array<int, 4> assignment{ -1, -1, -1, -1 };
    std::vector<bool> used(clusters.size(), false);
    for (int i = 0; i < 4; ++i) {
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
      if (bestC >= 0) used[bestC] = true;
    }
    for (int i = 0; i < 4; ++i) {
      const int cIdx = assignment[i] >= 0 ? assignment[i] : (i % static_cast<int>(clusters.size()));
      const WheelCluster &src = clusters[cIdx];
      const double centM[3] = { src.cx * tsx, src.cy * tsy, src.cz * tsz };
      const double tx = (anchorM[i][0] - bcx) - centM[tirePerm[0]];
      const double ty = (anchorM[i][1] - bcy) - centM[tirePerm[1]];
      const double tz = (anchorM[i][2] - bcz) - centM[tirePerm[2]];
      ParsedObj subset;
      subset.vertices.reserve(src.vertexCount);
      const int end = src.firstVertex + src.vertexCount;
      for (int v = src.firstVertex; v < end && v < static_cast<int>(tires.vertices.size()); ++v) {
        subset.vertices.push_back(tires.vertices[v]);
      }
      const bool isRight = (i == 1) || (i == 3);
      writeTranslatedTireCopy(os, subset, tx, ty, tz, kCopyNames[i],
                              vertOffset, tsx, tsy, tsz,
                              tirePerm[0], tirePerm[1], tirePerm[2],
                              isRight);
      vertOffset += static_cast<int>(subset.vertices.size());
      ++r.tireCopiesPlaced;
    }
  }

  out.close();
  r.mergedVertexCount = r.bodyVertexCount +
      (clusters.size() == 1 ? r.tireVertexCount * 4
                            : [&]() {
                                int total = 0;
                                for (int i = 0; i < 4; ++i) {
                                  const int cIdx = i < static_cast<int>(clusters.size()) ? i : 0;
                                  total += clusters[cIdx].vertexCount;
                                }
                                return total;
                              }());
  r.outputPath = outPath;
  r.ok = true;
  r.reason = QString("Merged: %1 body verts + %2 tire copies (%3 source tire(s)) → %4 total.")
               .arg(r.bodyVertexCount).arg(r.tireCopiesPlaced).arg(r.tireSourceCount).arg(r.mergedVertexCount);
  return r;
}

}  // namespace carla_studio::vehicle_import
