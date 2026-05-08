// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_import/MergedSpecBuilder.h"

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
  for (int i = 0; i < 4; ++i) {
    out[i].x = xs[i];
    out[i].y = ysSigned[i];
    out[i].z = zMin + radius;
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
    r.reason = "Body has no wheels and no wheels file was provided — vehicle would not be drivable.";
    return r;
  }

  if (bodySpec.hasFourWheels && wheelsFileProvided) {
    r.mode = IngestionMode::TireSwap;
    for (WheelSpec &w : r.spec.wheels) {
      w.radius = wheelsTpl.radius;
      w.width  = wheelsTpl.width;
      w.z      = bodySpec.chassisZMin + wheelsTpl.radius;
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

void writeTranslatedTireCopy(QTextStream &out,
                             const ParsedObj &tires,
                             double tx, double ty, double tz,
                             const QString &copyName) {
  out << "o " << copyName << "\n";
  for (const ObjVertex &v : tires.vertices) {
    out << "v " << QString::number(v.x + tx, 'f', 6)
        << " "  << QString::number(v.y + ty, 'f', 6)
        << " "  << QString::number(v.z + tz, 'f', 6) << "\n";
  }
}

}  // namespace

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
  QTextStream os(&out);
  writeBodyLines(os, body);

  const char *kCopyNames[4] = { "studio_tire_FL", "studio_tire_FR",
                                "studio_tire_RL", "studio_tire_RR" };

  if (clusters.size() == 1) {
    const WheelCluster &src = clusters.front();
    for (int i = 0; i < 4; ++i) {
      const double tx = wheelAnchors[i].x - src.cx;
      const double ty = wheelAnchors[i].y - src.cy;
      const double tz = wheelAnchors[i].z - src.cz;
      writeTranslatedTireCopy(os, tires, tx, ty, tz, kCopyNames[i]);
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
        const double dx = clusters[c].cx - wheelAnchors[i].x;
        const double dy = clusters[c].cy - wheelAnchors[i].y;
        const double dz = clusters[c].cz - wheelAnchors[i].z;
        const double d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < bestD) { bestD = d2; bestC = static_cast<int>(c); }
      }
      assignment[i] = bestC;
      if (bestC >= 0) used[bestC] = true;
    }
    for (int i = 0; i < 4; ++i) {
      const int cIdx = assignment[i] >= 0 ? assignment[i] : (i % static_cast<int>(clusters.size()));
      const WheelCluster &src = clusters[cIdx];
      const double tx = wheelAnchors[i].x - src.cx;
      const double ty = wheelAnchors[i].y - src.cy;
      const double tz = wheelAnchors[i].z - src.cz;
      ParsedObj subset;
      subset.vertices.reserve(src.vertexCount);
      const int end = src.firstVertex + src.vertexCount;
      for (int v = src.firstVertex; v < end && v < static_cast<int>(tires.vertices.size()); ++v) {
        subset.vertices.push_back(tires.vertices[v]);
      }
      writeTranslatedTireCopy(os, subset, tx, ty, tz, kCopyNames[i]);
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
