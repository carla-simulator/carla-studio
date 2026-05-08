// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_import/MeshAnalysis.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace carla_studio::vehicle_import {

namespace {

constexpr float kSmallVehicleWheelRadiusCm = 18.0f;

struct VertBox {
  float xMin, xMax, yMin, yMax, zMin, zMax;
  void feed(float x, float y, float z) {
    xMin = std::min(xMin, x); xMax = std::max(xMax, x);
    yMin = std::min(yMin, y); yMax = std::max(yMax, y);
    zMin = std::min(zMin, z); zMax = std::max(zMax, z);
  }
  static VertBox empty() { return {1e9f, -1e9f, 1e9f, -1e9f, 1e9f, -1e9f}; }
  float dx() const { return xMax - xMin; }
  float dy() const { return yMax - yMin; }
  float dz() const { return zMax - zMin; }
  float cx() const { return 0.5f * (xMin + xMax); }
  float cy() const { return 0.5f * (yMin + yMax); }
  float cz() const { return 0.5f * (zMin + zMax); }
};

std::vector<int> componentLabels(const MeshGeometry &g) {
  const int vc = g.vertexCount();
  std::vector<std::vector<int>> adj(vc);
  const int fc = g.faceCount();
  for (int i = 0; i < fc; ++i) {
    const int a = g.faces[i * 3 + 0];
    const int b = g.faces[i * 3 + 1];
    const int c = g.faces[i * 3 + 2];
    adj[a].push_back(b); adj[a].push_back(c);
    adj[b].push_back(a); adj[b].push_back(c);
    adj[c].push_back(a); adj[c].push_back(b);
  }
  std::vector<int> label(vc, -1);
  int next = 0;
  for (int seed = 0; seed < vc; ++seed) {
    if (label[seed] != -1) continue;
    label[seed] = next;
    std::queue<int> q;
    q.push(seed);
    while (!q.empty()) {
      const int v = q.front(); q.pop();
      for (int n : adj[v]) {
        if (label[n] == -1) { label[n] = next; q.push(n); }
      }
    }
    ++next;
  }
  return label;
}

SizeClass classifySize(float chassisLengthCm) {
  if (chassisLengthCm <= 0) return SizeClass::Unknown;
  if (chassisLengthCm < 120.0f)  return SizeClass::TinyRobot;
  if (chassisLengthCm < 380.0f)  return SizeClass::Compact;
  if (chassisLengthCm < 510.0f)  return SizeClass::Sedan;
  if (chassisLengthCm < 580.0f)  return SizeClass::Suv;
  if (chassisLengthCm < 800.0f)  return SizeClass::Truck;
  return SizeClass::Bus;
}

}  // namespace

QString sizeClassName(SizeClass s) {
  switch (s) {
    case SizeClass::TinyRobot: return QStringLiteral("tiny-robot");
    case SizeClass::Compact:   return QStringLiteral("compact");
    case SizeClass::Sedan:     return QStringLiteral("sedan");
    case SizeClass::Suv:       return QStringLiteral("suv");
    case SizeClass::Truck:     return QStringLiteral("truck");
    case SizeClass::Bus:       return QStringLiteral("bus");
    default:                   return QStringLiteral("unknown");
  }
}

MeshAnalysisResult analyzeMesh(const MeshGeometry &g, float scaleToCm) {
  MeshAnalysisResult r;
  if (!g.valid || g.faceCount() == 0) return r;

  const int vc = g.vertexCount();
  VertBox overall = VertBox::empty();
  for (int i = 0; i < vc; ++i) {
    float x, y, z; g.vertex(i, x, y, z);
    overall.feed(x * scaleToCm, y * scaleToCm, z * scaleToCm);
  }
  r.overallXMin = overall.xMin; r.overallXMax = overall.xMax;
  r.overallYMin = overall.yMin; r.overallYMax = overall.yMax;
  r.overallZMin = overall.zMin; r.overallZMax = overall.zMax;

  const std::vector<int> label = componentLabels(g);
  int numClusters = 0;
  for (int l : label) numClusters = std::max(numClusters, l + 1);
  std::vector<VertBox> bb(numClusters, VertBox::empty());
  std::vector<int>     count(numClusters, 0);
  for (int i = 0; i < vc; ++i) {
    float x, y, z; g.vertex(i, x, y, z);
    const int l = label[i];
    bb[l].feed(x * scaleToCm, y * scaleToCm, z * scaleToCm);
    ++count[l];
  }
  r.clusters.resize(numClusters);
  for (int i = 0; i < numClusters; ++i) {
    Cluster &c = r.clusters[i];
    c.vertexCount = count[i];
    c.xMin = bb[i].xMin; c.xMax = bb[i].xMax;
    c.yMin = bb[i].yMin; c.yMax = bb[i].yMax;
    c.zMin = bb[i].zMin; c.zMax = bb[i].zMax;
  }

  const float oxLen = overall.dx();
  const float oyLen = overall.dy();
  const float ozLen = overall.dz();
  const float zFloor = overall.zMin;
  const float zBand  = ozLen * 0.55f;

  struct Scored { int idx; float score, cx, cy, cz, radius, width; };
  std::vector<Scored> scored;
  for (int i = 0; i < numClusters; ++i) {
    const VertBox &b = bb[i];
    if (count[i] < 12) continue;
    if (b.zMin > zFloor + zBand) continue;
    const float dx = b.dx(), dy = b.dy(), dz = b.dz();
    const float r1 = std::max({dx, dy, dz}) * 0.5f;
    const float r0 = std::min({dx, dy, dz}) * 0.5f;
    const float aspect = (r1 > 0) ? r0 / r1 : 0.0f;
    if (aspect < 0.30f) continue;
    if (b.dx() > oxLen * 0.55f) continue;
    if (b.dy() > oyLen * 0.55f) continue;
    Scored s;
    s.idx = i;
    s.cx = b.cx(); s.cy = b.cy(); s.cz = b.cz();
    s.radius = std::max(b.dx(), b.dz()) * 0.5f;
    s.width  = b.dy();
    const float lateralBias = std::abs(b.cy()) / std::max(1.0f, oyLen * 0.5f);
    const float floorBias   = 1.0f - (b.zMin - zFloor) / std::max(1.0f, zBand);
    s.score = lateralBias * 0.6f + floorBias * 0.4f + aspect * 0.2f;
    scored.push_back(s);
  }

  std::sort(scored.begin(), scored.end(), [](const Scored &a, const Scored &b) {
    return a.score > b.score;
  });

  std::vector<WheelCandidate> picked;
  for (const Scored &s : scored) {
    bool tooClose = false;
    for (const WheelCandidate &p : picked) {
      const float ddx = s.cx - p.cx, ddy = s.cy - p.cy;
      if (std::sqrt(ddx * ddx + ddy * ddy) < std::max(s.radius, p.radius) * 1.2f) {
        tooClose = true; break;
      }
    }
    if (tooClose) continue;
    WheelCandidate w;
    w.cx = s.cx; w.cy = s.cy; w.cz = s.cz;
    w.radius = s.radius; w.width = s.width;
    w.clusterId = s.idx;
    picked.push_back(w);
    if (picked.size() == 4) break;
  }

  std::sort(picked.begin(), picked.end(), [](const WheelCandidate &a, const WheelCandidate &b) {
    if (std::abs(a.cx - b.cx) > 1.0f) return a.cx > b.cx;
    return a.cy > b.cy;
  });
  r.wheels = picked;
  r.hasFourWheels = picked.size() == 4;

  std::vector<bool> isWheelCluster(numClusters, false);
  for (const WheelCandidate &w : picked)
    if (w.clusterId >= 0 && w.clusterId < numClusters) isWheelCluster[w.clusterId] = true;
  VertBox chassis = VertBox::empty();
  bool anyChassis = false;
  for (int i = 0; i < numClusters; ++i) {
    if (isWheelCluster[i]) continue;
    if (count[i] < 8) continue;
    chassis.feed(bb[i].xMin, bb[i].yMin, bb[i].zMin);
    chassis.feed(bb[i].xMax, bb[i].yMax, bb[i].zMax);
    anyChassis = true;
  }
  if (!anyChassis) {
    r.chassisXMin = overall.xMin; r.chassisXMax = overall.xMax;
    r.chassisYMin = overall.yMin; r.chassisYMax = overall.yMax;
    r.chassisZMin = overall.zMin; r.chassisZMax = overall.zMax;
  } else {
    r.chassisXMin = chassis.xMin; r.chassisXMax = chassis.xMax;
    r.chassisYMin = chassis.yMin; r.chassisYMax = chassis.yMax;
    r.chassisZMin = chassis.zMin; r.chassisZMax = chassis.zMax;
  }

  const float chassisLength = std::abs(r.chassisXMax - r.chassisXMin);
  r.sizeClass = classifySize(chassisLength);

  if (r.hasFourWheels) {
    float minR = 1e9f;
    for (const WheelCandidate &w : picked) minR = std::min(minR, w.radius);
    r.smallVehicleNeedsWheelShape = minR < kSmallVehicleWheelRadiusCm;
  }

  r.ok = true;
  return r;
}

}  // namespace carla_studio::vehicle_import
