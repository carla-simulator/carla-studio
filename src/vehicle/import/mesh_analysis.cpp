// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/mesh_analysis.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace carla_studio::vehicle_import {

namespace {

constexpr float kSmallVehicleWheelRadiusCm = 18.0f;

struct VertBox {
  float x_min, x_max, y_min, y_max, z_min, z_max;
  void feed(float x, float y, float z) {
    x_min = std::min(x_min, x); x_max = std::max(x_max, x);
    y_min = std::min(y_min, y); y_max = std::max(y_max, y);
    z_min = std::min(z_min, z); z_max = std::max(z_max, z);
  }
  static VertBox empty() { return {1e9f, -1e9f, 1e9f, -1e9f, 1e9f, -1e9f}; }
  float dx() const { return x_max - x_min; }
  float dy() const { return y_max - y_min; }
  float dz() const { return z_max - z_min; }
  float cx() const { return 0.5f * (x_min + x_max); }
  float cy() const { return 0.5f * (y_min + y_max); }
  float cz() const { return 0.5f * (z_min + z_max); }
};

std::vector<int> componentLabels(const MeshGeometry &g) {
  const size_t vc = static_cast<size_t>(g.vertex_count());
  std::vector<std::vector<int>> adj(vc);
  const size_t fc = static_cast<size_t>(g.face_count());
  for (size_t i = 0; i < fc; ++i) {
    const int a = g.faces[i * 3 + 0];
    const int b = g.faces[i * 3 + 1];
    const int c = g.faces[i * 3 + 2];
    const size_t sa = static_cast<size_t>(a);
    const size_t sb = static_cast<size_t>(b);
    const size_t sc = static_cast<size_t>(c);
    adj[sa].push_back(b); adj[sa].push_back(c);
    adj[sb].push_back(a); adj[sb].push_back(c);
    adj[sc].push_back(a); adj[sc].push_back(b);
  }
  std::vector<int> label(vc, -1);
  int next = 0;
  for (size_t seed = 0; seed < vc; ++seed) {
    if (label[seed] != -1) continue;
    label[seed] = next;
    std::queue<size_t> q;
    q.push(seed);
    while (!q.empty()) {
      const size_t v = q.front(); q.pop();
      for (int n : adj[v]) {
        const size_t sn = static_cast<size_t>(n);
        if (label[sn] == -1) { label[sn] = next; q.push(sn); }
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

}

SizeClass classify_by_name(const QString &stem) {
  const QString lower = stem.toLower();
  struct Hint { const char *kw; SizeClass cls; };
  static const Hint hints[] = {
    {"robot",  SizeClass::TinyRobot}, {"cobot", SizeClass::TinyRobot},
    {"agv",    SizeClass::TinyRobot},
    {"truck",  SizeClass::Truck},     {"lorry", SizeClass::Truck},
    {"bus",    SizeClass::Bus},
    {"suv",    SizeClass::Suv},
    {"sedan",  SizeClass::Sedan},
  };
  for (const auto &h : hints)
    if (lower.contains(QString::fromLatin1(h.kw))) return h.cls;
  return SizeClass::Unknown;
}

QString size_class_name(SizeClass s) {
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

float detect_scale_to_cm_from_extent(float maxExt) {
  if (maxExt <= 0.f) return 1.0f;
  if (maxExt >= 2.0f   && maxExt <= 8.0f)    return 100.0f;
  if (maxExt >= 200.0f && maxExt <= 800.0f)   return 1.0f;
  if (maxExt >= 2000.0f && maxExt <= 8000.0f) return 0.1f;
  if (maxExt >= 0.3f   && maxExt < 2.0f)     return 100.0f;
  return 450.0f / maxExt;
}

float detect_scale_to_cm(const MeshGeometry &g) {
  if (!g.valid || g.vertex_count() == 0) return 1.0f;
  float xn = 1e30f, xx = -1e30f;
  float yn = 1e30f, yx = -1e30f;
  float zn = 1e30f, zx = -1e30f;
  const int vc = g.vertex_count();
  for (int i = 0; i < vc; ++i) {
    float x, y, z; g.vertex(i, x, y, z);
    xn = std::min(xn, x); xx = std::max(xx, x);
    yn = std::min(yn, y); yx = std::max(yx, y);
    zn = std::min(zn, z); zx = std::max(zx, z);
  }
  return detect_scale_to_cm_from_extent(std::max({xx - xn, yx - yn, zx - zn}));
}

MeshAnalysisResult analyze_mesh(const MeshGeometry &g, float scale_to_cm) {
  MeshAnalysisResult r;
  if (!g.valid || g.face_count() == 0) return r;

  const int vc = g.vertex_count();
  VertBox overall = VertBox::empty();
  for (int i = 0; i < vc; ++i) {
    float x, y, z; g.vertex(i, x, y, z);
    overall.feed(x * scale_to_cm, y * scale_to_cm, z * scale_to_cm);
  }
  r.overall_x_min = overall.x_min; r.overall_x_max = overall.x_max;
  r.overall_y_min = overall.y_min; r.overall_y_max = overall.y_max;
  r.overall_z_min = overall.z_min; r.overall_z_max = overall.z_max;

  const std::vector<int> label = componentLabels(g);
  int numClusters = 0;
  for (int l : label) numClusters = std::max(numClusters, l + 1);
  const auto nc = static_cast<size_t>(numClusters);
  std::vector<VertBox> bb(nc, VertBox::empty());
  std::vector<int>     count(nc, 0);
  for (int i = 0; i < vc; ++i) {
    float x, y, z; g.vertex(i, x, y, z);
    const size_t l = static_cast<size_t>(label[static_cast<size_t>(i)]);
    bb[l].feed(x * scale_to_cm, y * scale_to_cm, z * scale_to_cm);
    ++count[l];
  }
  r.clusters.resize(nc);
  for (size_t i = 0; i < nc; ++i) {
    Cluster &c = r.clusters[i];
    c.vertex_count = count[i];
    c.x_min = bb[i].x_min; c.x_max = bb[i].x_max;
    c.y_min = bb[i].y_min; c.y_max = bb[i].y_max;
    c.z_min = bb[i].z_min; c.z_max = bb[i].z_max;
  }

  const float oxLen = overall.dx();
  const float oyLen = overall.dy();
  const float zFloor = overall.z_min;

  struct Scored { int idx; float score, cx, cy, cz, radius, width; };
  std::vector<Scored> scored;
  for (size_t i = 0; i < nc; ++i) {
    const VertBox &b = bb[i];
    if (count[i] < 12) continue;
    const float dx = b.dx(), dy = b.dy(), dz = b.dz();
    const float approxRadius = std::max(dx, dz) * 0.5f;
    if (b.z_min > zFloor + approxRadius * 0.5f) continue;
    const float r1 = std::max({dx, dy, dz}) * 0.5f;
    const float r0 = std::min({dx, dy, dz}) * 0.5f;
    const float aspect = (r1 > 0) ? r0 / r1 : 0.0f;
    if (aspect < 0.30f) continue;
    if (b.dx() > oxLen * 0.55f) continue;
    if (b.dy() > oyLen * 0.55f) continue;
    Scored s;
    s.idx = static_cast<int>(i);
    s.cx = b.cx(); s.cy = b.cy(); s.cz = b.cz();
    s.radius = approxRadius;
    s.width  = b.dy();
    const float lateralBias = std::abs(b.cy()) / std::max(1.0f, oyLen * 0.5f);
    const float floorBias   = 1.0f - (b.z_min - zFloor) / std::max(approxRadius, 1.0f);
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
    w.cluster_id = s.idx;
    picked.push_back(w);
    if (picked.size() == 4) break;
  }

  std::sort(picked.begin(), picked.end(), [](const WheelCandidate &a, const WheelCandidate &b) {
    if (std::abs(a.cx - b.cx) > 1.0f) return a.cx > b.cx;
    return a.cy > b.cy;
  });
  r.wheels = picked;
  r.has_four_wheels = picked.size() == 4;

  std::vector<bool> isWheelCluster(nc, false);
  for (const WheelCandidate &w : picked)
    if (w.cluster_id >= 0 && w.cluster_id < numClusters) isWheelCluster[static_cast<size_t>(w.cluster_id)] = true;
  VertBox chassis = VertBox::empty();
  bool anyChassis = false;
  for (size_t i = 0; i < nc; ++i) {
    if (isWheelCluster[i]) continue;
    if (count[i] < 8) continue;
    chassis.feed(bb[i].x_min, bb[i].y_min, bb[i].z_min);
    chassis.feed(bb[i].x_max, bb[i].y_max, bb[i].z_max);
    anyChassis = true;
  }
  if (!anyChassis) {
    r.chassis_x_min = overall.x_min; r.chassis_x_max = overall.x_max;
    r.chassis_y_min = overall.y_min; r.chassis_y_max = overall.y_max;
    r.chassis_z_min = overall.z_min; r.chassis_z_max = overall.z_max;
  } else {
    r.chassis_x_min = chassis.x_min; r.chassis_x_max = chassis.x_max;
    r.chassis_y_min = chassis.y_min; r.chassis_y_max = chassis.y_max;
    r.chassis_z_min = chassis.z_min; r.chassis_z_max = chassis.z_max;
  }

  const float chassisLength = std::abs(r.chassis_x_max - r.chassis_x_min);
  r.size_class = classifySize(chassisLength);

  if (r.has_four_wheels) {
    float minR = 1e9f;
    for (const WheelCandidate &w : picked) minR = std::min(minR, w.radius);
    r.small_vehicle_needs_wheel_shape = minR < kSmallVehicleWheelRadiusCm;
  }

  r.ok = true;
  return r;
}

}
