// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/mesh_aabb.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QRegularExpression>
#include <QTextStream>
#include <QXmlStreamReader>
#include <algorithm>
#include <cmath>

namespace carla_studio::vehicle_import {

void MeshAABB::feed(float x, float y, float z) {
  x_min = std::min(x_min, x); x_max = std::max(x_max, x);
  y_min = std::min(y_min, y); y_max = std::max(y_max, y);
  z_min = std::min(z_min, z); z_max = std::max(z_max, z);
  valid = true;
}

void MeshAABB::detect_conventions(const QString &ext) {
  if (!valid) return;
  const float dx = x_max - x_min, dy = y_max - y_min, dz = z_max - z_min;
  const float maxDim = std::max({dx, dy, dz});
  scale_to_cm = (ext == "gltf" || ext == "glb" || maxDim < 20.f) ? 100.f : 1.f;

  const float mins[3] = { x_min, y_min, z_min };
  const float dims[3] = { dx,   dy,   dz   };
  forward_axis = (dx >= dy && dx >= dz) ? 0 : (dy >= dz ? 1 : 2);

  int best = -1;
  float bestAbs = 1e9f;
  for (int i = 0; i < 3; ++i) {
    if (i == forward_axis) continue;
    if (std::abs(mins[i]) < bestAbs) { bestAbs = std::abs(mins[i]); best = i; }
  }
  up_axis = (best >= 0) ? best : 2;

  if (ext == "obj" || ext == "gltf" || ext == "glb") {
    if (dims[1] < dims[0] && dims[1] < dims[2]) up_axis = 1;
  }
}

void MeshAABB::to_ue(float &xLo, float &xHi,
                    float &yLo, float &yHi,
                    float &zLo, float &zHi) const {
  const float lo[3] = { x_min, y_min, z_min };
  const float hi[3] = { x_max, y_max, z_max };
  const int rightAxis = 3 - up_axis - forward_axis;
  xLo = lo[forward_axis] * scale_to_cm; xHi = hi[forward_axis] * scale_to_cm;
  yLo = lo[rightAxis]   * scale_to_cm; yHi = hi[rightAxis]   * scale_to_cm;
  zLo = lo[up_axis]      * scale_to_cm; zHi = hi[up_axis]      * scale_to_cm;
}

MeshAABB parse_obj(const QString &path) {
  MeshAABB bb;
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return bb;
  QTextStream in(&f);
  static const QRegularExpression kWs("\\s+");
  while (!in.atEnd()) {
    const QString line = in.readLine().trimmed();
    if (line.startsWith(QLatin1String("v ")) || line.startsWith(QLatin1String("v\t"))) {
      const QStringList tok = line.split(kWs, Qt::SkipEmptyParts);
      if (tok.size() >= 4) {
        bool ok1, ok2, ok3;
        const float x = tok[1].toFloat(&ok1);
        const float y = tok[2].toFloat(&ok2);
        const float z = tok[3].toFloat(&ok3);
        if (ok1 && ok2 && ok3) {
          bb.feed(x, y, z);
          ++bb.vertex_count;
        }
      }
    } else if (line.startsWith(QLatin1String("vn ")) || line.startsWith(QLatin1String("vn\t"))) {
      ++bb.normals_count;
    } else if (line.startsWith(QLatin1String("vt ")) || line.startsWith(QLatin1String("vt\t"))) {
      bb.has_uvs = true;
    } else if (line.startsWith(QLatin1String("f ")) || line.startsWith(QLatin1String("f\t"))) {
      const QStringList tok = line.split(kWs, Qt::SkipEmptyParts);
      if (tok.size() >= 4) ++bb.face_count;
      else                 ++bb.malformed_face_lines;
    } else if (line.startsWith(QLatin1String("mtllib"))) {
      ++bb.mtl_refs;
    }
  }
  return bb;
}

MeshAABB parse_gltf(const QString &path) {
  MeshAABB bb;
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) return bb;
  QByteArray raw = f.readAll();

  if (raw.size() >= 12 &&
      raw[0] == 'g' && raw[1] == 'l' && raw[2] == 'T' && raw[3] == 'F') {
    if (raw.size() < 20) return bb;
    const quint32 chunkLen = static_cast<quint32>((quint8)raw[12])
                           | (static_cast<quint32>((quint8)raw[13]) << 8)
                           | (static_cast<quint32>((quint8)raw[14]) << 16)
                           | (static_cast<quint32>((quint8)raw[15]) << 24);
    raw = raw.mid(20, chunkLen);
  }

  QJsonParseError err;
  QJsonDocument doc = QJsonDocument::fromJson(raw, &err);
  if (err.error != QJsonParseError::NoError) return bb;

  const QJsonArray accessors = doc.object().value("accessors").toArray();
  for (const QJsonValue &av : accessors) {
    const QJsonObject ao = av.toObject();
    if (ao.value("type").toString() != QLatin1String("VEC3")) continue;
    if (!ao.contains("min") || !ao.contains("max")) continue;
    const QJsonArray mn = ao.value("min").toArray();
    const QJsonArray mx = ao.value("max").toArray();
    if (mn.size() < 3 || mx.size() < 3) continue;
    bb.feed((float)mn[0].toDouble(), (float)mn[1].toDouble(), (float)mn[2].toDouble());
    bb.feed((float)mx[0].toDouble(), (float)mx[1].toDouble(), (float)mx[2].toDouble());
  }
  return bb;
}

MeshAABB parse_dae(const QString &path) {
  MeshAABB bb;
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return bb;
  QXmlStreamReader xml(&f);
  bool insidePositionSource = false;
  while (!xml.atEnd()) {
    xml.readNext();
    if (xml.isStartElement()) {
      const QString name = xml.name().toString().toLower();
      if (name == QLatin1String("source")) {
        const QString id = xml.attributes().value("id").toString().toLower();
        insidePositionSource = id.contains("position") || id.contains("pos");
      } else if (name == QLatin1String("float_array") && insidePositionSource) {
        const QString text = xml.readElementText();
        const QStringList vals = text.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        for (int i = 0; i + 2 < vals.size(); i += 3) {
          bool ok1, ok2, ok3;
          const float x = vals[i  ].toFloat(&ok1);
          const float y = vals[i+1].toFloat(&ok2);
          const float z = vals[i+2].toFloat(&ok3);
          if (ok1 && ok2 && ok3) bb.feed(x, y, z);
        }
      }
    } else if (xml.isEndElement()) {
      if (xml.name().toString().toLower() == QLatin1String("source"))
        insidePositionSource = false;
    }
  }
  return bb;
}

}
