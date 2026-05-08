// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/mesh_geometry.h"

#include <QFile>
#include <QFileInfo>
#include <QTextStream>

namespace carla_studio::vehicle_import {

namespace {

int parseFaceIndex(const QString &token, int vertex_count) {
  const qsizetype slash = token.indexOf(QLatin1Char('/'));
  const QString head = slash < 0 ? token : token.left(slash);
  bool ok = false;
  int idx = head.toInt(&ok);
  if (!ok) return -1;
  if (idx < 0) idx = vertex_count + idx + 1;
  if (idx < 1 || idx > vertex_count) return -1;
  return idx - 1;
}

}

MeshGeometry load_mesh_geometry_obj(const QString &path) {
  MeshGeometry g;
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return g;
  QTextStream in(&f);
  while (!in.atEnd()) {
    const QString line = in.readLine().trimmed();
    if (line.isEmpty() || line.startsWith(QLatin1Char('#'))) continue;
    if (line.startsWith(QLatin1String("v "))) {
      const QStringList tokens = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
      if (tokens.size() < 4) continue;
      bool ok1, ok2, ok3;
      const float x = tokens[1].toFloat(&ok1);
      const float y = tokens[2].toFloat(&ok2);
      const float z = tokens[3].toFloat(&ok3);
      if (!(ok1 && ok2 && ok3)) continue;
      g.verts.push_back(x); g.verts.push_back(y); g.verts.push_back(z);
    } else if (line.startsWith(QLatin1String("f "))) {
      const QStringList tokens = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
      if (tokens.size() < 4) { ++g.malformed_faces; continue; }
      const int vc = g.vertex_count();
      std::vector<int> ring;
      ring.reserve(static_cast<size_t>(tokens.size() - 1));
      bool bad = false;
      for (int i = 1; i < tokens.size(); ++i) {
        const int idx = parseFaceIndex(tokens[i], vc);
        if (idx < 0) { bad = true; break; }
        ring.push_back(idx);
      }
      if (bad || ring.size() < 3) { ++g.malformed_faces; continue; }
      for (size_t i = 1; i + 1 < ring.size(); ++i) {
        g.faces.push_back(ring[0]);
        g.faces.push_back(ring[i]);
        g.faces.push_back(ring[i + 1]);
      }
    }
  }
  g.valid = !g.verts.empty() && !g.faces.empty();
  return g;
}

MeshGeometry load_mesh_geometry(const QString &path) {
  const QString ext = QFileInfo(path).suffix().toLower();
  if (ext == "obj") return load_mesh_geometry_obj(path);
  return MeshGeometry{};
}

}
