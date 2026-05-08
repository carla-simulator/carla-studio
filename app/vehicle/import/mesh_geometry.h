// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QString>
#include <vector>

namespace carla_studio::vehicle_import {

struct MeshGeometry {
  bool valid = false;
  std::vector<float> verts;
  std::vector<int>   faces;
  int  malformedFaces = 0;

  int  vertexCount() const { return static_cast<int>(verts.size() / 3); }
  int  faceCount()   const { return static_cast<int>(faces.size() / 3); }
  void vertex(int i, float &x, float &y, float &z) const {
    const int o = i * 3;
    x = verts[o]; y = verts[o + 1]; z = verts[o + 2];
  }
};

MeshGeometry loadMeshGeometry(const QString &path);
MeshGeometry loadMeshGeometryOBJ(const QString &path);

}  // namespace carla_studio::vehicle_import
