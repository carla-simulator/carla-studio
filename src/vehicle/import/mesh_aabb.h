// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QString>

namespace carla_studio::vehicle_import {

struct MeshAABB {
  float x_min =  1e9f, x_max = -1e9f;
  float y_min =  1e9f, y_max = -1e9f;
  float z_min =  1e9f, z_max = -1e9f;
  bool  valid = false;

  float scale_to_cm   = 1.0f;
  int   up_axis      = 2;
  int   forward_axis = 0;

  int  vertex_count        = 0;
  int  face_count          = 0;
  int  malformed_face_lines = 0;
  int  normals_count       = 0;
  int  mtl_refs            = 0;
  bool has_uvs             = false;

  void feed(float x, float y, float z);
  void detect_conventions(const QString &ext);
  void to_ue(float &xLo, float &xHi,
            float &yLo, float &yHi,
            float &zLo, float &zHi) const;
};

MeshAABB parse_obj (const QString &path);
MeshAABB parse_gltf(const QString &path);
MeshAABB parse_dae (const QString &path);

}
