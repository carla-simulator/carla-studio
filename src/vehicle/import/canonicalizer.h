// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include "vehicle/import/mesh_geometry.h"

#include <QString>

namespace carla_studio::vehicle_import {

struct CanonicalizeResult {
  bool    ok = false;
  QString output_path;
  QString detail;
  bool    did_rescale         = false;
  bool    did_axis_remap       = false;
  bool    did_origin_recenter  = false;
  bool    did_smoothing_groups = false;
  bool    did_ascii_to_binary   = false;
  bool    did_bone_rename      = false;
};

CanonicalizeResult canonicalize_mesh(const QString &input_path,
                                    const QString &output_path);

bool assimp_available();

MeshGeometry load_mesh_geometry_any(const QString &path);

}
