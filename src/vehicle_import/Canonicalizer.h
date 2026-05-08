// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include "vehicle_import/MeshGeometry.h"

#include <QString>

namespace carla_studio::vehicle_import {

struct CanonicalizeResult {
  bool    ok = false;
  QString outputPath;
  QString detail;
  bool    didRescale         = false;
  bool    didAxisRemap       = false;
  bool    didOriginRecenter  = false;
  bool    didSmoothingGroups = false;
  bool    didAsciiToBinary   = false;
  bool    didBoneRename      = false;
};

CanonicalizeResult canonicalizeMesh(const QString &inputPath,
                                    const QString &outputPath);

bool assimpAvailable();

MeshGeometry loadMeshGeometryAny(const QString &path);

}  // namespace carla_studio::vehicle_import
