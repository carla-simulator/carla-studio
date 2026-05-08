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

struct SanitizeReport {
  QString output_path;
  int     skipped_face_lines = 0;
  bool    ok = false;
  float   applied_scale = 1.0f;
  bool    swap_xy       = false;
  float   z_lift_cm      = 0.0f;
};

SanitizeReport sanitize_obj(const QString &input_path, float scale_to_cm);

}
