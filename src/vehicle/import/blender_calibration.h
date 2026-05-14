// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QString>
#include <array>

namespace carla_studio::vehicle_import {

// Result of a Blender-based calibration render.
struct BlenderCalibrationResult {
  bool    ok   = false;
  QString top_png, side_png, front_png, perspective_png;
  QString log;   // blender stdout/stderr (truncated)
};

// Render calibration views (top / side / front / perspective) for a vehicle
// mesh using Blender in headless/batch mode. Wheel positions are in CARLA cm
// (X=lateral, Y=forward, Z=up). Returns false if blender is not found or
// rendering fails.
//
// Order of wheel_xyz_cm: FL x,y,z  FR x,y,z  RL x,y,z  RR x,y,z
BlenderCalibrationResult render_calibration_with_blender(
    const QString &mesh_path,
    const std::array<float, 12> &wheel_xyz_cm,
    const QString &out_dir);

// Convenience overload: no wheel positions (shows mesh only).
BlenderCalibrationResult render_calibration_with_blender(
    const QString &mesh_path,
    const QString &out_dir);

}
