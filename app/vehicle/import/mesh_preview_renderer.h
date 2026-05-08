// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QImage>
#include <QString>
#include <QVector3D>

namespace carla_studio::vehicle_import {

struct MeshGeometry;

enum class PreviewView {
  Top,    // XY plane, +X right, +Y up
  Side,   // XZ plane, +X right, +Z up
  Front   // YZ plane, +Y right, +Z up
};

struct PreviewWheels {
  bool   set = false;
  QVector3D fl, fr, rl, rr;
};

struct PreviewSummary {
  bool   valid = false;
  int    vertCount = 0;
  int    faceCount = 0;
  int    malformedFaces = 0;
  float  scaleToCm = 1.0f;
  QString unitsHint;
  float  extCm[3] = {0,0,0};
  float  volumeM3 = 0.0f;
  QString sizeClass;
  int    forwardAxis = 0;          // 0=X, 1=Y, 2=Z
  int    upAxis      = 2;          // 0=X, 1=Y, 2=Z
  int    forwardSign = +1;
  long   vertsAhead = 0;
  long   vertsBehind = 0;
};

PreviewSummary analyseMesh(const MeshGeometry &g);

QImage renderPreview(const MeshGeometry &g,
                     const PreviewSummary &sum,
                     PreviewView view,
                     int pixels = 640,
                     const PreviewWheels &wheels = {});

}  // namespace carla_studio::vehicle_import
