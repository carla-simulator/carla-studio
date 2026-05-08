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
  Top,
  Side,
  Front
};

struct PreviewWheels {
  bool   set = false;
  QVector3D fl, fr, rl, rr;
};

struct PreviewSummary {
  bool   valid = false;
  int    vert_count = 0;
  int    face_count = 0;
  int    malformed_faces = 0;
  float  scale_to_cm = 1.0f;
  QString units_hint;
  float  ext_cm[3] = {0,0,0};
  float  volume_m3 = 0.0f;
  QString size_class;
  int    forward_axis = 0;
  int    up_axis      = 2;
  int    forward_sign = +1;
  long   verts_ahead = 0;
  long   verts_behind = 0;
};

PreviewSummary analyse_mesh(const MeshGeometry &g);

QImage render_preview(const MeshGeometry &g,
                     const PreviewSummary &sum,
                     PreviewView view,
                     int pixels = 640,
                     const PreviewWheels &wheels = {});

}
