// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/obj_sanitizer.h"

#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <limits>

namespace carla_studio::vehicle_import {

namespace {

bool scanBounds(const QString &path,
                float &x_min, float &x_max,
                float &y_min, float &y_max,
                float &z_min, float &z_max) {
  QFile in(path);
  if (!in.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
  QTextStream s(&in);
  x_min = y_min = z_min = std::numeric_limits<float>::infinity();
  x_max = y_max = z_max = -std::numeric_limits<float>::infinity();
  bool any = false;
  while (!s.atEnd()) {
    const QString line = s.readLine();
    if (!(line.startsWith("v ") || line.startsWith("v\t"))) continue;
    const QStringList tok = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
    if (tok.size() < 4) continue;
    bool ok[3];
    const float x = tok[1].toFloat(&ok[0]);
    const float y = tok[2].toFloat(&ok[1]);
    const float z = tok[3].toFloat(&ok[2]);
    if (!(ok[0] && ok[1] && ok[2])) continue;
    any = true;
    x_min = std::min(x_min, x); x_max = std::max(x_max, x);
    y_min = std::min(y_min, y); y_max = std::max(y_max, y);
    z_min = std::min(z_min, z); z_max = std::max(z_max, z);
  }
  return any;
}

void colocateMaterials(const QString &srcDir, const QString &dstDir) {
  static const QStringList kFilters = {
    "*.mtl", "*.png", "*.jpg", "*.jpeg", "*.tga", "*.bmp", "*.dds"
  };
  QDirIterator it(srcDir, kFilters, QDir::Files, QDirIterator::NoIteratorFlags);
  while (it.hasNext()) {
    const QString src = it.next();
    const QString dst = dstDir + "/" + QFileInfo(src).fileName();
    if (QFile::exists(dst)) continue;
    QFile::copy(src, dst);
  }
}

}

SanitizeReport sanitize_obj(const QString &input_path, float scale_to_cm) {
  SanitizeReport rep;
  QFile in(input_path);
  if (!in.open(QIODevice::ReadOnly | QIODevice::Text)) return rep;

  rep.output_path = QDir::tempPath() + "/" +
                   QFileInfo(input_path).completeBaseName() + "_sanitized.obj";

  float x_min, x_max, y_min, y_max, z_min, z_max;
  if (!scanBounds(input_path, x_min, x_max, y_min, y_max, z_min, z_max)) return rep;

  bool autoMode = !(scale_to_cm > 0.0f);
  float scale = autoMode ? 1.0f : scale_to_cm;
  bool swap_xy = false;
  float zLift = 0.0f;

  if (autoMode) {
    const float maxAbs = std::max({std::fabs(x_min), std::fabs(x_max),
                                   std::fabs(y_min), std::fabs(y_max),
                                   std::fabs(z_min), std::fabs(z_max)});
    if (maxAbs > 50.0f)        scale = 1.0f;
    else if (maxAbs >= 0.5f)   scale = 100.0f;
    else if (maxAbs > 0.0f)    scale = 250.0f / maxAbs;

    const float rngX = (x_max - x_min) * scale;
    const float rngY = (y_max - y_min) * scale;
    swap_xy = rngY > rngX;
    zLift = -z_min * scale;
  }

  in.close();
  if (!in.open(QIODevice::ReadOnly | QIODevice::Text)) return rep;
  QFile out(rep.output_path);
  if (!out.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) return rep;

  QTextStream sIn(&in), sOut(&out);
  sOut.setRealNumberPrecision(7);

  while (!sIn.atEnd()) {
    const QString line = sIn.readLine();
    if (line.startsWith("v ") || line.startsWith("v\t")) {
      const QStringList tok = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
      if (tok.size() >= 4) {
        bool okx, oky, okz;
        const float x0 = tok[1].toFloat(&okx);
        const float y0 = tok[2].toFloat(&oky);
        const float z0 = tok[3].toFloat(&okz);
        if (okx && oky && okz) {
          float x = x0 * scale, y = y0 * scale, z = z0 * scale + zLift;
          if (swap_xy) std::swap(x, y);
          sOut << "v " << x << ' ' << y << ' ' << z << '\n';
          continue;
        }
      }
    }
    if (line.startsWith("f ") || line.startsWith("f\t")) {
      const QStringList tok = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
      if (tok.size() < 4) { ++rep.skipped_face_lines; continue; }
    }
    sOut << line << '\n';
  }

  colocateMaterials(QFileInfo(input_path).absolutePath(),
                    QFileInfo(rep.output_path).absolutePath());

  rep.ok = true;
  rep.applied_scale = scale;
  rep.swap_xy       = swap_xy;
  rep.z_lift_cm      = zLift;
  return rep;
}

}
