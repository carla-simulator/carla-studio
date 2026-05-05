// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_import/ObjSanitizer.h"

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
                float &xMin, float &xMax,
                float &yMin, float &yMax,
                float &zMin, float &zMax) {
  QFile in(path);
  if (!in.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
  QTextStream s(&in);
  xMin = yMin = zMin = std::numeric_limits<float>::infinity();
  xMax = yMax = zMax = -std::numeric_limits<float>::infinity();
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
    xMin = std::min(xMin, x); xMax = std::max(xMax, x);
    yMin = std::min(yMin, y); yMax = std::max(yMax, y);
    zMin = std::min(zMin, z); zMax = std::max(zMax, z);
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

}  // namespace

SanitizeReport sanitizeOBJ(const QString &inputPath, float scaleToCm) {
  SanitizeReport rep;
  QFile in(inputPath);
  if (!in.open(QIODevice::ReadOnly | QIODevice::Text)) return rep;

  rep.outputPath = QDir::tempPath() + "/" +
                   QFileInfo(inputPath).completeBaseName() + "_sanitized.obj";

  float xMin, xMax, yMin, yMax, zMin, zMax;
  if (!scanBounds(inputPath, xMin, xMax, yMin, yMax, zMin, zMax)) return rep;

  bool autoMode = !(scaleToCm > 0.0f);
  float scale = autoMode ? 1.0f : scaleToCm;
  bool swapXY = false;
  float zLift = 0.0f;

  if (autoMode) {
    const float maxAbs = std::max({std::fabs(xMin), std::fabs(xMax),
                                   std::fabs(yMin), std::fabs(yMax),
                                   std::fabs(zMin), std::fabs(zMax)});
    if (maxAbs > 50.0f)        scale = 1.0f;
    else if (maxAbs >= 0.5f)   scale = 100.0f;
    else if (maxAbs > 0.0f)    scale = 250.0f / maxAbs;

    const float rngX = (xMax - xMin) * scale;
    const float rngY = (yMax - yMin) * scale;
    swapXY = rngY > rngX;
    zLift = -zMin * scale;
  }

  in.close();
  if (!in.open(QIODevice::ReadOnly | QIODevice::Text)) return rep;
  QFile out(rep.outputPath);
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
          if (swapXY) std::swap(x, y);
          sOut << "v " << x << ' ' << y << ' ' << z << '\n';
          continue;
        }
      }
    }
    if (line.startsWith("f ") || line.startsWith("f\t")) {
      const QStringList tok = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
      if (tok.size() < 4) { ++rep.skippedFaceLines; continue; }
    }
    sOut << line << '\n';
  }

  colocateMaterials(QFileInfo(inputPath).absolutePath(),
                    QFileInfo(rep.outputPath).absolutePath());

  rep.ok = true;
  rep.appliedScale = scale;
  rep.swapXY       = swapXY;
  rep.zLiftCm      = zLift;
  return rep;
}

}  // namespace carla_studio::vehicle_import
