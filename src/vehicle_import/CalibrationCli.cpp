// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later
//

#include "MeshPreviewRenderer.h"
#include "MeshGeometry.h"

#include <QGuiApplication>
#include <QDir>
#include <QFileInfo>
#include <QImage>
#include <QString>
#include <QStringList>
#include <QTextStream>

#include <cstdio>

using namespace carla_studio::vehicle_import;

static int dieUsage()
{
  fprintf(stderr,
          "usage: carla-studio-vehicle-preview <mesh-path> [<out-dir>] [--scale F]\n"
          "       Renders top/side/front PNGs on a 1m calibration grid and\n"
          "       prints a JSON summary (orientation, class, dimensions).\n"
          "       --scale F  multiply detected scale_to_cm by F before rendering\n");
  return 2;
}

int main(int argc, char **argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QGuiApplication app(argc, argv);
  if (argc < 2) return dieUsage();
  const QString meshPath = QString::fromLocal8Bit(argv[1]);
  const QString baseName = QFileInfo(meshPath).completeBaseName();
  QString outDir = QString("/tmp/vehicle_preview/") + baseName;
  float scaleOverride = 1.0f;
  for (int i = 2; i < argc; ++i) {
    const QString a = QString::fromLocal8Bit(argv[i]);
    if (a == "--scale" && i + 1 < argc) { scaleOverride = QString(argv[++i]).toFloat(); }
    else if (!a.startsWith("--")) { outDir = a; }
  }
  QDir().mkpath(outDir);

  MeshGeometry g = loadMeshGeometry(meshPath);
  if (!g.valid) {
    fprintf(stderr, "load failed: %s\n", argv[1]);
    return 3;
  }
  PreviewSummary sum = analyseMesh(g);
  if (!sum.valid) {
    fprintf(stderr, "analyse failed: empty mesh\n");
    return 4;
  }
  if (scaleOverride != 1.0f) {
    sum.scaleToCm *= scaleOverride;
    for (int k = 0; k < 3; ++k) sum.extCm[k] *= scaleOverride;
    sum.volumeM3 *= scaleOverride * scaleOverride * scaleOverride;
    sum.unitsHint += QString(" *%1").arg(scaleOverride);
  }

  const struct { PreviewView v; const char *name; } views[] = {
    { PreviewView::Top,   "top"   },
    { PreviewView::Side,  "side"  },
    { PreviewView::Front, "front" }
  };
  QStringList pngs;
  for (auto &it : views) {
    QImage img = renderPreview(g, sum, it.v, 720);
    const QString out = QDir(outDir).filePath(QString("%1.png").arg(it.name));
    if (!img.save(out, "PNG")) {
      fprintf(stderr, "save failed: %s\n", qPrintable(out));
      return 5;
    }
    pngs << out;
  }

  const char fwdAxisName = (sum.forwardAxis == 0) ? 'X'
                          : (sum.forwardAxis == 1) ? 'Y' : 'Z';

  QString summaryPath = QDir(outDir).filePath("summary.json");
  QTextStream out(stdout);
  QString json;
  QTextStream js(&json);
  js << "{\n";
  js << "  \"name\": \""        << baseName << "\",\n";
  js << "  \"source\": \""      << meshPath << "\",\n";
  js << "  \"verts\": "         << sum.vertCount << ",\n";
  js << "  \"faces\": "         << sum.faceCount << ",\n";
  js << "  \"malformed_faces\": " << sum.malformedFaces << ",\n";
  js << "  \"units_detected\": \"" << sum.unitsHint << "\",\n";
  js << "  \"scale_to_cm\": "   << sum.scaleToCm << ",\n";
  js << "  \"extents_cm\": ["   << sum.extCm[0] << ", "
                                << sum.extCm[1] << ", "
                                << sum.extCm[2] << "],\n";
  js << "  \"extents_m\": ["    << sum.extCm[0]/100.0 << ", "
                                << sum.extCm[1]/100.0 << ", "
                                << sum.extCm[2]/100.0 << "],\n";
  js << "  \"volume_m3\": "     << sum.volumeM3 << ",\n";
  js << "  \"size_class\": \""  << sum.sizeClass << "\",\n";
  js << "  \"forward_axis\": \"" << fwdAxisName << "\",\n";
  js << "  \"forward_sign\": "  << sum.forwardSign << ",\n";
  js << "  \"verts_ahead\": "   << sum.vertsAhead << ",\n";
  js << "  \"verts_behind\": "  << sum.vertsBehind << ",\n";
  js << "  \"needs_180_flip\": " << (sum.forwardSign < 0 ? "true" : "false") << ",\n";
  js << "  \"png_top\": \""     << pngs[0] << "\",\n";
  js << "  \"png_side\": \""    << pngs[1] << "\",\n";
  js << "  \"png_front\": \""   << pngs[2] << "\",\n";
  js << "  \"out_dir\": \""     << outDir << "\"\n";
  js << "}\n";

  out << json;
  out.flush();

  QFile f(summaryPath);
  if (f.open(QIODevice::WriteOnly)) { f.write(json.toUtf8()); f.close(); }
  return 0;
}
