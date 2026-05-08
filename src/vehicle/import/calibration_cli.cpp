// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later
//

#include "mesh_preview_renderer.h"
#include "mesh_geometry.h"
#include "vehicle_preview_window.h"

#include <QApplication>
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
          "usage: carla-studio-vehicle-preview <mesh-path> [<out-dir>] [--scale F] [--interactive]\n"
          "       Renders top/side/front PNGs on a 1m calibration grid and\n"
          "       prints a JSON summary (orientation, class, dimensions).\n"
          "       --scale F      multiply detected scale_to_cm by F before rendering\n"
          "       --interactive  open the Qt3D orbit-camera spin viewer; blocks until user closes\n");
  return 2;
}

int main(int argc, char **argv)
{
  bool interactive = false;
  for (int i = 1; i < argc; ++i) {
    const QString a = QString::fromLocal8Bit(argv[i]);
    if (a == "--interactive") interactive = true;
  }
  if (!interactive) qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  if (argc < 2) return dieUsage();
  const QString mesh_path = QString::fromLocal8Bit(argv[1]);
  const QString baseName = QFileInfo(mesh_path).completeBaseName();
  QString out_dir = QString("/tmp/vehicle_preview/") + baseName;
  float scaleOverride = 1.0f;



  QVector3D wheelFL, wheelFR, wheelRL, wheelRR;
  bool gotWheels = false;
  auto parseV3 = [](const QString &s, QVector3D &out) {
    const QStringList p = s.split(',');
    if (p.size() < 3) return false;
    out.setX(p[0].toFloat());
    out.setY(p[1].toFloat());
    out.setZ(p[2].toFloat());
    return true;
  };
  for (int i = 2; i < argc; ++i) {
    const QString a = QString::fromLocal8Bit(argv[i]);
    if (a == "--scale" && i + 1 < argc) { scaleOverride = QString(argv[++i]).toFloat(); }
    else if (a == "--interactive") {}
    else if (a == "--wheel-fl" && i + 1 < argc) {
      gotWheels |= parseV3(QString::fromLocal8Bit(argv[++i]), wheelFL);
    }
    else if (a == "--wheel-fr" && i + 1 < argc) {
      gotWheels |= parseV3(QString::fromLocal8Bit(argv[++i]), wheelFR);
    }
    else if (a == "--wheel-rl" && i + 1 < argc) {
      gotWheels |= parseV3(QString::fromLocal8Bit(argv[++i]), wheelRL);
    }
    else if (a == "--wheel-rr" && i + 1 < argc) {
      gotWheels |= parseV3(QString::fromLocal8Bit(argv[++i]), wheelRR);
    }
    else if (!a.startsWith("--")) { out_dir = a; }
  }
  QDir().mkpath(out_dir);

  if (interactive) {
    auto *win = VehiclePreviewWindow::instance();
    win->setWindowTitle(QString("Calibration - %1  (orbit + drag, close window when done)")
                          .arg(baseName));
    win->resize(1024, 720);
    if (gotWheels) {
      win->show_for(mesh_path, wheelFL, wheelFR, wheelRL, wheelRR);
    } else {
      win->show_for_mesh(mesh_path);
    }
    return app.exec();
  }

  MeshGeometry g = load_mesh_geometry(mesh_path);
  if (!g.valid) {
    fprintf(stderr, "load failed: %s\n", argv[1]);
    return 3;
  }
  PreviewSummary sum = analyse_mesh(g);
  if (!sum.valid) {
    fprintf(stderr, "analyse failed: empty mesh\n");
    return 4;
  }
  if (scaleOverride != 1.0f) {
    sum.scale_to_cm *= scaleOverride;
    for (int k = 0; k < 3; ++k) sum.ext_cm[k] *= scaleOverride;
    sum.volume_m3 *= scaleOverride * scaleOverride * scaleOverride;
    sum.units_hint += QString(" *%1").arg(scaleOverride);
  }

  const struct { PreviewView v; const char *name; } views[] = {
    { PreviewView::Top,   "top"   },
    { PreviewView::Side,  "side"  },
    { PreviewView::Front, "front" }
  };
  QStringList pngs;
  for (auto &it : views) {
    QImage img = render_preview(g, sum, it.v, 720);
    const QString out = QDir(out_dir).filePath(QString("%1.png").arg(it.name));
    if (!img.save(out, "PNG")) {
      fprintf(stderr, "save failed: %s\n", qPrintable(out));
      return 5;
    }
    pngs << out;
  }

  const char fwdAxisName = (sum.forward_axis == 0) ? 'X'
                          : (sum.forward_axis == 1) ? 'Y' : 'Z';

  QString summaryPath = QDir(out_dir).filePath("summary.json");
  QTextStream out(stdout);
  QString json;
  QTextStream js(&json);
  js << "{\n";
  js << "  \"name\": \""        << baseName << "\",\n";
  js << "  \"source\": \""      << mesh_path << "\",\n";
  js << "  \"verts\": "         << sum.vert_count << ",\n";
  js << "  \"faces\": "         << sum.face_count << ",\n";
  js << "  \"malformed_faces\": " << sum.malformed_faces << ",\n";
  js << "  \"units_detected\": \"" << sum.units_hint << "\",\n";
  js << "  \"scale_to_cm\": "   << sum.scale_to_cm << ",\n";
  js << "  \"extents_cm\": ["   << sum.ext_cm[0] << ", "
                                << sum.ext_cm[1] << ", "
                                << sum.ext_cm[2] << "],\n";
  js << "  \"extents_m\": ["    << sum.ext_cm[0]/100.0 << ", "
                                << sum.ext_cm[1]/100.0 << ", "
                                << sum.ext_cm[2]/100.0 << "],\n";
  js << "  \"volume_m3\": "     << sum.volume_m3 << ",\n";
  js << "  \"size_class\": \""  << sum.size_class << "\",\n";
  js << "  \"forward_axis\": \"" << fwdAxisName << "\",\n";
  js << "  \"forward_sign\": "  << sum.forward_sign << ",\n";
  js << "  \"verts_ahead\": "   << sum.verts_ahead << ",\n";
  js << "  \"verts_behind\": "  << sum.verts_behind << ",\n";
  js << "  \"needs_180_flip\": " << (sum.forward_sign < 0 ? "true" : "false") << ",\n";
  js << "  \"png_top\": \""     << pngs[0] << "\",\n";
  js << "  \"png_side\": \""    << pngs[1] << "\",\n";
  js << "  \"png_front\": \""   << pngs[2] << "\",\n";
  js << "  \"out_dir\": \""     << out_dir << "\"\n";
  js << "}\n";

  out << json;
  out.flush();

  QFile f(summaryPath);
  if (f.open(QIODevice::WriteOnly)) { f.write(json.toUtf8()); f.close(); }
  return 0;
}
