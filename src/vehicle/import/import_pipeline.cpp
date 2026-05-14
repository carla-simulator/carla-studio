// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/import_pipeline.h"

#include "vehicle/import/carla_spawn.h"
#include "vehicle/import/importer_client.h"
#include "vehicle/import/merged_spec_builder.h"
#include "vehicle/import/mesh_analysis.h"
#include "vehicle/import/mesh_geometry.h"
#include "vehicle/import/vehicle_preview_window.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDialog>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QRegularExpression>
#include <QString>
#include <QWidget>

namespace carla_studio::vehicle_import {

namespace {

void emitLog(const ImportPipelineCallbacks &cb, const QString &msg) {
  if (cb.log) cb.log(msg);
}

VehicleSpec buildSpecFromInput(const ImportPipelineInput &in,
                               const ImportPipelineCallbacks &cb) {
  VehicleSpec s;
  MeshGeometry g = load_mesh_geometry(in.mesh_path);
  if (!g.valid) {
    emitLog(cb, QString("[mesh-analysis] could not load geometry from %1 "
                        "(unsupported format or empty file); using fallback wheel positions.")
                  .arg(QFileInfo(in.mesh_path).fileName()));
  } else {
    const float scale_to_cm = detect_scale_to_cm(g);
    emitLog(cb, QString("[mesh-analysis] loaded %1 verts, %2 tris from %3  "
                        "unit→cm scale=×%4")
                  .arg(g.vertex_count()).arg(g.face_count())
                  .arg(QFileInfo(in.mesh_path).fileName())
                  .arg(scale_to_cm, 0, 'f', 2));
    const MeshAnalysisResult ar = analyze_mesh(g, scale_to_cm);
    if (ar.ok) {
      s = build_spec_from_analysis(ar, in.vehicle_name, in.mesh_path, scale_to_cm, 2, 0);
      emitLog(cb, QString("[mesh-analysis] size=%1  chassis=%2×%3×%4 cm  "
                          "wheels_detected=%5")
                    .arg(size_class_name(ar.size_class))
                    .arg(static_cast<int>(ar.chassis_x_max - ar.chassis_x_min))
                    .arg(static_cast<int>(ar.chassis_y_max - ar.chassis_y_min))
                    .arg(static_cast<int>(ar.chassis_z_max - ar.chassis_z_min))
                    .arg(ar.has_four_wheels ? "yes" : "no"));
      if (ar.has_four_wheels) {
        const char *wlabels[4] = { "FL", "FR", "RL", "RR" };
        for (size_t i = 0; i < 4 && i < ar.wheels.size(); ++i) {
          const WheelCandidate &w = ar.wheels[i];
          emitLog(cb, QString("[mesh-analysis]   wheel[%1] cx=%.1f cy=%.1f cz=%.1f "
                              "r=%.1f w=%.1f")
                        .arg(wlabels[i])
                        .arg(w.cx).arg(w.cy).arg(w.cz)
                        .arg(w.radius).arg(w.width));
        }
      } else {
        emitLog(cb, "[mesh-analysis] 4-wheel detection failed — "
                    "wheels may be topologically merged with body or "
                    "aspect-ratio filter excluded them; "
                    "derived wheel anchors from chassis AABB instead.");
      }
    } else {
      emitLog(cb, "[mesh-analysis] analysis returned no result "
                  "(zero faces after loading?); using fallback wheel positions.");
    }
  }
  if (s.name.isEmpty()) {
    s.name     = in.vehicle_name;
    s.mesh_path = in.mesh_path;
    for (size_t i = 0; i < 4; ++i) {
      s.wheels[i].radius = 35.f;
      s.wheels[i].width  = 22.f;
    }
    s.wheels[0].x =  140; s.wheels[0].y = -80; s.wheels[0].z = 35;
    s.wheels[1].x =  140; s.wheels[1].y =  80; s.wheels[1].z = 35;
    s.wheels[2].x = -140; s.wheels[2].y = -80; s.wheels[2].z = 35;
    s.wheels[3].x = -140; s.wheels[3].y =  80; s.wheels[3].z = 35;
    emitLog(cb, "[mesh-analysis] fallback: using default sedan wheel positions "
                "(140/80/35 cm); review calibration window and adjust if needed.");
  }
  s.content_path    = "/Game/Carla/Static/Vehicles/4Wheeled";
  s.base_vehicle_bp = "/Game/Carla/Blueprints/Vehicles/Mustang/BP_Mustang";
  s.mass           = in.knobs.mass;
  s.susp_damping    = in.knobs.susp_damping;
  s.size_class      = in.knobs.size_class;
  for (size_t i = 0; i < 4; ++i) {
    s.wheels[i].max_steer_angle  = (i < 2) ? in.knobs.max_steer_angle : 0.f;
    s.wheels[i].max_brake_torque = in.knobs.max_brake_torque;
    s.wheels[i].susp_max_raise   = in.knobs.susp_max_raise;
    s.wheels[i].susp_max_drop    = in.knobs.susp_max_drop;
  }
  return s;
}

QString defaultWorkspace(const QString &requested) {
  if (!requested.isEmpty()) {
    QDir().mkpath(requested);
    return requested;
  }
  const QString ws = QDir::current().absoluteFilePath(".cs_import");
  QDir().mkpath(ws);
  return ws;
}

QString defaultOutputDir(const QString &requested) {
  if (!requested.isEmpty()) {
    QDir().mkpath(requested);
    return requested;
  }
  return QDir::current().absolutePath();
}

}

ImportPipelineResult run_import_pipeline(const ImportPipelineInput &inIn,
                                       const ImportPipelineCallbacks &cb) {
  ImportPipelineInput in = inIn;
  if (in.vehicle_name.isEmpty()) {
    in.vehicle_name = QFileInfo(in.mesh_path).completeBaseName().toLower()
                       .replace(QRegularExpression("[^a-z0-9_]+"), "_");
  }
  if (in.workspace_dir.isEmpty()) in.workspace_dir = defaultWorkspace(QString());
  if (in.output_dir.isEmpty())    in.output_dir    = defaultOutputDir(QString());

  ImportPipelineResult res;
  if (cb.edit_knobs) cb.edit_knobs(in.knobs);
  if (cb.open_calibration) cb.open_calibration(in.mesh_path);
  if (cb.confirm_start_import && !cb.confirm_start_import()) {
    res.error_detail = "user cancelled at confirm-start gate";
    return res;
  }

  VehicleSpec spec = buildSpecFromInput(in, cb);
  if (!in.tires_path.isEmpty()
      && QFileInfo(in.mesh_path).suffix().toLower() == "obj"
      && QFileInfo(in.tires_path).suffix().toLower() == "obj"
      && QFileInfo(in.tires_path).isFile()) {
    const QString out_dir = in.workspace_dir + "/merged_" + in.vehicle_name;
    QDir(out_dir).removeRecursively();
    QDir().mkpath(out_dir);
    const ObjMergeResult mr = merge_body_and_tires(
        in.mesh_path, in.tires_path, out_dir, spec.wheels);
    if (mr.ok) {
      spec.mesh_path = mr.output_path;
      emitLog(cb, QString("merged body+tires (%1+%2 → %3 verts; %4/%5 copies) → %6")
                    .arg(mr.body_vertex_count).arg(mr.tire_vertex_count)
                    .arg(mr.merged_vertex_count).arg(mr.tire_copies_placed)
                    .arg(mr.tire_source_count).arg(mr.output_path));
    } else {
      emitLog(cb, QString("body+tires merge failed: %1 - sending body-only.")
                    .arg(mr.reason));
    }
  }

  emitLog(cb, QString("Sending spec to editor on port %1 (5-stage import; "
                       "30-90 s blocking)").arg(in.editor_port));
  const QJsonObject json = spec_to_json(spec);
  const QString resp = send_json(json);
  if (resp.isEmpty()) {
    res.error_detail = "no response from editor (port " +
                      QString::number(in.editor_port) + ")";
    emitLog(cb, "[FAIL] " + res.error_detail);
    return res;
  }
  const QJsonObject obj = QJsonDocument::fromJson(resp.toUtf8()).object();
  res.imported = obj.value("success").toBool();
  res.asset_path = obj.value("asset_path").toString();
  if (!res.imported) {
    res.error_detail = obj.value("error").toString();
    emitLog(cb, "[FAIL] import: " + res.error_detail);
    return res;
  }
  emitLog(cb, "[ OK ] import → " + res.asset_path);

  if (in.carla_shipping_root.isEmpty() || in.uproject_path.isEmpty()
      || in.editor_binary.isEmpty()) {
    res.error_detail = "shipping/uproject/editor_binary not configured "
                      "(missing CARLA_SHIPPING_ROOT / CARLA_SRC_ROOT / "
                      "CARLA_UNREAL_ENGINE_PATH)";
    emitLog(cb, "[FAIL] deploy preflight: " + res.error_detail);
    return res;
  }
  VehicleRegistration reg;
  reg.uproject_path      = in.uproject_path;
  reg.shipping_carla_root = in.carla_shipping_root;
  reg.bp_asset_path       = res.asset_path;
  reg.make              = "Custom";
  reg.model             = in.vehicle_name;
  reg.editor_binary      = in.editor_binary;

  const RegisterResult rr = register_vehicle_in_json(reg);
  emitLog(cb, QString("register: %1 - %2").arg(rr.ok ? "OK" : "FAIL", rr.detail));
  if (!rr.ok) { res.error_detail = "register: " + rr.detail; return res; }

  emitLog(cb, "deploying (UE cook commandlet - 2-3 min first cook, <30 s subsequent)");
  const DeployResult dr = deploy_vehicle_to_shipping_carla(reg);
  res.deployed = dr.ok;
  if (!dr.ok) {
    res.error_detail = "deploy: " + dr.detail;
    emitLog(cb, "[FAIL] " + res.error_detail);
    return res;
  }
  emitLog(cb, QString("deploy: OK - cooked=%1 copied=%2")
                .arg(dr.cooked_files).arg(dr.files_copied));

  if (in.zip_cooked_output) {
    const QString cookedSrc = QFileInfo(in.uproject_path).absolutePath()
                            + "/Saved/Cooked/Linux/CarlaUnreal/Content/"
                              "Carla/Static/Vehicles/4Wheeled/" + in.vehicle_name;
    if (QFileInfo(cookedSrc).isDir()) {
      const QString tarPath = QDir(in.output_dir).absoluteFilePath(
          QString("%1_cooked_%2.tar.gz")
            .arg(in.vehicle_name)
            .arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss")));
      QProcess tar;
      tar.start("/bin/sh", QStringList() << "-c"
                << QString("tar -C %1 -czf %2 %3")
                     .arg(QFileInfo(cookedSrc).absolutePath(), tarPath, in.vehicle_name));
      tar.waitForFinished(120000);
      if (tar.exitCode() == 0 && QFileInfo(tarPath).isFile()) {
        res.cooked_tar_gz = tarPath;
        emitLog(cb, "cooked artifact: " + tarPath);
      } else {
        emitLog(cb, QString("zip failed: tar exit %1").arg(tar.exitCode()));
      }
    }
  }

  return res;
}

static void openCalibrationImpl(const QString &mesh_path,
                                const std::array<float, 12> *wheels_cm,
                                QWidget *parent) {
  QCoreApplication *app = QCoreApplication::instance();
  const bool widgetsAvailable = app && app->inherits("QApplication");
  if (widgetsAvailable) {
    auto *win = VehiclePreviewWindow::instance();
    if (parent) win->setParent(parent, win->windowFlags());
    win->setWindowTitle(QString("Calibration - %1  "
                                "(orbit + drag, close window when done)")
                          .arg(QFileInfo(mesh_path).completeBaseName()));
    win->resize(1024, 720);
    if (wheels_cm) {
      const auto &w = *wheels_cm;
      win->show_for(mesh_path,
                   QVector3D(w[0],  w[1],  w[2]),
                   QVector3D(w[3],  w[4],  w[5]),
                   QVector3D(w[6],  w[7],  w[8]),
                   QVector3D(w[9],  w[10], w[11]));
    } else {
      win->show_for_mesh(mesh_path);
    }
    win->exec();
    return;
  }
  const QString previewBin = QCoreApplication::applicationDirPath()
                           + "/carla-studio-vehicle-preview";
  if (QFileInfo(previewBin).isExecutable()) {
    QStringList args;
    args << mesh_path << "--interactive";
    if (wheels_cm) {
      const auto &w = *wheels_cm;
      auto fmt = [](float a, float b, float c) {
        return QString("%1,%2,%3").arg(a, 0, 'f', 2)
                                   .arg(b, 0, 'f', 2)
                                   .arg(c, 0, 'f', 2);
      };
      args << "--wheel-fl" << fmt(w[0],  w[1],  w[2])
           << "--wheel-fr" << fmt(w[3],  w[4],  w[5])
           << "--wheel-rl" << fmt(w[6],  w[7],  w[8])
           << "--wheel-rr" << fmt(w[9],  w[10], w[11]);
    }
    QProcess spinner;
    spinner.setProcessChannelMode(QProcess::ForwardedChannels);
    spinner.start(previewBin, args);
    spinner.waitForStarted(5000);
    spinner.waitForFinished(-1);
  }
}

void open_calibration_window(const QString &mesh_path, QWidget *parent) {
  openCalibrationImpl(mesh_path, nullptr, parent);
}

void open_calibration_window_with_wheels(const QString &mesh_path,
                                     const std::array<float, 12> &wheels_cm_xyz,
                                     QWidget *parent) {
  openCalibrationImpl(mesh_path, &wheels_cm_xyz, parent);
}

}
