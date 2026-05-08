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

VehicleSpec buildSpecFromInput(const ImportPipelineInput &in) {
  VehicleSpec s;
  MeshGeometry g = loadMeshGeometry(in.meshPath);
  if (g.valid) {
    const MeshAnalysisResult ar = analyzeMesh(g, 1.0f);
    if (ar.ok) {
      s = buildSpecFromAnalysis(ar, in.vehicleName, in.meshPath, 1.0f, 2, 0);
    }
  }
  if (s.name.isEmpty()) {
    s.name     = in.vehicleName;
    s.meshPath = in.meshPath;
    for (int i = 0; i < 4; ++i) {
      s.wheels[i].radius = 35.f;
      s.wheels[i].width  = 22.f;
    }
    s.wheels[0].x =  140; s.wheels[0].y = -80; s.wheels[0].z = 35;
    s.wheels[1].x =  140; s.wheels[1].y =  80; s.wheels[1].z = 35;
    s.wheels[2].x = -140; s.wheels[2].y = -80; s.wheels[2].z = 35;
    s.wheels[3].x = -140; s.wheels[3].y =  80; s.wheels[3].z = 35;
  }
  s.contentPath   = "/Game/Carla/Static/Vehicles/4Wheeled";
  s.baseVehicleBP = "/Game/Carla/Blueprints/Vehicles/Mustang/BP_Mustang";
  s.mass          = in.knobs.mass;
  s.suspDamping   = in.knobs.suspDamping;
  s.sizeClass     = in.knobs.sizeClass;
  for (int i = 0; i < 4; ++i) {
    s.wheels[i].maxSteerAngle  = (i < 2) ? in.knobs.maxSteerAngle : 0.f;
    s.wheels[i].maxBrakeTorque = in.knobs.maxBrakeTorque;
    s.wheels[i].suspMaxRaise   = in.knobs.suspMaxRaise;
    s.wheels[i].suspMaxDrop    = in.knobs.suspMaxDrop;
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

}  // namespace

ImportPipelineResult runImportPipeline(const ImportPipelineInput &inIn,
                                       const ImportPipelineCallbacks &cb) {
  ImportPipelineInput in = inIn;
  if (in.vehicleName.isEmpty()) {
    in.vehicleName = QFileInfo(in.meshPath).completeBaseName().toLower()
                       .replace(QRegularExpression("[^a-z0-9_]+"), "_");
  }
  if (in.workspaceDir.isEmpty()) in.workspaceDir = defaultWorkspace(QString());
  if (in.outputDir.isEmpty())    in.outputDir    = defaultOutputDir(QString());

  ImportPipelineResult res;
  if (cb.editKnobs) cb.editKnobs(in.knobs);
  if (cb.openCalibration) cb.openCalibration(in.meshPath);
  if (cb.confirmStartImport && !cb.confirmStartImport()) {
    res.errorDetail = "user cancelled at confirm-start gate";
    return res;
  }

  // ----- Step A: build spec, optionally merge body+tires -----
  VehicleSpec spec = buildSpecFromInput(in);
  if (!in.tiresPath.isEmpty()
      && QFileInfo(in.meshPath).suffix().toLower() == "obj"
      && QFileInfo(in.tiresPath).suffix().toLower() == "obj"
      && QFileInfo(in.tiresPath).isFile()) {
    const QString outDir = in.workspaceDir + "/merged_" + in.vehicleName;
    QDir(outDir).removeRecursively();
    QDir().mkpath(outDir);
    const ObjMergeResult mr = mergeBodyAndTires(
        in.meshPath, in.tiresPath, outDir, spec.wheels);
    if (mr.ok) {
      spec.meshPath = mr.outputPath;
      emitLog(cb, QString("merged body+tires (%1+%2 → %3 verts; %4/%5 copies) → %6")
                    .arg(mr.bodyVertexCount).arg(mr.tireVertexCount)
                    .arg(mr.mergedVertexCount).arg(mr.tireCopiesPlaced)
                    .arg(mr.tireSourceCount).arg(mr.outputPath));
    } else {
      emitLog(cb, QString("body+tires merge failed: %1 - sending body-only.")
                    .arg(mr.reason));
    }
  }

  // ----- Step B: send to editor (5-stage import) -----
  emitLog(cb, QString("Sending spec to editor on port %1 (5-stage import; "
                       "30-90 s blocking)").arg(in.editorPort));
  const QJsonObject json = specToJson(spec);
  const QString resp = sendJson(json);
  if (resp.isEmpty()) {
    res.errorDetail = "no response from editor (port " +
                      QString::number(in.editorPort) + ")";
    emitLog(cb, "[FAIL] " + res.errorDetail);
    return res;
  }
  const QJsonObject obj = QJsonDocument::fromJson(resp.toUtf8()).object();
  res.imported = obj.value("success").toBool();
  res.assetPath = obj.value("asset_path").toString();
  if (!res.imported) {
    res.errorDetail = obj.value("error").toString();
    emitLog(cb, "[FAIL] import: " + res.errorDetail);
    return res;
  }
  emitLog(cb, "[ OK ] import → " + res.assetPath);

  // ----- Step C: register + cook + deploy to shipping -----
  if (in.carlaShippingRoot.isEmpty() || in.uprojectPath.isEmpty()
      || in.editorBinary.isEmpty()) {
    res.errorDetail = "shipping/uproject/editorBinary not configured "
                      "(missing CARLA_SHIPPING_ROOT / CARLA_SRC_ROOT / "
                      "CARLA_UNREAL_ENGINE_PATH)";
    emitLog(cb, "[FAIL] deploy preflight: " + res.errorDetail);
    return res;
  }
  VehicleRegistration reg;
  reg.uprojectPath      = in.uprojectPath;
  reg.shippingCarlaRoot = in.carlaShippingRoot;
  reg.bpAssetPath       = res.assetPath;
  reg.make              = "Custom";
  reg.model             = in.vehicleName;
  reg.editorBinary      = in.editorBinary;

  const RegisterResult rr = registerVehicleInJson(reg);
  emitLog(cb, QString("register: %1 - %2").arg(rr.ok ? "OK" : "FAIL", rr.detail));
  if (!rr.ok) { res.errorDetail = "register: " + rr.detail; return res; }

  emitLog(cb, "deploying (UE cook commandlet - 2-3 min first cook, <30 s subsequent)");
  const DeployResult dr = deployVehicleToShippingCarla(reg);
  res.deployed = dr.ok;
  if (!dr.ok) {
    res.errorDetail = "deploy: " + dr.detail;
    emitLog(cb, "[FAIL] " + res.errorDetail);
    return res;
  }
  emitLog(cb, QString("deploy: OK - cooked=%1 copied=%2")
                .arg(dr.cookedFiles).arg(dr.filesCopied));

  // ----- Step D: optional zip cooked subdir into outputDir -----
  if (in.zipCookedOutput) {
    const QString cookedSrc = QFileInfo(in.uprojectPath).absolutePath()
                            + "/Saved/Cooked/Linux/CarlaUnreal/Content/"
                              "Carla/Static/Vehicles/4Wheeled/" + in.vehicleName;
    if (QFileInfo(cookedSrc).isDir()) {
      const QString tarPath = QDir(in.outputDir).absoluteFilePath(
          QString("%1_cooked_%2.tar.gz")
            .arg(in.vehicleName)
            .arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss")));
      QProcess tar;
      tar.start("/bin/sh", QStringList() << "-c"
                << QString("tar -C %1 -czf %2 %3")
                     .arg(QFileInfo(cookedSrc).absolutePath(), tarPath, in.vehicleName));
      tar.waitForFinished(120000);
      if (tar.exitCode() == 0 && QFileInfo(tarPath).isFile()) {
        res.cookedTarGz = tarPath;
        emitLog(cb, "cooked artifact: " + tarPath);
      } else {
        emitLog(cb, QString("zip failed: tar exit %1").arg(tar.exitCode()));
      }
    }
  }

  return res;
}

static void openCalibrationImpl(const QString &meshPath,
                                const std::array<float, 12> *wheelsCm,
                                QWidget *parent) {
  QCoreApplication *app = QCoreApplication::instance();
  const bool widgetsAvailable = app && app->inherits("QApplication");
  if (widgetsAvailable) {
    auto *win = VehiclePreviewWindow::instance();
    if (parent) win->setParent(parent, win->windowFlags());
    win->setWindowTitle(QString("Calibration - %1  "
                                "(orbit + drag, close window when done)")
                          .arg(QFileInfo(meshPath).completeBaseName()));
    win->resize(1024, 720);
    if (wheelsCm) {
      const auto &w = *wheelsCm;
      win->showFor(meshPath,
                   QVector3D(w[0],  w[1],  w[2]),
                   QVector3D(w[3],  w[4],  w[5]),
                   QVector3D(w[6],  w[7],  w[8]),
                   QVector3D(w[9],  w[10], w[11]));
    } else {
      win->showForMesh(meshPath);
    }
    win->exec();
    return;
  }
  const QString previewBin = QCoreApplication::applicationDirPath()
                           + "/carla-studio-vehicle-preview";
  if (QFileInfo(previewBin).isExecutable()) {
    QStringList args;
    args << meshPath << "--interactive";
    if (wheelsCm) {
      const auto &w = *wheelsCm;
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

void openCalibrationWindow(const QString &meshPath, QWidget *parent) {
  openCalibrationImpl(meshPath, nullptr, parent);
}

void openCalibrationWindowWithWheels(const QString &meshPath,
                                     const std::array<float, 12> &wheelsCmXYZ,
                                     QWidget *parent) {
  openCalibrationImpl(meshPath, &wheelsCmXYZ, parent);
}

}  // namespace carla_studio::vehicle_import
