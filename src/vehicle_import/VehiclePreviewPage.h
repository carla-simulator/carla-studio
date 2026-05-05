// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QWidget>
#include <QString>
#include <QVector3D>
#include <array>

class QLineEdit;
class QPushButton;
class QPlainTextEdit;
class QLabel;
class QToolButton;

namespace Qt3DCore   { class QEntity; class QTransform; }
namespace Qt3DRender { class QSceneLoader; class QPointLight; class QDirectionalLight; }
namespace Qt3DExtras { class Qt3DWindow; class QOrbitCameraController;
                       class QPhongMaterial; class QSphereMesh; class QPlaneMesh;
                       class QCylinderMesh; }

namespace carla_studio::vehicle_import {

class VehiclePreviewPage : public QWidget {
  Q_OBJECT
 public:
  explicit VehiclePreviewPage(QWidget *parent = nullptr);

 public slots:
  void loadMesh(const QString &path);
  void setWheelPositionsCm(const QVector3D &fl, const QVector3D &fr,
                           const QVector3D &rl, const QVector3D &rr);
  void resetCamera();

  void viewTop();
  void viewSide();
  void viewFront();
  void view3D();
  void fitCamera();
  void rotateMinus90();
  void rotatePlus90();
  void rotate180();
  void mirrorX();
  void mirrorY();
  void recenterMesh();
  void resetMeshTransform();
  void applyToSpec();

 protected:
  bool eventFilter(QObject *obj, QEvent *ev) override;

 signals:
  void applyRequested(float yawDeg, float mirrorX, float mirrorY);

 private slots:
  void onBrowse();
  void onLoadClicked();
  void onSceneStatusChanged(int status);

 private:
  void buildScene();
  void buildGridFloor(Qt3DCore::QEntity *root, float halfExtentCm, float cellCm);
  void buildAxisGizmo(Qt3DCore::QEntity *root, float lengthCm);
  void buildReferenceOutlines(Qt3DCore::QEntity *root);
  void buildWheelMarkers(Qt3DCore::QEntity *root);
  void updateMeshBounds();
  void applyAdjustment(const QString &label);  // logs to info box

  float mAdjustYawDeg = 0.f;
  float mAdjustMirrorX = 1.f;
  float mAdjustMirrorY = 1.f;

  Qt3DExtras::Qt3DWindow            *mView          = nullptr;
  Qt3DCore::QEntity                 *mRoot          = nullptr;
  Qt3DCore::QEntity                 *mMeshEntity    = nullptr;
  Qt3DCore::QTransform              *mMeshTransform = nullptr;
  Qt3DRender::QSceneLoader          *mSceneLoader   = nullptr;
  Qt3DExtras::QOrbitCameraController *mCamCtl       = nullptr;

  std::array<Qt3DCore::QEntity*, 4>  mWheelMarkers{};
  std::array<Qt3DCore::QTransform*, 4> mWheelXforms{};

  QLineEdit      *mPathEdit       = nullptr;
  QPushButton    *mBrowseBtn      = nullptr;
  QPushButton    *mLoadBtn        = nullptr;
  QPushButton    *mResetCamBtn    = nullptr;
  QLabel         *mStatusLabel    = nullptr;
  QLabel         *mCalibrationBadge = nullptr;
  QPlainTextEdit *mInfoBox        = nullptr;
};

}  // namespace carla_studio::vehicle_import
