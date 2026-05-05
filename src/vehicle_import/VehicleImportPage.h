// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include "vehicle_import/ImportMode.h"
#include "vehicle_import/VehicleSpec.h"

#include <QString>
#include <QWidget>
#include <functional>
#include <memory>

class QDoubleSpinBox;
class QJsonObject;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;

namespace carla_studio::vehicle_import {

struct WheelSpinRow {
  QDoubleSpinBox *x = nullptr;
  QDoubleSpinBox *y = nullptr;
  QDoubleSpinBox *z = nullptr;
  QDoubleSpinBox *r = nullptr;
};

class VehicleImportPage : public QWidget {
  Q_OBJECT
 public:
  using EditorBinaryResolver = std::function<QString()>;
  using UprojectResolver     = std::function<QString()>;
  using CarlaRootResolver    = std::function<QString()>;
  using StartCarlaRequester  = std::function<void()>;

  explicit VehicleImportPage(EditorBinaryResolver findEditor,
                             UprojectResolver     findUproject,
                             CarlaRootResolver    findCarlaRoot,
                             StartCarlaRequester  requestStartCarla,
                             QWidget *parent = nullptr);

  void loadMeshFromPath(const QString &path);

 private:
  void onBrowse();
  void onImport();
  void onDrop();
  void onExport();
  void onBuildKit();
  void onBrowseEnginePath();
  void onBrowseUproject();
  void onBrowseWheels();
  void refreshModeBanner();
  void refreshSourceIndicators();
  void refreshDepCheckRow();
  void recomputeMergedSpec();
  void applyDisabledStateStyling();
  QString resolvedEditorBinary() const;
  QString resolvedUproject() const;
  QJsonObject buildImportSpec(const QString &meshPathToSend) const;

  EditorBinaryResolver mFindEditor;
  UprojectResolver     mFindUproject;
  CarlaRootResolver    mFindCarlaRoot;
  StartCarlaRequester  mRequestStartCarla;

  QLineEdit       *mEnginePathEdit     = nullptr;
  QPushButton     *mEngineBrowseBtn    = nullptr;
  QLineEdit       *mUprojectEdit       = nullptr;
  QPushButton     *mUprojectBrowseBtn  = nullptr;
  QLineEdit       *mMeshPathEdit       = nullptr;
  QPushButton     *mBrowseBtn          = nullptr;
  QLineEdit       *mWheelsPathEdit     = nullptr;
  QPushButton     *mWheelsBrowseBtn    = nullptr;
  QLabel          *mWheelsHintLabel    = nullptr;
  QLabel          *mAabbLabel          = nullptr;
  QLabel          *mValidationLabel    = nullptr;
  WheelSpinRow     mRowFL, mRowFR, mRowRL, mRowRR;
  QLineEdit       *mVehicleNameEdit    = nullptr;
  QLineEdit       *mContentPathEdit    = nullptr;
  QLineEdit       *mBaseVehicleBPEdit  = nullptr;
  QDoubleSpinBox  *mMassSpin           = nullptr;
  QDoubleSpinBox  *mSteerSpin          = nullptr;
  QDoubleSpinBox  *mSuspRaiseSpin      = nullptr;
  QDoubleSpinBox  *mSuspDropSpin       = nullptr;
  QDoubleSpinBox  *mSuspDampSpin       = nullptr;
  QDoubleSpinBox  *mBrakeSpin          = nullptr;
  QLabel          *mUeStatusLabel      = nullptr;
  QPushButton     *mImportBtn          = nullptr;
  QPushButton     *mCalibrateBtn       = nullptr;
  QPushButton     *mDropBtn            = nullptr;
  QPushButton     *mExportBtn          = nullptr;
  QPushButton     *mBuildKitBtn        = nullptr;
  QLabel          *mModeBanner         = nullptr;
  QProgressBar    *mProgress           = nullptr;
  QLabel          *mAssetLabel         = nullptr;
  QPlainTextEdit  *mLog                = nullptr;

  std::shared_ptr<float>   mScaleToCm   = std::make_shared<float>(1.0f);
  std::shared_ptr<int>     mUpAxis      = std::make_shared<int>(2);
  std::shared_ptr<int>     mForwardAxis = std::make_shared<int>(0);
  std::shared_ptr<bool>    mAabbValid   = std::make_shared<bool>(false);
  std::shared_ptr<QString> mLastAsset   = std::make_shared<QString>();
  std::shared_ptr<VehicleSpec> mDetectedSpec = std::make_shared<VehicleSpec>();
  std::shared_ptr<ImportMode>  mActiveMode   = std::make_shared<ImportMode>(ImportMode::Lite);
  QLabel                  *mDetectionLabel  = nullptr;
  QLabel                  *mDepCheckLabel   = nullptr;
};

}  // namespace carla_studio::vehicle_import
