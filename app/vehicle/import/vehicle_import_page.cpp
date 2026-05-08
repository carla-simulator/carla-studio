// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/vehicle_import_page.h"

#include "vehicle/import/bp_autopicker.h"
#include "vehicle/import/carla_spawn.h"
#include "vehicle/import/editor_process.h"
#include "vehicle/import/importer_client.h"
#include "vehicle/import/import_mode.h"
#include "vehicle/import/kit_bundler.h"
#include "vehicle/import/merged_spec_builder.h"
#include "vehicle/import/mesh_aabb.h"
#include "vehicle/import/mesh_analysis.h"
#ifdef CARLA_STUDIO_WITH_QT3D
#include "vehicle/import/vehicle_preview_window.h"
#include "vehicle/import/vehicle_preview_page.h"
#endif
#include "vehicle/import/mesh_geometry.h"
#include "vehicle/import/name_sanitizer.h"
#include "vehicle/import/obj_sanitizer.h"

#include <QApplication>
#include <QDesktopServices>
#include <QDoubleSpinBox>
#include <QFontMetrics>
#include <QSizePolicy>
#include <QUrl>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPointer>
#include <QProcess>
#include <QProgressBar>
#include <QPushButton>
#include <QSettings>
#include <QDateTime>
#include <QStandardPaths>
#include <QTime>
#include <QTimer>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrent>
#include <algorithm>
#include <array>
#include <cmath>

namespace carla_studio::vehicle_import {

namespace {

constexpr const char *kImportButtonStyle =
    "QPushButton { padding: 4px 14px; font-weight: 600; min-width: 80px; }"
    "QPushButton:disabled {"
    "  color: palette(mid);"
    "  background-color: palette(button);"
    "  border: 1px solid palette(mid);"
    "}";

constexpr const char *kSuccessButtonStyle =
    "QPushButton { padding: 4px 14px; font-weight: 600; min-width: 80px; "
    "background-color: #C8E6C9; color: #1B5E20; "
    "border: 1px solid #81C784; border-radius: 3px; }"
    "QPushButton:hover { background-color: #A5D6A7; }";

constexpr const char *kDefaultBaseVehicleBP =
    "/Game/Carla/Blueprints/USDImportTemplates/BaseUSDImportVehicle";

QString stamp() { return QTime::currentTime().toString("hh:mm:ss"); }

void mirrorUeStatus(QPointer<QLabel> p, const QString &text) {
  if (p) p->setText(text);
}

struct WheelGuess { float x, y, z, radius; };

std::array<WheelGuess, 4> guessWheels(float xMin, float xMax,
                                      float yMin, float yMax,
                                      float zMin, float zMax) {
  const float cx     = (xMin + xMax) * 0.5f;
  const float halfW  = (yMax - yMin) * 0.5f;
  const float axleY  = halfW * 0.8f;
  const float radius = std::max(5.f, (zMax - zMin) * 0.18f);
  const float axleZ  = zMin + radius;
  const float frontX = cx + (xMax - cx) * 0.60f;
  const float rearX  = cx - (cx - xMin) * 0.60f;
  return {{
    { frontX,  axleY, axleZ, radius },
    { frontX, -axleY, axleZ, radius },
    { rearX,   axleY, axleZ, radius },
    { rearX,  -axleY, axleZ, radius },
  }};
}

}  // namespace

VehicleImportPage::VehicleImportPage(EditorBinaryResolver findEditor,
                                     UprojectResolver     findUproject,
                                     CarlaRootResolver    findCarlaRoot,
                                     StartCarlaRequester  requestStartCarla,
                                     QWidget *parent)
    : QWidget(parent),
      mFindEditor(std::move(findEditor)),
      mFindUproject(std::move(findUproject)),
      mFindCarlaRoot(std::move(findCarlaRoot)),
      mRequestStartCarla(std::move(requestStartCarla)) {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(12, 12, 12, 12);
  root->setSpacing(6);

  mEnginePathEdit = new QLineEdit(this); mEnginePathEdit->setVisible(false);
  mUprojectEdit   = new QLineEdit(this); mUprojectEdit  ->setVisible(false);

  mDepCheckLabel = new QLabel(this);
  mDepCheckLabel->setWordWrap(true);
  mDepCheckLabel->setTextInteractionFlags(Qt::TextBrowserInteraction);
  mDepCheckLabel->setOpenExternalLinks(false);
  root->addWidget(mDepCheckLabel);
  refreshDepCheckRow();

  auto *fileRow = new QHBoxLayout();
  auto *vehLbl = new QLabel("Vehicle:");
  vehLbl->setFixedWidth(60);
  mMeshPathEdit = new QLineEdit();
  mMeshPathEdit->setPlaceholderText("path to .obj / .fbx / .glb / .dae …");
  mMeshPathEdit->setReadOnly(true);
  mBrowseBtn = new QPushButton("Browse…");
  auto *vehTrailer = new QLabel("(*.obj *.fbx *.glb *.dae)");
  vehTrailer->setStyleSheet("color: #888;");
  const int kTrailerWidth =
      QFontMetrics(vehTrailer->font()).horizontalAdvance(vehTrailer->text()) + 8;
  vehTrailer->setFixedWidth(kTrailerWidth);
  fileRow->addWidget(vehLbl);
  fileRow->addWidget(mMeshPathEdit, 1);
  fileRow->addWidget(mBrowseBtn);
  fileRow->addWidget(vehTrailer);
  root->addLayout(fileRow);

  auto *wheelsFileRow = new QHBoxLayout();
  auto *tireLbl = new QLabel("Tires:");
  tireLbl->setFixedWidth(60);
  mWheelsPathEdit = new QLineEdit();
  mWheelsPathEdit->setPlaceholderText("optional");
  mWheelsPathEdit->setReadOnly(true);
  mWheelsBrowseBtn = new QPushButton("Browse…");
  mWheelsHintLabel = new QLabel("(optional)");
  mWheelsHintLabel->setStyleSheet("color: #888;");
  mWheelsHintLabel->setFixedWidth(kTrailerWidth);
  wheelsFileRow->addWidget(tireLbl);
  wheelsFileRow->addWidget(mWheelsPathEdit, 1);
  wheelsFileRow->addWidget(mWheelsBrowseBtn);
  wheelsFileRow->addWidget(mWheelsHintLabel);
  root->addLayout(wheelsFileRow);

  mAabbLabel       = new QLabel(this); mAabbLabel->setVisible(false);
  mDetectionLabel  = new QLabel(this); mDetectionLabel->setVisible(false);
  mValidationLabel = new QLabel(this); mValidationLabel->setVisible(false);

  auto *wheelsGroup = new QGroupBox("Wheel Positions (cm - Unreal units)");
  auto *wheelsForm  = new QFormLayout();
  wheelsForm->setRowWrapPolicy(QFormLayout::DontWrapRows);
  wheelsForm->setVerticalSpacing(4);
  wheelsForm->setHorizontalSpacing(8);
  wheelsForm->setContentsMargins(8, 6, 8, 6);

  auto makeWheelRow = [&](const QString &label) -> WheelSpinRow {
    WheelSpinRow wr;
    auto *row = new QHBoxLayout();
    row->setSpacing(0);
    row->setContentsMargins(0, 0, 0, 0);
    auto makeSpin = [row](const QString &prefix, float lo, float hi, bool firstInRow) -> QDoubleSpinBox * {
      auto *pl = new QLabel(prefix);
      pl->setFixedWidth(14);
      pl->setAlignment(Qt::AlignCenter);
      pl->setContentsMargins(firstInRow ? 0 : 10, 0, 4, 0);
      auto *sb = new QDoubleSpinBox();
      sb->setRange(lo, hi);
      sb->setDecimals(1);
      sb->setSingleStep(0.5);
      sb->setFixedWidth(88);
      row->addWidget(pl);
      row->addWidget(sb);
      return sb;
    };
    wr.x = makeSpin("X", -5000, 5000, true);
    wr.y = makeSpin("Y", -5000, 5000, false);
    wr.z = makeSpin("Z", -5000, 5000, false);
    row->addStretch(1);
    wr.r = makeSpin("R",      1,  200, false);
    wr.r->setToolTip("Wheel radius");
    auto *rowW = new QWidget();
    rowW->setLayout(row);
    wheelsForm->addRow(label, rowW);
    return wr;
  };
  mRowFL = makeWheelRow("Front Left");
  mRowFR = makeWheelRow("Front Right");
  mRowRL = makeWheelRow("Rear Left");
  mRowRR = makeWheelRow("Rear Right");
  wheelsGroup->setLayout(wheelsForm);
  root->addWidget(wheelsGroup);

  auto *paramsGroup = new QGroupBox("Vehicle Parameters");
  auto *paramsForm  = new QFormLayout();
  paramsForm->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  paramsForm->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
  paramsForm->setRowWrapPolicy(QFormLayout::DontWrapRows);
  paramsForm->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
  paramsForm->setVerticalSpacing(4);
  paramsForm->setHorizontalSpacing(10);
  paramsForm->setContentsMargins(8, 6, 8, 6);

  mVehicleNameEdit = new QLineEdit();
  mVehicleNameEdit->setPlaceholderText("e.g. my_truck (lowercase letters, digits, underscores)");
  mVehicleNameEdit->setValidator(new VehicleNameValidator(mVehicleNameEdit));
  mVehicleNameEdit->setMaxLength(kMaxVehicleNameLength);
  QObject::connect(mVehicleNameEdit, &QLineEdit::editingFinished, mVehicleNameEdit, [edit = mVehicleNameEdit]() {
    const QString fixed = sanitizeVehicleName(edit->text());
    if (fixed != edit->text()) edit->setText(fixed);
  });
  paramsForm->addRow("Vehicle Name", mVehicleNameEdit);

  mContentPathEdit = new QLineEdit("/Game/Carla/Static/Vehicles/4Wheeled");
  paramsForm->addRow("UE Content Path", mContentPathEdit);

  mBaseVehicleBPEdit = new QLineEdit(kDefaultBaseVehicleBP);
  mBaseVehicleBPEdit->setProperty("lastAutoPick", QString(kDefaultBaseVehicleBP));
  mBaseVehicleBPEdit->setToolTip(
    "Existing CARLA vehicle Blueprint to use as the rigging template.\n"
    "Defaults to BaseUSDImportVehicle - the only stock BP with a real "
    "USkeletalMesh + PhysicsAsset that the importer can consume.");
  paramsForm->addRow("Base Vehicle BP", mBaseVehicleBPEdit);

  const QSizePolicy expandH(QSizePolicy::Expanding, QSizePolicy::Fixed);
  mVehicleNameEdit  ->setSizePolicy(expandH);
  mVehicleNameEdit  ->setAlignment(Qt::AlignRight);
  mContentPathEdit  ->setSizePolicy(expandH);
  mContentPathEdit  ->setAlignment(Qt::AlignRight);
  mBaseVehicleBPEdit->setSizePolicy(expandH);
  mBaseVehicleBPEdit->setAlignment(Qt::AlignRight);

  auto makeSpin = [&expandH](double lo, double hi, double def, const QString &suffix, int decimals = 2) {
    auto *sb = new QDoubleSpinBox();
    sb->setRange(lo, hi);
    sb->setDecimals(decimals);
    sb->setValue(def);
    if (!suffix.isEmpty()) sb->setSuffix(" " + suffix);
    sb->setSizePolicy(expandH);
    sb->setAlignment(Qt::AlignRight);
    return sb;
  };
  mMassSpin      = makeSpin(100,  100000, 1500, "kg",   2);
  mSteerSpin     = makeSpin(0,    90,     70,   "°",    2);
  mSuspRaiseSpin = makeSpin(0,    100,    10,   "cm",   2);
  mSuspDropSpin  = makeSpin(0,    100,    10,   "cm",   2);
  mSuspDampSpin  = makeSpin(0.1,  2.0,    0.65, "",     2);
  mSuspDampSpin->setSingleStep(0.05);
  mBrakeSpin     = makeSpin(0,    10000,  1500, "Nm",   2);
  const QString kDriveabilityTip =
      "* Driveability-critical: differs per vehicle (sedan vs SUV vs truck). "
      "If the imported vehicle won't drive forward, try adjusting these. "
      "There is no auto-tune; physics depends on body geometry.";
  auto markDriveability = [&](QDoubleSpinBox *sb) {
    sb->setToolTip(kDriveabilityTip + (sb->toolTip().isEmpty() ? QString()
                                       : "\n" + sb->toolTip()));
  };
  markDriveability(mMassSpin);
  markDriveability(mSuspRaiseSpin);
  markDriveability(mSuspDropSpin);
  markDriveability(mSuspDampSpin);
  markDriveability(mBrakeSpin);
  paramsForm->addRow("* Vehicle Mass",         mMassSpin);
  paramsForm->addRow("Max Steer Angle",        mSteerSpin);
  paramsForm->addRow("* Suspension Max Raise", mSuspRaiseSpin);
  paramsForm->addRow("* Suspension Max Drop",  mSuspDropSpin);
  paramsForm->addRow("* Suspension Damping",   mSuspDampSpin);
  paramsForm->addRow("* Max Brake Torque",     mBrakeSpin);
  auto *driveabilityNote = new QLabel(
      "<i>* Driveability-critical fields. Tune per-vehicle if "
      "the import doesn't drive forward; defaults are sedan-tuned.</i>");
  driveabilityNote->setWordWrap(true);
  driveabilityNote->setStyleSheet("color: #888; font-size: 11px;");
  paramsForm->addRow(driveabilityNote);
  paramsGroup->setLayout(paramsForm);
  root->addWidget(paramsGroup);

  auto *importRow = new QHBoxLayout();
  mUeStatusLabel = new QLabel("UE Editor: not connected");
  mUeStatusLabel->setStyleSheet("color: gray;");

  const QString envEngineHint =
      QString::fromLocal8Bit(qgetenv("CARLA_UNREAL_ENGINE_PATH")).trimmed();
  const QString autoUprojHint = mFindUproject ? mFindUproject() : QString();

  mEngineBrowseBtn = new QPushButton("Unreal Editor");
  mEngineBrowseBtn->setMinimumHeight(28);
  mEngineBrowseBtn->setMinimumWidth(120);

  mUprojectBrowseBtn = new QPushButton("Carla built from source");
  mUprojectBrowseBtn->setMinimumHeight(28);
  mUprojectBrowseBtn->setMinimumWidth(140);

  mImportBtn = new QPushButton("Import");
  mImportBtn->setEnabled(false);
  mImportBtn->setMinimumWidth(96);
  mImportBtn->setToolTip(
    "Run the import end-to-end:\n"
    "  • Spawn a headless UE Editor in the background.\n"
    "  • Wait for the VehicleImporter port to come up.\n"
    "  • Send the spec, import the mesh, build wheel + vehicle BPs.");


  mDropBtn = new QPushButton("Drive");
  mDropBtn->setEnabled(false);
  mDropBtn->setMinimumWidth(110);
  mDropBtn->setToolTip(
    "Register the imported vehicle in VehicleParameters.json, spawn it in the\n"
    "running CARLA simulator, and hand control to the Traffic Manager (SAE L5 -\n"
    "the vehicle drives itself around the city). Requires CARLA started via the\n"
    "Studio START button. If the blueprint isn't yet visible to CARLA, restart\n"
    "the simulator to pick up the new registration.");

  mModeBanner = new QLabel(this);
  mModeBanner->setVisible(false);

  mExportBtn = new QPushButton("Export");
  mExportBtn->setEnabled(false);
  mExportBtn->setMinimumWidth(96);
  mExportBtn->setToolTip(
      "Bundle the cooked + driveable vehicle into a single .zip archive named\n"
      "carla_<vehicle>_ue<4|5>_<MMDDYYYY>.zip under ~/.carla_studio/exports/.\n"
      "Useful for redistributing or installing into another CARLA root via the\n"
      "(former) Pre-built Package install.");
  connect(mExportBtn, &QPushButton::clicked, this, &VehicleImportPage::onExport);

  auto *prereqRow = new QHBoxLayout();
  prereqRow->setSpacing(8);
  mEngineBrowseBtn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  mUprojectBrowseBtn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  prereqRow->addWidget(mEngineBrowseBtn,   1);
  prereqRow->addWidget(mUprojectBrowseBtn, 1);
  root->addLayout(prereqRow);

  mCalibrateBtn = new QPushButton("Calibrate");
  mCalibrateBtn->setEnabled(false);
  mCalibrateBtn->setMinimumWidth(96);
  mCalibrateBtn->setToolTip(
      "Open the Vehicle Preview window for the current import - verify\n"
      "orientation on the calibration grid; use Rot/Mirror buttons + Apply &\n"
      "Re-import if the auto-detected forward axis is wrong.");
  connect(mCalibrateBtn, &QPushButton::clicked, this, [this]() {
#ifdef CARLA_STUDIO_WITH_QT3D
    if (!mLastAsset || mLastAsset->isEmpty()) {
      if (mLog) mLog->appendPlainText(QString("[%1]  Calibrate: no imported asset - run Import first.").arg(stamp()));
      return;
    }
    const QString name = sanitizeVehicleName(mVehicleNameEdit->text().trimmed());
    const QString canonical = QString("/tmp/vi_%1_body_canonical.obj").arg(name);
    const QString showPath  = QFileInfo(canonical).exists()
        ? canonical : mMeshPathEdit->text().trimmed();
    QVector3D fl, fr, rl, rr;
    if (mDetectedSpec) {
      fl = QVector3D(mDetectedSpec->wheels[0].x, mDetectedSpec->wheels[0].y, mDetectedSpec->wheels[0].z);
      fr = QVector3D(mDetectedSpec->wheels[1].x, mDetectedSpec->wheels[1].y, mDetectedSpec->wheels[1].z);
      rl = QVector3D(mDetectedSpec->wheels[2].x, mDetectedSpec->wheels[2].y, mDetectedSpec->wheels[2].z);
      rr = QVector3D(mDetectedSpec->wheels[3].x, mDetectedSpec->wheels[3].y, mDetectedSpec->wheels[3].z);
    }
    VehiclePreviewWindow::instance()->showFor(showPath, fl, fr, rl, rr);
#endif
  });

  importRow->setSpacing(8);
  for (QPushButton *b : { mImportBtn, mCalibrateBtn, mDropBtn, mExportBtn }) {
    b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    b->setMinimumWidth(110);
    b->setStyleSheet(kImportButtonStyle);
  }
  importRow->addWidget(mImportBtn,    1);
  importRow->addWidget(mCalibrateBtn, 1);
  importRow->addWidget(mDropBtn,      1);
  importRow->addWidget(mExportBtn,    1);
  root->addLayout(importRow);

  mProgress = new QProgressBar();
  mProgress->setRange(0, 100);
  mProgress->setValue(0);
  mProgress->setTextVisible(true);
  mProgress->setFormat("idle");
  mProgress->setVisible(false);
  root->addWidget(mProgress);

  mAssetLabel = new QLabel();
  mAssetLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
  mAssetLabel->setVisible(false);
  mAssetLabel->setStyleSheet("color: #2E7D32; font-weight: 600;");
  root->addWidget(mAssetLabel);

  mLog = new QPlainTextEdit();
  mLog->setReadOnly(true);
  mLog->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  mLog->setMinimumHeight(160);
  mLog->setPlaceholderText("Import log will appear here…");
  root->addWidget(mLog, 1);

  auto *footerRow = new QHBoxLayout();
  footerRow->setContentsMargins(0, 0, 0, 0);
  mUeStatusLabel->setVisible(true);
  mUeStatusLabel->setStyleSheet("color: #555; font-size: 11px;");
  mUeStatusLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  footerRow->addWidget(mUeStatusLabel, 1, Qt::AlignLeft);
  root->addLayout(footerRow);

  applyDisabledStateStyling();

  connect(mBrowseBtn,         &QPushButton::clicked, this, &VehicleImportPage::onBrowse);
  connect(mImportBtn,         &QPushButton::clicked, this, &VehicleImportPage::onImport);
  connect(mDropBtn,           &QPushButton::clicked, this, &VehicleImportPage::onDrop);
  connect(mWheelsBrowseBtn,   &QPushButton::clicked, this, &VehicleImportPage::onBrowseWheels);
  connect(mEngineBrowseBtn,   &QPushButton::clicked, this, &VehicleImportPage::onBrowseEnginePath);
  connect(mUprojectBrowseBtn, &QPushButton::clicked, this, &VehicleImportPage::onBrowseUproject);

#ifdef CARLA_STUDIO_WITH_QT3D
  connect(VehiclePreviewWindow::instance(), &QDialog::finished, this,
          [this](int) {
            if (mCalibrateBtn && mCalibrateBtn->isEnabled())
              mCalibrateBtn->setStyleSheet(kSuccessButtonStyle);
          });

  if (auto *page = VehiclePreviewWindow::instance()->page()) {
    connect(page, &VehiclePreviewPage::applyRequested, this,
            [this](float yawDeg, float mirrorX, float mirrorY) {
              if (!mDetectedSpec) {
                if (mLog) mLog->appendPlainText(
                    QString("[%1]  Apply: no spec to update.").arg(stamp()));
                return;
              }
              mDetectedSpec->adjustYawDeg  = yawDeg;
              mDetectedSpec->adjustMirrorX = mirrorX;
              mDetectedSpec->adjustMirrorY = mirrorY;
              if (mLog) mLog->appendPlainText(QString(
                  "[%1]  Apply calibration → spec (yaw=%2°, mirror=(%3,%4)) - re-importing.")
                    .arg(stamp()).arg(yawDeg, 0, 'f', 0)
                    .arg(mirrorX > 0 ? "+" : "-")
                    .arg(mirrorY > 0 ? "+" : "-"));
              onImport();
            });
  }
#endif

  refreshSourceIndicators();
}

static QString autoConvertBlendIfNeeded(const QString &path, QPlainTextEdit *log);

void VehicleImportPage::onBrowseWheels() {
  const QString seed = mWheelsPathEdit->text().trimmed();
  const QString p = QFileDialog::getOpenFileName(
      this, "Select tire-pack mesh",
      seed.isEmpty() ? QStandardPaths::writableLocation(QStandardPaths::HomeLocation) : seed,
      "Mesh files (*.obj *.gltf *.glb *.dae *.blend *.fbx)");
  if (p.isEmpty()) return;
  const QString resolved = autoConvertBlendIfNeeded(p, mLog);
  mWheelsPathEdit->setText(resolved.isEmpty() ? p : resolved);
  recomputeMergedSpec();
}

void VehicleImportPage::recomputeMergedSpec() {
  if (!mDetectedSpec || !mDetectedSpec->detectedFromMesh) return;
  const QString wheelsPath = mWheelsPathEdit ? mWheelsPathEdit->text().trimmed() : QString();
  WheelTemplate tpl;
  if (!wheelsPath.isEmpty() && QFileInfo(wheelsPath).suffix().toLower() == "obj") {
    tpl = wheelTemplateFromTiresObj(wheelsPath, *mScaleToCm);
    if (!tpl.valid) {
      const MeshGeometry wg = loadMeshGeometryOBJ(wheelsPath);
      if (wg.valid) {
        const MeshAnalysisResult wr = analyzeMesh(wg, *mScaleToCm);
        tpl = wheelTemplateFromAnalysis(wr);
      }
    }
  }
  const MergeResult mr = mergeBodyAndWheels(*mDetectedSpec, tpl, !wheelsPath.isEmpty());
  if (!mr.ok) {
    mLog->appendPlainText(QString("[%1]  Merge: %2").arg(stamp(), mr.reason));
    if (mImportBtn)   mImportBtn->setEnabled(false);
      return;
  }
  *mDetectedSpec = mr.spec;
  const auto applyRowFromWheel = [](WheelSpinRow &row, const WheelSpec &w) {
    row.x->setValue(w.x); row.y->setValue(w.y);
    row.z->setValue(w.z); row.r->setValue(w.radius);
  };
  applyRowFromWheel(mRowFL, mDetectedSpec->wheels[0]);
  applyRowFromWheel(mRowFR, mDetectedSpec->wheels[1]);
  applyRowFromWheel(mRowRL, mDetectedSpec->wheels[2]);
  applyRowFromWheel(mRowRR, mDetectedSpec->wheels[3]);
  const QString modeName =
      mr.mode == IngestionMode::Combined      ? "combined"      :
      mr.mode == IngestionMode::TireSwap      ? "tire-swap"     :
      mr.mode == IngestionMode::BodyPlusTires ? "body+tires"    : "?";
  mLog->appendPlainText(QString("[%1]  Merge: %2 (%3).").arg(stamp(), modeName, mr.reason));

  const SpecVerification v = verifyChaosVehicleSpec(*mDetectedSpec);
  for (const QString &warn : v.warnings)
    mLog->appendPlainText(QString("[%1]  Chaos-safety warn: %2").arg(stamp(), warn));
  for (const QString &err : v.errors)
    mLog->appendPlainText(QString("[%1]  Chaos-safety ERR : %2").arg(stamp(), err));
  if (mImportBtn) {
    mImportBtn->setEnabled(v.ok);
    if (!v.ok) {
      mImportBtn->setToolTip(
        "Spec has Chaos-safety errors (see log) - fix before importing.");
    } else {
      mImportBtn->setToolTip(QString());
    }
  }
}

void VehicleImportPage::refreshDepCheckRow() {
  if (!mDepCheckLabel) return;

  struct Probe {
    QString name;
    bool    ok;
    QString hint;
  };
  QList<Probe> probes;

  const QString blender = QStandardPaths::findExecutable("blender");
  probes.append({ "blender", !blender.isEmpty(),
                  "needed for .blend/.fbx/.gltf/.glb/.dae source meshes - "
                  "<i>sudo apt install blender</i>" });

  const QString zip = QStandardPaths::findExecutable("zip");
  probes.append({ "zip", !zip.isEmpty(),
                  "needed for Export-as-Kit - <i>sudo apt install zip</i>" });

#ifdef CARLA_STUDIO_WITH_LIBCARLA
  probes.append({ "libcarla", true, QString() });
#else
  probes.append({ "libcarla", false,
                  "Drive cannot reach a running CARLA - rebuild CarlaStudio "
                  "with -DBUILD_CARLA_CLIENT=ON" });
#endif

  QStringList missing;
  for (const Probe &p : probes) if (!p.ok) missing << p.name;

  if (missing.isEmpty()) {
    mDepCheckLabel->setText(
      QString("<span style='color:#2E7D32;font-size:11px;'>"
              "Vehicle Import deps OK: %1</span>")
        .arg(QStringList({ "blender", "zip", "libcarla" }).join(" · ")));
  } else {
    QStringList lines;
    for (const Probe &p : probes) {
      if (p.ok) continue;
      lines << QString("<span style='color:#C62828;'>missing</span> "
                       "<b>%1</b> - %2").arg(p.name, p.hint);
    }
    mDepCheckLabel->setText(
      QString("<span style='color:#E65100;font-size:11px;'>"
              "Vehicle Import dep check (%1 missing):</span><br>%2")
        .arg(missing.size()).arg(lines.join("<br>")));
  }
}

void VehicleImportPage::refreshModeBanner() {
  refreshSourceIndicators();
  refreshDepCheckRow();
}

void VehicleImportPage::refreshSourceIndicators() {
  const auto findUprojectUnder = [](const QString &root) -> QString {
    if (root.isEmpty()) return QString();
    for (const QString &rel : {
        QStringLiteral("/Unreal/CarlaUnreal/CarlaUnreal.uproject"),
        QStringLiteral("/Unreal/CarlaUE5/CarlaUE5.uproject"),
        QStringLiteral("/Unreal/CarlaUE4/CarlaUE4.uproject")}) {
      const QString cand = root + rel;
      if (QFileInfo(cand).isFile()) return cand;
    }
    return QString();
  };
  QString uproj = resolvedUproject();
  if (uproj.isEmpty() || !QFileInfo(uproj).isFile()) {
    const QString carlaRoot = mFindCarlaRoot ? mFindCarlaRoot() : QString();
    uproj = findUprojectUnder(carlaRoot);
  }
  if (uproj.isEmpty()) {
    uproj = findUprojectUnder(QString::fromLocal8Bit(qgetenv("CARLA_ROOT")).trimmed());
  }
  if (uproj.isEmpty()) {
    uproj = findUprojectUnder(QString::fromLocal8Bit(qgetenv("CARLA_SRC_ROOT")).trimmed());
  }
  QString eng = mEnginePathEdit ? mEnginePathEdit->text().trimmed() : QString();
  if (eng.isEmpty()) {
    const QString bin = resolvedEditorBinary();
    if (!bin.isEmpty()) {
      const QString suffix = QStringLiteral("/Engine/Binaries/Linux/UnrealEditor");
      if (bin.endsWith(suffix))
        eng = bin.left(bin.size() - suffix.size());
      else
        eng = QFileInfo(bin).absolutePath();
    }
  }
  if (eng.isEmpty()) {
    eng = QString::fromLocal8Bit(qgetenv("CARLA_UNREAL_ENGINE_PATH")).trimmed();
  }
  const bool engOk = !eng.isEmpty()
      && QFileInfo(eng + "/Engine/Binaries/Linux/UnrealEditor").isFile();
  const bool uprojOk = !uproj.isEmpty() && QFileInfo(uproj).isFile();
  const QString indicatorStyle = QStringLiteral(
      "QPushButton {"
      "  padding: 4px 14px; font-weight: 600; min-width: 80px;"
      "  background-color: %1; color: %2;"
      "  border: 1px solid %3; border-radius: 3px;"
      "}"
      "QPushButton:hover { background-color: %4; }");
  const QString okStyle  = indicatorStyle.arg("#C8E6C9", "#1B5E20", "#81C784", "#A5D6A7");
  const QString badStyle = indicatorStyle.arg("#FFCDD2", "#B00020", "#E57373", "#EF9A9A");
  if (mEngineBrowseBtn) {
    mEngineBrowseBtn->setText(engOk ? QStringLiteral("Unreal Editor") : QStringLiteral("Unreal Editor"));
    mEngineBrowseBtn->setStyleSheet(engOk ? okStyle : badStyle);
    mEngineBrowseBtn->setToolTip(engOk
        ? QStringLiteral("Unreal Engine root: %1\n(click to override)").arg(eng)
        : QStringLiteral("Unreal Engine 5 not detected. Set CARLA_UNREAL_ENGINE_PATH "
                         "or click to choose the engine root."));
  }
  if (mUprojectBrowseBtn) {
    mUprojectBrowseBtn->setText(uprojOk ? QStringLiteral("Carla built from source") : QStringLiteral("Carla built from source"));
    mUprojectBrowseBtn->setStyleSheet(uprojOk ? okStyle : badStyle);
    mUprojectBrowseBtn->setToolTip(uprojOk
        ? QStringLiteral("CarlaUnreal.uproject: %1\n(click to override)").arg(uproj)
        : QStringLiteral("CarlaUnreal.uproject not found. Set CARLA_ROOT or "
                         "CARLA_SRC_ROOT, or click to choose the uproject."));
  }
  if (mImportBtn) {
    const bool ready = engOk && uprojOk;
    mImportBtn->setToolTip(ready
        ? QStringLiteral("Headless UE Editor: -unattended -RenderOffScreen -nosplash -nosound. "
                         "Imports mesh, builds wheel + vehicle BPs, saves assets.")
        : QStringLiteral("Set UE5 + CARLA src paths first (buttons on the left)."));
  }
  if (mDropBtn) {
    mDropBtn->setToolTip(QStringLiteral(
        "Spawn imported vehicle in CARLA on Traffic Manager autopilot (SAE L5)."));
  }
}

void VehicleImportPage::onBuildKit() {
  if (!mDetectedSpec || !mDetectedSpec->detectedFromMesh) {
    mLog->appendPlainText("Build Kit: no mesh-driven spec yet - drop a model first.");
    return;
  }
  VehicleSpec specCopy = *mDetectedSpec;
  specCopy.name = sanitizeVehicleName(mVehicleNameEdit->text().trimmed());
  if (specCopy.name.isEmpty()) {
    mLog->appendPlainText("Build Kit: vehicle name is empty after sanitization.");
    return;
  }
  const QString tmpRoot = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
  const QString target  = tmpRoot + "/carla_studio_kits/" + specCopy.name + "_kit";
  const KitBundleResult res = writeVehicleKit(specCopy, target);
  if (res.ok) {
    mLog->appendPlainText(QString("[%1]  Vehicle kit written: %2 file(s) at %3")
                            .arg(stamp()).arg(res.writtenFiles.size()).arg(res.outputDir));
    if (mUeStatusLabel)
      mUeStatusLabel->setText(QString("Kit ready: %1").arg(res.outputDir));
    QDesktopServices::openUrl(QUrl::fromLocalFile(res.outputDir));
  } else {
    mLog->appendPlainText(QString("[%1]  Vehicle kit FAILED: %2")
                            .arg(stamp()).arg(res.detail));
    if (mUeStatusLabel)
      mUeStatusLabel->setText(QString("Kit failed: %1").arg(res.detail));
  }
}

QString VehicleImportPage::resolvedEditorBinary() const {
  const QString override_ = mEnginePathEdit ? mEnginePathEdit->text().trimmed() : QString();
  if (!override_.isEmpty()) {
    const QString candidate = override_ + "/Engine/Binaries/Linux/UnrealEditor";
    if (QFileInfo(candidate).isExecutable()) return candidate;
    if (QFileInfo(override_).isExecutable()) return override_;
  }
  return mFindEditor ? mFindEditor() : QString();
}

QString VehicleImportPage::resolvedUproject() const {
  const QString override_ = mUprojectEdit ? mUprojectEdit->text().trimmed() : QString();
  if (!override_.isEmpty() && QFileInfo(override_).isFile()) return override_;
  return mFindUproject ? mFindUproject() : QString();
}

void VehicleImportPage::onBrowseEnginePath() {
  const QString seed = mEnginePathEdit->text().trimmed();
  const QString dir  = QFileDialog::getExistingDirectory(
      this, "Select Unreal Engine 5 root",
      seed.isEmpty() ? QStandardPaths::writableLocation(QStandardPaths::HomeLocation) : seed);
  if (!dir.isEmpty()) {
    mEnginePathEdit->setText(dir);
    refreshSourceIndicators();
  }
}

void VehicleImportPage::onBrowseUproject() {
  const QString seed = mUprojectEdit->text().trimmed();
  const QString dir  = seed.isEmpty()
      ? QStandardPaths::writableLocation(QStandardPaths::HomeLocation)
      : QFileInfo(seed).absolutePath();
  const QString path = QFileDialog::getOpenFileName(
      this, "Select CarlaUnreal.uproject", dir, "Unreal Project (*.uproject);;All files (*)");
  if (!path.isEmpty()) {
    mUprojectEdit->setText(path);
    refreshSourceIndicators();
  }
}

void VehicleImportPage::applyDisabledStateStyling() {
  mImportBtn->setStyleSheet(kImportButtonStyle);
  mDropBtn  ->setStyleSheet(kImportButtonStyle);
}

static QString autoConvertBlendIfNeeded(const QString &path,
                                        QPlainTextEdit *log)
{
  const QString ext = QFileInfo(path).suffix().toLower();
  if (ext == "obj") return path;
  static const QStringList kConvertibles = { "blend", "fbx", "gltf", "glb", "dae" };
  if (!kConvertibles.contains(ext)) return path;

  const QString blender = QStandardPaths::findExecutable("blender");
  if (blender.isEmpty()) {
    if (log) log->appendPlainText(
        QString("%1->obj: blender CLI not on PATH; cannot auto-convert.").arg(ext));
    return QString();
  }
  const QString outObj = QString("/tmp/cs_%1_convert_%2.obj")
      .arg(ext, QFileInfo(path).baseName());
  if (log) log->appendPlainText(
      QString("%1->obj: converting %2 -> %3 …").arg(ext, path, outObj));

  QString preImport;
  if      (ext == "fbx")   preImport = QString("bpy.ops.import_scene.fbx(filepath=r'%1')").arg(path);
  else if (ext == "gltf"
        || ext == "glb")   preImport = QString("bpy.ops.import_scene.gltf(filepath=r'%1')").arg(path);
  else if (ext == "dae")   preImport = QString("bpy.ops.wm.collada_import(filepath=r'%1')").arg(path);

  QStringList args;
  args << "--background";
  if (ext == "blend") args << path;          // open as scene
  args << "--python-expr";
  const QString expr = QString(
      "import bpy\n"
      "if %1:\n"
      "    for o in list(bpy.data.objects):\n"
      "        bpy.data.objects.remove(o, do_unlink=True)\n"
      "    %2\n"
      "try:\n"
      "    bpy.ops.wm.obj_export(filepath=r'%3')\n"
      "except Exception:\n"
      "    bpy.ops.export_scene.obj(filepath=r'%3')\n"
  ).arg(preImport.isEmpty() ? "False" : "True",
        preImport.isEmpty() ? QStringLiteral("pass") : preImport,
        outObj);
  args << expr;
  QProcess proc;
  proc.start(blender, args);
  if (!proc.waitForFinished(180000) || proc.exitCode() != 0) {
    if (log) log->appendPlainText(
        QString("%1->obj: conversion failed (%2)").arg(ext, proc.errorString()));
    return QString();
  }
  if (!QFileInfo(outObj).isFile()) {
    if (log) log->appendPlainText(
        QString("%1->obj: blender exited 0 but output missing").arg(ext));
    return QString();
  }
  if (log) log->appendPlainText(QString("%1->obj: ok").arg(ext));
  return outObj;
}

void VehicleImportPage::onBrowse() {
  const QString path = QFileDialog::getOpenFileName(
      this, "Select 3D Model",
      QStandardPaths::writableLocation(QStandardPaths::HomeLocation),
      "Supported 3D Models (*.obj *.gltf *.glb *.dae *.blend *.fbx);;"
      "OBJ (*.obj);;glTF (*.gltf *.glb);;Collada (*.dae);;Blender (*.blend);;FBX (*.fbx);;All files (*)");
  if (path.isEmpty()) return;
  const QString resolved = autoConvertBlendIfNeeded(path, mLog);
  loadMeshFromPath(resolved.isEmpty() ? path : resolved);
}

void VehicleImportPage::loadMeshFromPath(const QString &path) {
  if (path.isEmpty() || !QFileInfo(path).isFile()) return;
  mMeshPathEdit->setText(path);
  mImportBtn->setEnabled(false);
  mImportBtn->setStyleSheet(kImportButtonStyle);
  if (mDropBtn) {
    mDropBtn->setStyleSheet(kImportButtonStyle);
    mDropBtn->setEnabled(false);
  }
  if (mCalibrateBtn) {
    mCalibrateBtn->setStyleSheet(kImportButtonStyle);
    mCalibrateBtn->setEnabled(false);
  }
  if (mExportBtn) {
    mExportBtn->setStyleSheet(kImportButtonStyle);
    mExportBtn->setEnabled(false);
  }
  if (mAssetLabel) mAssetLabel->setVisible(false);
  if (mLastAsset)  mLastAsset->clear();

  const QString ext = QFileInfo(path).suffix().toLower();
  MeshAABB bb;
  if      (ext == "obj")                    bb = parseOBJ(path);
  else if (ext == "gltf" || ext == "glb")   bb = parseGLTF(path);
  else if (ext == "dae")                    bb = parseDAE(path);

  if (!bb.valid) {
    mLog->appendPlainText(QString("[%1]  Could not read geometry from %2").arg(stamp(), path));
    *mAabbValid = false;
    return;
  }

  if (ext == "obj") {
    if (bb.faceCount == 0) {
      mLog->appendPlainText(QString("[%1]  No usable faces - import disabled").arg(stamp()));
      mImportBtn->setEnabled(false);
      *mAabbValid = false;
      return;
    }
    if (bb.malformedFaceLines > 0) {
      mLog->appendPlainText(QString("[%1]  %2 malformed face line(s) will be skipped on import")
                              .arg(stamp()).arg(bb.malformedFaceLines));
    }
  }

  bb.detectConventions(ext);
  *mScaleToCm   = bb.scaleToCm;
  *mUpAxis      = bb.upAxis;
  *mForwardAxis = bb.forwardAxis;
  *mAabbValid   = true;

  float xLo, xHi, yLo, yHi, zLo, zHi;
  bb.toUE(xLo, xHi, yLo, yHi, zLo, zHi);

  mLog->appendPlainText(QString("[%1]  Bounding box: %2 × %3 × %4 cm  (×%5, up=%6)")
      .arg(stamp())
      .arg(xHi - xLo, 0, 'f', 0)
      .arg(yHi - yLo, 0, 'f', 0)
      .arg(zHi - zLo, 0, 'f', 0)
      .arg(bb.scaleToCm, 0, 'f', 0)
      .arg(QString("XYZ").at(bb.upAxis)));

  *mDetectedSpec = VehicleSpec{};
  bool meshDriven = false;
  if (ext == "obj") {
    const MeshGeometry g = loadMeshGeometryOBJ(path);
    if (g.valid) {
      const MeshAnalysisResult ar = analyzeMesh(g, bb.scaleToCm);
      if (ar.ok && ar.hasFourWheels) {
        *mDetectedSpec = buildSpecFromAnalysis(ar,
                                               mVehicleNameEdit->text().trimmed(),
                                               path, bb.scaleToCm,
                                               bb.upAxis, bb.forwardAxis);
        const auto applyRowFromWheel = [](WheelSpinRow &row, const WheelSpec &w) {
          row.x->setValue(w.x); row.y->setValue(w.y);
          row.z->setValue(w.z); row.r->setValue(w.radius);
        };
        applyRowFromWheel(mRowFL, mDetectedSpec->wheels[0]);
        applyRowFromWheel(mRowFR, mDetectedSpec->wheels[1]);
        applyRowFromWheel(mRowRL, mDetectedSpec->wheels[2]);
        applyRowFromWheel(mRowRR, mDetectedSpec->wheels[3]);
        SizeClass effClass = ar.sizeClass;
        const SizeClass nameHint = classifyByName(QFileInfo(path).completeBaseName());
        if (nameHint != SizeClass::Unknown) effClass = nameHint;
        const SizePreset preset = presetForSizeClass(effClass);
        mMassSpin->setValue(preset.mass);
        mSteerSpin->setValue(preset.maxSteerAngle);
        mBrakeSpin->setValue(preset.maxBrakeTorque);
        mSuspRaiseSpin->setValue(preset.suspMaxRaise);
        mSuspDropSpin->setValue(preset.suspMaxDrop);
        mSuspDampSpin->setValue(preset.suspDamping);
        QStringList parts;
        parts << QString("4 wheels (%1 cm radius)").arg(mDetectedSpec->wheels[0].radius, 0, 'f', 1);
        parts << QString("size class: %1%2")
                  .arg(sizeClassName(effClass),
                       nameHint != SizeClass::Unknown ? " (filename hint)" : "");
        parts << QString("mass %1 kg").arg(preset.mass, 0, 'f', 0);
        parts << QString("torque `%1`").arg(preset.torqueCurveTag);
        if (mDetectedSpec->smallVehicleNeedsWheelShape)
          parts << "small-WheelShape applied (Chaos < 18 cm fix)";
        mLog->appendPlainText(QString("[%1]  Detected: %2").arg(stamp(), parts.join("  ·  ")));
        meshDriven = true;
      }
    }
  }
  if (!meshDriven) {
    const auto wheels = guessWheels(xLo, xHi, yLo, yHi, zLo, zHi);
    const auto applyRowFromGuess = [](WheelSpinRow &r, const WheelGuess &w) {
      r.x->setValue(w.x); r.y->setValue(w.y);
      r.z->setValue(w.z); r.r->setValue(w.radius);
    };
    applyRowFromGuess(mRowFL, wheels[0]);
    applyRowFromGuess(mRowFR, wheels[1]);
    applyRowFromGuess(mRowRL, wheels[2]);
    applyRowFromGuess(mRowRR, wheels[3]);
    mLog->appendPlainText(QString(
        "[%1]  No wheel clusters auto-detected - wheel positions filled from bbox guess "
        "(mesh-driven analysis is .obj-only for now).").arg(stamp()));
  }

  {
    const QString sanitized = sanitizeVehicleName(QFileInfo(path).completeBaseName());
    if (!sanitized.isEmpty()) mVehicleNameEdit->setText(sanitized);
  }

  const QString autoPicked = pickClosestBaseVehicleBP(xHi - xLo);
  const QString lastAuto   = mBaseVehicleBPEdit->property("lastAutoPick").toString();
  const QString cur        = mBaseVehicleBPEdit->text().trimmed();
  if (cur.isEmpty() || cur == lastAuto) {
    mBaseVehicleBPEdit->setText(autoPicked);
    mBaseVehicleBPEdit->setProperty("lastAutoPick", autoPicked);
  }

  mImportBtn->setEnabled(true);
}

QJsonObject VehicleImportPage::buildImportSpec(const QString &meshPathToSend) const {
  auto wheelObj = [](const WheelSpinRow &r, double steer, double brake,
                     double raise, double drop) {
    QJsonObject o;
    o["x"]                = r.x->value();
    o["y"]                = r.y->value();
    o["z"]                = r.z->value();
    o["radius"]           = r.r->value();
    o["max_steer_angle"]  = steer;
    o["max_brake_torque"] = brake;
    o["susp_max_raise"]   = raise;
    o["susp_max_drop"]    = drop;
    return o;
  };
  QJsonObject spec;
  spec["vehicle_name"]    = sanitizeVehicleName(mVehicleNameEdit->text().trimmed());
  spec["mesh_path"]       = meshPathToSend;
  spec["content_path"]    = mContentPathEdit->text().trimmed();
  spec["base_vehicle_bp"] = mBaseVehicleBPEdit->text().trimmed();
  spec["mass"]            = mMassSpin->value();
  spec["susp_damping"]    = mSuspDampSpin->value();
  if (*mAabbValid) {
    spec["source_scale_to_cm"]  = *mScaleToCm;
    spec["source_up_axis"]      = *mUpAxis;
    spec["source_forward_axis"] = *mForwardAxis;
  }
  spec["wheel_fl"] = wheelObj(mRowFL, mSteerSpin->value(), mBrakeSpin->value(),
                              mSuspRaiseSpin->value(), mSuspDropSpin->value());
  spec["wheel_fr"] = wheelObj(mRowFR, mSteerSpin->value(), mBrakeSpin->value(),
                              mSuspRaiseSpin->value(), mSuspDropSpin->value());
  spec["wheel_rl"] = wheelObj(mRowRL, 0.0,                  mBrakeSpin->value(),
                              mSuspRaiseSpin->value(), mSuspDropSpin->value());
  spec["wheel_rr"] = wheelObj(mRowRR, 0.0,                  mBrakeSpin->value(),
                              mSuspRaiseSpin->value(), mSuspDropSpin->value());
  if (mDetectedSpec && mDetectedSpec->detectedFromMesh) {
    QJsonObject chassis;
    chassis["x_min"] = mDetectedSpec->chassisXMin;
    chassis["x_max"] = mDetectedSpec->chassisXMax;
    chassis["y_min"] = mDetectedSpec->chassisYMin;
    chassis["y_max"] = mDetectedSpec->chassisYMax;
    chassis["z_min"] = mDetectedSpec->chassisZMin;
    chassis["z_max"] = mDetectedSpec->chassisZMax;
    spec["chassis_aabb_cm"] = chassis;
  }
  return spec;
}

void VehicleImportPage::onImport() {
  const QString meshPath = mMeshPathEdit->text().trimmed();
  const QString name     = sanitizeVehicleName(mVehicleNameEdit->text().trimmed());
  if (meshPath.isEmpty() || name.isEmpty()) {
    mLog->appendPlainText("Error: vehicle name and mesh path are required.");
    return;
  }
  if (mVehicleNameEdit->text().trimmed() != name)
    mVehicleNameEdit->setText(name);


  QString meshToSend = meshPath;
  const QString tiresPath = mWheelsPathEdit ? mWheelsPathEdit->text().trimmed() : QString();
  if (!tiresPath.isEmpty()
      && QFileInfo(meshPath).suffix().toLower() == "obj"
      && QFileInfo(tiresPath).suffix().toLower() == "obj"
      && mDetectedSpec) {
    const QString outDir = QStandardPaths::writableLocation(QStandardPaths::TempLocation)
                         + "/carla_studio_merge";
    const ObjMergeResult mr = mergeBodyAndTires(meshPath, tiresPath, outDir, mDetectedSpec->wheels);
    if (mr.ok) {
      meshToSend = mr.outputPath;
      mLog->appendPlainText(QString("[%1]  Merged body+tires → %2 (%3)")
                              .arg(stamp(), meshToSend, mr.reason));
    } else {
      mLog->appendPlainText(QString("[%1]  Body+tires merge failed: %2 - sending body-only OBJ.")
                              .arg(stamp(), mr.reason));
    }
  }
  if (QFileInfo(meshToSend).suffix().toLower() == "obj") {
    const SanitizeReport rep = sanitizeOBJ(meshToSend, 0.0f);
    if (rep.ok) {
      meshToSend = rep.outputPath;
      mLog->appendPlainText(QString("[%1]  Sanitized OBJ → %2  (auto: ×%3 scale, swap_xy=%4, z_lift=%5cm, skipped %6 malformed face line(s))")
                              .arg(stamp()).arg(meshToSend)
                              .arg(rep.appliedScale, 0, 'f', 2)
                              .arg(rep.swapXY ? "true" : "false")
                              .arg(rep.zLiftCm, 0, 'f', 1)
                              .arg(rep.skippedFaceLines));
    }
  }

  const QJsonObject spec = buildImportSpec(meshToSend);

  mImportBtn->setEnabled(false);
  mProgress->setVisible(true);
  mProgress->setRange(0, 100);
  mProgress->setValue(0);
  mProgress->setFormat("starting…");
  mAssetLabel->clear();
  mAssetLabel->setVisible(false);

  QPointer<QPushButton>     btnPtr      = mImportBtn;
  QPointer<QPushButton>     dropPtr     = mDropBtn;
  QPointer<QLabel>          statusPtr   = mUeStatusLabel;
  QPointer<QPlainTextEdit>  logPtr      = mLog;
  QPointer<QProgressBar>    progressPtr = mProgress;
  QPointer<QLabel>          assetPtr    = mAssetLabel;
  std::shared_ptr<QString>  lastAsset   = mLastAsset;
  const QString    capturedName       = name;
  const QString    capturedSourceMesh = meshPath;
  QVector3D capturedFL, capturedFR, capturedRL, capturedRR;
  if (mDetectedSpec) {
    capturedFL = QVector3D(mDetectedSpec->wheels[0].x, mDetectedSpec->wheels[0].y, mDetectedSpec->wheels[0].z);
    capturedFR = QVector3D(mDetectedSpec->wheels[1].x, mDetectedSpec->wheels[1].y, mDetectedSpec->wheels[1].z);
    capturedRL = QVector3D(mDetectedSpec->wheels[2].x, mDetectedSpec->wheels[2].y, mDetectedSpec->wheels[2].z);
    capturedRR = QVector3D(mDetectedSpec->wheels[3].x, mDetectedSpec->wheels[3].y, mDetectedSpec->wheels[3].z);
  }

  auto sendNow = [=]() {
    if (!logPtr) return;
    logPtr->appendPlainText(QString("[%1]  Sending spec to UE Editor…").arg(stamp()));
    if (statusPtr) statusPtr->setText("UE Editor: sending …");
    if (progressPtr) {
      progressPtr->setRange(0, 0);
      progressPtr->setFormat("importing in UE Editor…");
    }
    Q_UNUSED(QtConcurrent::run([=]() {
      const QString response = sendJson(spec);
      QMetaObject::invokeMethod(qApp, [=]() {
        if (!logPtr) return;
        if (btnPtr) btnPtr->setEnabled(true);
        if (progressPtr) progressPtr->setRange(0, 100);
        if (response.isEmpty()) {
          if (statusPtr) {
            statusPtr->setText("UE Editor: connection failed");
            statusPtr->setStyleSheet("color: red;");
          }
          if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("connection failed"); }
          logPtr->appendPlainText(QString("[%1]  Error: no response from UE Editor on port %2.")
                                    .arg(stamp()).arg(kImporterPort));
          return;
        }
        const QJsonObject resp = QJsonDocument::fromJson(response.toUtf8()).object();
        const bool        ok   = resp.value("success").toBool();
        const QString     path = resp.value("asset_path").toString();
        const QString     err  = resp.value("error").toString();
        if (ok) {
          if (statusPtr) {
            statusPtr->setText("UE Editor: import succeeded");
            statusPtr->setStyleSheet("color: green;");
          }
          if (progressPtr) { progressPtr->setValue(100); progressPtr->setFormat("done (100%)"); }
          if (assetPtr) {
            assetPtr->setText(QString("Imported asset: %1").arg(path));
            assetPtr->setVisible(true);
          }
          *lastAsset = path;
          if (dropPtr) dropPtr->setEnabled(!path.isEmpty());
          if (mCalibrateBtn) mCalibrateBtn->setEnabled(!path.isEmpty());
          if (mExportBtn) mExportBtn->setEnabled(!path.isEmpty());
          if (btnPtr) {
            btnPtr->setStyleSheet(
              "QPushButton { padding: 4px 14px; font-weight: 600; min-width: 80px; "
              "background-color: #C8E6C9; color: #1B5E20; "
              "border: 1px solid #81C784; border-radius: 3px; }"
              "QPushButton:hover { background-color: #A5D6A7; }");
          }
          logPtr->appendPlainText(QString("[%1]  Success! Blueprint created at: %2").arg(stamp(), path));
#ifdef CARLA_STUDIO_WITH_QT3D
          {
            const QString canonical = QString("/tmp/vi_%1_body_canonical.obj").arg(capturedName);
            const QString showPath  = QFileInfo(canonical).exists() ? canonical : capturedSourceMesh;
            VehiclePreviewWindow::instance()->showFor(
                showPath, capturedFL, capturedFR, capturedRL, capturedRR);
            logPtr->appendPlainText(QString("[%1]  Preview opened (%2).").arg(stamp(), showPath));
          }
#endif
        } else {
          if (statusPtr) {
            statusPtr->setText("UE Editor: import failed");
            statusPtr->setStyleSheet("color: red;");
          }
          if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("failed"); }
          if (btnPtr) btnPtr->setStyleSheet(kImportButtonStyle);
          logPtr->appendPlainText(QString("[%1]  Failed: %2").arg(stamp(), err));
        }
      }, Qt::QueuedConnection);
    }));
  };

  const QString uprojectEarly = resolvedUproject();

  if (probeImporterPort()) {
    const EditorProcessInfo info = findEditorForUproject(uprojectEarly);
    if (info.exists && info.isHeadless && editorIsStale(info, uprojectEarly)) {
      if (logPtr) logPtr->appendPlainText(QString(
          "[%1]  Plugin libUnrealEditor-CarlaTools.so is newer than running editor "
          "(pid %2) - restarting before send.").arg(stamp()).arg(info.pid));
      if (progressPtr) { progressPtr->setRange(0, 0); progressPtr->setFormat("restarting stale editor…"); }
      killEditor(info.pid);
    } else {
      if (progressPtr) { progressPtr->setValue(50); progressPtr->setFormat("editor already up - sending"); }
      sendNow();
      return;
    }
  } else {
    const EditorProcessInfo orphan = findEditorForUproject(uprojectEarly);
    if (orphan.exists && orphan.isHeadless) {
      if (logPtr) logPtr->appendPlainText(QString(
          "[%1]  Found orphan headless editor pid %2 not bound to port %3 - killing before relaunch.")
          .arg(stamp()).arg(orphan.pid).arg(kImporterPort));
      killEditor(orphan.pid);
    } else if (orphan.exists && !orphan.isHeadless) {
      if (logPtr) logPtr->appendPlainText(QString(
          "[%1]  An interactive UnrealEditor (pid %2) is already running this project but the "
          "import server is not bound on port %3. Close that editor (or use it with the import "
          "plugin loaded) before retrying - refusing to spawn a duplicate.")
          .arg(stamp()).arg(orphan.pid).arg(kImporterPort));
      if (statusPtr) {
        statusPtr->setText("UE Editor: duplicate refused");
        statusPtr->setStyleSheet("color: red;");
      }
      if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("editor already running"); }
      if (btnPtr) btnPtr->setEnabled(true);
      return;
    }
  }

  const QString editorEarly = resolvedEditorBinary();
  QString enginePathEarly = mEnginePathEdit ? mEnginePathEdit->text().trimmed() : QString();
  if (enginePathEarly.isEmpty() && !editorEarly.isEmpty()) {
    QDir d(QFileInfo(editorEarly).absolutePath());
    for (int i = 0; i < 3 && !d.isRoot(); ++i) d.cdUp();
    enginePathEarly = d.absolutePath();
  }
  if (enginePathEarly.isEmpty()) {
    enginePathEarly = QString::fromLocal8Bit(qgetenv("CARLA_UNREAL_ENGINE_PATH")).trimmed();
  }

  if (uprojectEarly.isEmpty()) {
    if (logPtr) logPtr->appendPlainText(QString(
      "[%1]  No .uproject resolved - set CARLA_SRC_ROOT (with `export`!) or pick the .uproject "
      "via the … button next to Import.").arg(stamp()));
    if (statusPtr) {
      statusPtr->setText("UE Editor: no .uproject");
      statusPtr->setStyleSheet("color: red;");
    }
    if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("uproject not set"); }
    if (btnPtr) btnPtr->setEnabled(true);
    return;
  }
  if (enginePathEarly.isEmpty()) {
    if (logPtr) logPtr->appendPlainText(QString(
      "[%1]  No UE Engine root resolved - set CARLA_UNREAL_ENGINE_PATH (with `export`!) or pick "
      "the engine root via the … button next to Import.").arg(stamp()));
    if (statusPtr) {
      statusPtr->setText("UE Editor: no engine path");
      statusPtr->setStyleSheet("color: red;");
    }
    if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("engine path not set"); }
    if (btnPtr) btnPtr->setEnabled(true);
    return;
  }
  if (editorEarly.isEmpty()) {
    if (logPtr) logPtr->appendPlainText(QString(
      "[%1]  UnrealEditor binary not found at %2/Engine/Binaries/Linux/UnrealEditor - verify the engine path.")
      .arg(stamp(), enginePathEarly));
    if (statusPtr) {
      statusPtr->setText("UE Editor: binary missing");
      statusPtr->setStyleSheet("color: red;");
    }
    if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("editor binary missing"); }
    if (btnPtr) btnPtr->setEnabled(true);
    return;
  }

  const QString editor   = editorEarly;
  const QString uproject = uprojectEarly;
  const QString editorLog = QStandardPaths::writableLocation(QStandardPaths::TempLocation)
                          + "/carla_studio_ue_editor.log";
  auto shEsc = [](const QString &s) {
    QString out = s; out.replace('\'', "'\\''"); return "'" + out + "'";
  };
  const QString shellCmd = QString("exec %1 %2 -unattended -RenderOffScreen -nosplash -nosound >>%3 2>&1")
      .arg(shEsc(editor), shEsc(uproject), shEsc(editorLog));
  QStringList args;
  args << "-c" << shellCmd;
  qint64 pid = 0;
  if (!QProcess::startDetached("/bin/sh", args, QString(), &pid)) {
    if (logPtr) logPtr->appendPlainText(QString("[%1]  Could not start UE Editor: %2").arg(stamp(), editor));
    if (statusPtr) {
      statusPtr->setText("UE Editor: launch failed");
      statusPtr->setStyleSheet("color: red;");
    }
    if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("launch failed"); }
    if (btnPtr) btnPtr->setEnabled(true);
    return;
  }
  if (logPtr) logPtr->appendPlainText(QString("[%1]  Launched headless UE Editor (pid %2, stdout→%3) - waiting for VehicleImporter port %4.")
                                        .arg(stamp()).arg(pid).arg(editorLog).arg(kImporterPort));
  if (statusPtr) {
    statusPtr->setText("Processing …");
    statusPtr->setStyleSheet("color: #BB8800;");
  }
  if (progressPtr) {
    progressPtr->setValue(5);
    progressPtr->setRange(0, 0);
    progressPtr->setFormat("starting headless UE Editor…");
  }
  auto attempts = std::make_shared<int>(0);
  const qint64 launchedPid = pid;
  auto *poll = new QTimer(this);
  poll->setInterval(1000);
  connect(poll, &QTimer::timeout, this, [=]() {
    ++*attempts;
    if (probeImporterPort()) {
      poll->stop(); poll->deleteLater();
      if (logPtr) logPtr->appendPlainText(QString("[%1]  Editor ready after %2 s (port %3 open).")
                                            .arg(stamp()).arg(*attempts).arg(kImporterPort));
      if (progressPtr) {
        progressPtr->setRange(0, 100);
        progressPtr->setValue(50);
        progressPtr->setFormat("editor ready - sending spec");
      }
      sendNow();
      return;
    }
    const bool alive = launchedPid > 0 && QFileInfo(QString("/proc/%1").arg(launchedPid)).exists();
    if (!alive) {
      poll->stop(); poll->deleteLater();
      if (logPtr) logPtr->appendPlainText(QString("[%1]  UE Editor process %2 exited after %3 s before opening port %4 - check the editor log under <project>/Saved/Logs/ for the reason.")
                                            .arg(stamp()).arg(launchedPid).arg(*attempts).arg(kImporterPort));
      if (statusPtr) {
        statusPtr->setText("UE Editor: process died");
        statusPtr->setStyleSheet("color: red;");
      }
      if (progressPtr) {
        progressPtr->setRange(0, 100);
        progressPtr->setValue(0);
        progressPtr->setFormat("editor exited");
      }
      if (btnPtr) btnPtr->setEnabled(true);
      return;
    }
    if (*attempts == 30 || *attempts == 120 || *attempts == 300 ||
        (*attempts > 300 && *attempts % 300 == 0)) {
      if (logPtr) logPtr->appendPlainText(QString("[%1]  Still waiting - UE Editor pid %2 alive, port %3 not yet open (%4 s elapsed).")
                                            .arg(stamp()).arg(launchedPid).arg(kImporterPort).arg(*attempts));
    }
  });
  poll->start();
}

void VehicleImportPage::onDrop() {
  const QString assetPath = *mLastAsset;
  if (assetPath.isEmpty()) {
    mLog->appendPlainText(QString("[%1]  Drive in CARLA: no imported asset yet - run Import first.").arg(stamp()));
    return;
  }
  const QString uproject = resolvedUproject();
  if (uproject.isEmpty()) {
    mLog->appendPlainText(QString("[%1]  Drive in CARLA: uproject path unresolved - set CARLA_SRC_ROOT or pick the .uproject above.").arg(stamp()));
    return;
  }
  const QString name = sanitizeVehicleName(mVehicleNameEdit->text().trimmed());
  if (name.isEmpty()) return;
  if (mVehicleNameEdit->text().trimmed() != name)
    mVehicleNameEdit->setText(name);

  VehicleRegistration reg;
  reg.uprojectPath      = uproject;
  reg.shippingCarlaRoot = mFindCarlaRoot ? mFindCarlaRoot() : QString();
  reg.bpAssetPath       = assetPath;
  reg.make              = "Custom";
  reg.model             = name;
  reg.editorBinary      = resolvedEditorBinary();

  mDropBtn->setEnabled(false);
  mLog->appendPlainText(QString("[%1]  Drive in CARLA: registering %2 in VehicleParameters.json …")
                          .arg(stamp(), assetPath));

  QPointer<QPushButton>    btnPtr    = mDropBtn;
  QPointer<QLabel>         statusPtr = mUeStatusLabel;
  QPointer<QPlainTextEdit> logPtr    = mLog;
  const QString            make      = reg.make;
  const QString            model     = reg.model;

  QPointer<QProgressBar> progPtr = mProgress;
  Q_UNUSED(QtConcurrent::run([=]() {
    RegisterResult rr;
    DeployResult   dr;
    SpawnResult    sr;
    QString        unhandled;
    try {
      rr = registerVehicleInJson(reg);
      if (rr.ok) {
        const EditorProcessInfo importEd = findEditorForUproject(uproject);
        if (importEd.exists && importEd.isHeadless) {
          QMetaObject::invokeMethod(qApp, [logPtr, pid = importEd.pid]() {
            if (logPtr) logPtr->appendPlainText(QString(
                "[%1]  Killing import editor (pid %2) before cook to free RAM/GPU + clear locks.")
                  .arg(stamp()).arg(pid));
          }, Qt::QueuedConnection);
          killEditor(importEd.pid);
        }
        QMetaObject::invokeMethod(qApp, [logPtr, progPtr]() {
          if (logPtr) logPtr->appendPlainText(QString(
            "[%1]  Cooking assets for shipping CARLA - first cook is 2-3 min "
            "(shader compile), subsequent cooks ~30 s. UE editor commandlet running…")
            .arg(stamp()));
          if (progPtr) {
            progPtr->setVisible(true);
            progPtr->setRange(0, 0);
            progPtr->setFormat("cooking assets for shipping CARLA…");
          }
        }, Qt::QueuedConnection);
        dr = deployVehicleToShippingCarla(reg);
        QMetaObject::invokeMethod(qApp, [progPtr, ok = dr.ok]() {
          if (progPtr) {
            progPtr->setRange(0, 100);
            progPtr->setValue(ok ? 100 : 0);
            progPtr->setFormat(ok ? "cook + deploy done" : "cook failed");
          }
        }, Qt::QueuedConnection);
      }
      if (rr.ok && dr.ok) {
        QMetaObject::invokeMethod(qApp, [progPtr]() {
          if (progPtr) {
            progPtr->setRange(0, 0);
            progPtr->setFormat("starting CARLA + spawning…");
          }
        }, Qt::QueuedConnection);
        sr = spawnInRunningCarla(make, model);
      }
    } catch (const std::exception &e) {
      unhandled = QString("std::exception: %1").arg(QString::fromUtf8(e.what()));
    } catch (...) {
      unhandled = "unknown C++ exception";
    }

    QMetaObject::invokeMethod(qApp, [=]() {
      if (!unhandled.isEmpty()) {
        if (logPtr) logPtr->appendPlainText(QString(
          "[%1]  Drive: caught %2 - likely API mismatch between this Studio's "
          "libcarla-client and the running CARLA. Drive aborted; Import side is unaffected.")
          .arg(stamp(), unhandled));
        if (statusPtr) {
          statusPtr->setText("Drive: aborted (LibCarla error)");
          statusPtr->setStyleSheet("color: red;");
        }
        if (btnPtr) btnPtr->setEnabled(true);
        return;
      }
      if (!logPtr) return;
      if (btnPtr) btnPtr->setEnabled(true);
      logPtr->appendPlainText(QString("[%1]  Registration (dev tree): %2  (%3)")
                                .arg(stamp()).arg(rr.ok ? "OK" : "FAIL").arg(rr.detail));
      logPtr->appendPlainText(QString("[%1]  Deploy to shipping CARLA: %2  (%3)")
                                .arg(stamp()).arg(dr.ok ? "OK" : "SKIP/FAIL").arg(dr.detail));
      if (dr.ok && !dr.destDir.isEmpty()) {
        const QString archiveRoot = QStandardPaths::writableLocation(QStandardPaths::HomeLocation)
                                  + "/.carla_studio/archive/" + model;
        QDir().mkpath(archiveRoot);
        QDir srcD(dr.destDir);
        const QStringList files = srcD.entryList({"*.uasset", "*.uexp"}, QDir::Files);
        int archived = 0;
        for (const QString &f : files) {
          const QString s = dr.destDir + "/" + f;
          const QString t = archiveRoot + "/" + f;
          if (QFile::exists(t)) QFile::remove(t);
          if (QFile::copy(s, t)) ++archived;
        }
        logPtr->appendPlainText(QString("[%1]  Installed at: %2").arg(stamp(), dr.destDir));
        logPtr->appendPlainText(QString("[%1]  Archived %2 file(s) → %3").arg(stamp())
                                  .arg(archived).arg(archiveRoot));
      }
      if (!rr.ok) {
        if (statusPtr) {
          statusPtr->setText("CARLA: registration failed");
          statusPtr->setStyleSheet("color: red;");
        }
        return;
      }
      switch (sr.kind) {
        case SpawnResult::Kind::Spawned:
          if (statusPtr) {
            statusPtr->setText("Drive: spawned + autopilot ON");
            statusPtr->setStyleSheet("color: green;");
          }
          if (mDropBtn) {
            mDropBtn->setStyleSheet(
              "QPushButton { padding: 4px 14px; font-weight: 600; min-width: 80px; "
              "background-color: #FFF59D; color: #6A4F00; "
              "border: 1px solid #FFD54F; border-radius: 3px; }"
              "QPushButton:hover { background-color: #FFE082; }");
          }
          logPtr->appendPlainText(QString("[%1]  Spawn: OK - %2").arg(stamp(), sr.detail));
          break;
        case SpawnResult::Kind::BlueprintNotFound:
          if (statusPtr) {
            statusPtr->setText("CARLA: restart needed");
            statusPtr->setStyleSheet("color: #BB8800;");
          }
          logPtr->appendPlainText(QString("[%1]  Spawn: %2").arg(stamp(), sr.detail));
          break;
        case SpawnResult::Kind::NoSimulator:
          if (statusPtr) {
            statusPtr->setText("CARLA: starting simulator …");
            statusPtr->setStyleSheet("color: #BB8800;");
          }
          logPtr->appendPlainText(QString("[%1]  Spawn: %2").arg(stamp(), sr.detail));
          if (dr.ok && mRequestStartCarla) {
            logPtr->appendPlainText(QString(
              "[%1]  Auto-pressing Studio's START button - will retry spawn once the simulator "
              "is up on port 2000.").arg(stamp()));
            mRequestStartCarla();
            auto attempts = std::make_shared<int>(0);
            auto *poll = new QTimer(this);
            poll->setInterval(2000);
            QPointer<QPlainTextEdit> logL = logPtr;
            QPointer<QLabel>         stL  = statusPtr;
            QPointer<QPushButton>    btnL = btnPtr;
            const QString            mk   = make;
            const QString            md   = model;
            auto probePort2000 = []() {
              int s = ::socket(AF_INET, SOCK_STREAM, 0);
              if (s < 0) return false;
              sockaddr_in addr{};
              addr.sin_family = AF_INET;
              addr.sin_port   = htons(2000);
              ::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
              const bool up = ::connect(s, (sockaddr*)&addr, sizeof(addr)) == 0;
              ::close(s);
              return up;
            };
            connect(poll, &QTimer::timeout, this, [=]() {
              ++*attempts;
              const bool up = probePort2000();
              if (up) {
                poll->stop(); poll->deleteLater();
                if (logL) logL->appendPlainText(QString("[%1]  CARLA simulator is up after %2 s - retrying spawn.")
                                                  .arg(stamp()).arg(*attempts * 2));
                Q_UNUSED(QtConcurrent::run([=]() {
                  SpawnResult sr2;
                  try { sr2 = spawnInRunningCarla(mk, md); }
                  catch (...) { sr2.detail = "second-pass spawn threw"; }
                  QMetaObject::invokeMethod(qApp, [=]() {
                    if (logL) logL->appendPlainText(QString("[%1]  Retry spawn: %2 - %3")
                                                      .arg(stamp())
                                                      .arg(sr2.kind == SpawnResult::Kind::Spawned ? "OK" : "FAIL")
                                                      .arg(sr2.detail));
                    if (stL) {
                      if (sr2.kind == SpawnResult::Kind::Spawned) {
                        stL->setText("CARLA: spawned");
                        stL->setStyleSheet("color: green;");
                      } else {
                        stL->setText("CARLA: spawn failed");
                        stL->setStyleSheet("color: red;");
                      }
                    }
                    if (btnL) btnL->setEnabled(true);
                  }, Qt::QueuedConnection);
                }));
                return;
              }
              if (*attempts == 15 || *attempts == 60 || *attempts == 150) {
                if (logL) logL->appendPlainText(QString("[%1]  Still waiting for CARLA on port 2000 (%2 s).")
                                                  .arg(stamp()).arg(*attempts * 2));
              }
              if (*attempts >= 300) {  // 10 min cap
                poll->stop(); poll->deleteLater();
                if (logL) logL->appendPlainText(QString("[%1]  CARLA simulator did not come up within 10 min - giving up.").arg(stamp()));
                if (stL) {
                  stL->setText("CARLA: start timeout");
                  stL->setStyleSheet("color: red;");
                }
                if (btnL) btnL->setEnabled(true);
              }
            });
            poll->start();
            return;
          } else if (dr.ok) {
            logPtr->appendPlainText(QString(
              "[%1]  Vehicle is now on disk in the shipping CARLA tree. Start CARLA via "
              "Studio's START button, then click Drive again to spawn it.").arg(stamp()));
          }
          break;
        case SpawnResult::Kind::Failed:
          if (sr.detail.isEmpty()) {
            if (statusPtr) {
              statusPtr->setText("Drive: deployed to CARLA");
              statusPtr->setStyleSheet("color: green;");
            }
            if (dr.ok) {
              logPtr->appendPlainText(QString(
                "[%1]  Vehicle deployed. Could not auto-spawn - start CARLA and re-click Drive.")
                .arg(stamp()));
            }
          } else {
            if (statusPtr) {
              statusPtr->setText("Drive: spawn failed");
              statusPtr->setStyleSheet("color: red;");
            }
            logPtr->appendPlainText(QString("[%1]  Spawn: FAIL - %2").arg(stamp(), sr.detail));
          }
          break;
      }
    }, Qt::QueuedConnection);
  }));
}

void VehicleImportPage::onExport() {
  if (!mLastAsset || mLastAsset->isEmpty()) {
    if (mLog) mLog->appendPlainText(QString("[%1]  Export: no imported asset yet - run Import first.").arg(stamp()));
    return;
  }
  const QString name = sanitizeVehicleName(mVehicleNameEdit->text().trimmed());
  if (name.isEmpty()) return;

  const QString carlaRoot = mFindCarlaRoot ? mFindCarlaRoot() : QString();
  const QString uproject  = resolvedUproject();

  VehicleRegistration reg;
  reg.uprojectPath      = uproject;
  reg.shippingCarlaRoot = carlaRoot;
  reg.bpAssetPath       = *mLastAsset;
  reg.make              = "Custom";
  reg.model             = name;
  reg.editorBinary      = resolvedEditorBinary();

  if (mExportBtn) mExportBtn->setEnabled(false);
  if (mLog) mLog->appendPlainText(QString("[%1]  Export: cooking + bundling %2 …").arg(stamp(), name));

  QPointer<QPushButton>    btnPtr  = mExportBtn;
  QPointer<QPlainTextEdit> logPtr  = mLog;
  const QString            engineRoot = qEnvironmentVariable("CARLA_UNREAL_ENGINE_PATH");

  Q_UNUSED(QtConcurrent::run([=]() {
    DeployResult dr;
    QString unhandled;
    try {
      const EditorProcessInfo importEd = findEditorForUproject(uproject);
      if (importEd.exists && importEd.isHeadless) {
        QMetaObject::invokeMethod(qApp, [logPtr, pid = importEd.pid]() {
          if (logPtr) logPtr->appendPlainText(QString(
              "[%1]  Killing import editor (pid %2) before cook.")
                .arg(stamp()).arg(pid));
        }, Qt::QueuedConnection);
        killEditor(importEd.pid);
      }
      dr = deployVehicleToShippingCarla(reg);
    } catch (const std::exception &e) {
      unhandled = QString("std::exception: %1").arg(QString::fromUtf8(e.what()));
    } catch (...) {
      unhandled = "unknown C++ exception";
    }

    QString zipPath, zipDetail;
    bool zipOk = false;
    if (dr.ok && !dr.destDir.isEmpty()) {
      int ueMajor = 5;
      if (engineRoot.contains("Unreal4") || engineRoot.contains("UE4")) ueMajor = 4;
      const QString date = QDateTime::currentDateTime().toString("MMddyyyy");
      const QString outDir = QStandardPaths::writableLocation(QStandardPaths::HomeLocation)
                           + "/.carla_studio/exports";
      QDir().mkpath(outDir);
      zipPath = QString("%1/carla_%2_ue%3_%4.zip").arg(outDir, name).arg(ueMajor).arg(date);
      QFile::remove(zipPath);
      QProcess proc;
      proc.setWorkingDirectory(QFileInfo(dr.destDir).absolutePath());
      const QString folderName = QFileInfo(dr.destDir).fileName();
      proc.start("zip", QStringList() << "-r" << "-q" << zipPath << folderName);
      if (proc.waitForFinished(120000) && proc.exitCode() == 0
          && QFileInfo(zipPath).size() > 0) {
        zipOk = true;
        zipDetail = QString("%1 bytes").arg(QFileInfo(zipPath).size());
      } else {
        zipDetail = QString("zip exit=%1: %2")
            .arg(proc.exitCode())
            .arg(QString::fromUtf8(proc.readAllStandardError()).trimmed());
      }
    }

    QMetaObject::invokeMethod(qApp, [=]() {
      if (btnPtr) btnPtr->setEnabled(true);
      if (!logPtr) return;
      if (!unhandled.isEmpty()) {
        logPtr->appendPlainText(QString("[%1]  Export: deploy aborted - %2").arg(stamp(), unhandled));
        return;
      }
      logPtr->appendPlainText(QString("[%1]  Export: deploy %2  (%3)")
                                .arg(stamp()).arg(dr.ok ? "OK" : "FAIL").arg(dr.detail));
      if (zipOk) {
        logPtr->appendPlainText(QString("[%1]  Export: archive → %2  (%3)").arg(stamp(), zipPath, zipDetail));
        if (mExportBtn) mExportBtn->setStyleSheet(kSuccessButtonStyle);
      } else if (dr.ok) {
        logPtr->appendPlainText(QString("[%1]  Export: zip failed - %2").arg(stamp(), zipDetail));
      }
    }, Qt::QueuedConnection);
  }));
}

}  // namespace carla_studio::vehicle_import
