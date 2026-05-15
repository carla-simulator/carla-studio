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

struct WheelGuess { float x, y, z, radius; };

std::array<WheelGuess, 4> guessWheels(float x_min, float x_max,
                                      float y_min, float y_max,
                                      float z_min, float z_max) {
  const float cx     = (x_min + x_max) * 0.5f;
  const float halfW  = (y_max - y_min) * 0.5f;
  const float axleY  = halfW * 0.8f;
  const float radius = std::max(5.f, (z_max - z_min) * 0.18f);
  const float axleZ  = z_min + radius;
  const float frontX = cx + (x_max - cx) * 0.60f;
  const float rearX  = cx - (cx - x_min) * 0.60f;
  return {{
    { frontX,  axleY, axleZ, radius },
    { frontX, -axleY, axleZ, radius },
    { rearX,   axleY, axleZ, radius },
    { rearX,  -axleY, axleZ, radius },
  }};
}

}

VehicleImportPage::VehicleImportPage(EditorBinaryResolver find_editor,
                                     UprojectResolver     find_uproject,
                                     CarlaRootResolver    find_carla_root,
                                     StartCarlaRequester  request_start_carla,
                                     QWidget *parent)
    : QWidget(parent),
      m_find_editor(std::move(find_editor)),
      m_find_uproject(std::move(find_uproject)),
      m_find_carla_root(std::move(find_carla_root)),
      m_request_start_carla(std::move(request_start_carla)) {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(12, 12, 12, 12);
  root->setSpacing(6);

  m_engine_path_edit = new QLineEdit(this); m_engine_path_edit->setVisible(false);
  m_uproject_edit   = new QLineEdit(this); m_uproject_edit  ->setVisible(false);

  auto *fileRow = new QHBoxLayout();
  auto *vehLbl = new QLabel("Vehicle:");
  vehLbl->setFixedWidth(60);
  m_mesh_path_edit = new QLineEdit();
  m_mesh_path_edit->setPlaceholderText("path to .obj / .fbx / .glb / .dae …");
  m_mesh_path_edit->setReadOnly(true);
  m_browse_btn = new QPushButton("Browse…");
  auto *vehTrailer = new QLabel("(*.obj *.fbx *.glb *.dae)");
  vehTrailer->setStyleSheet("color: #888;");
  const int kTrailerWidth =
      QFontMetrics(vehTrailer->font()).horizontalAdvance(vehTrailer->text()) + 8;
  vehTrailer->setFixedWidth(kTrailerWidth);
  fileRow->addWidget(vehLbl);
  fileRow->addWidget(m_mesh_path_edit, 1);
  fileRow->addWidget(m_browse_btn);
  fileRow->addWidget(vehTrailer);
  root->addLayout(fileRow);

  auto *wheels_file_row = new QHBoxLayout();
  auto *tire_lbl = new QLabel("Tires:");
  tire_lbl->setFixedWidth(60);
  m_wheels_path_edit = new QLineEdit();
  m_wheels_path_edit->setPlaceholderText("optional");
  m_wheels_path_edit->setReadOnly(true);
  m_wheels_browse_btn = new QPushButton("Browse…");
  m_wheels_hint_label = new QLabel("(optional)");
  m_wheels_hint_label->setStyleSheet("color: #888;");
  m_wheels_hint_label->setFixedWidth(kTrailerWidth);
  wheels_file_row->addWidget(tire_lbl);
  wheels_file_row->addWidget(m_wheels_path_edit, 1);
  wheels_file_row->addWidget(m_wheels_browse_btn);
  wheels_file_row->addWidget(m_wheels_hint_label);
  root->addLayout(wheels_file_row);

  m_aabb_label       = new QLabel(this); m_aabb_label->setVisible(false);
  m_detection_label  = new QLabel(this); m_detection_label->setVisible(false);
  m_validation_label = new QLabel(this); m_validation_label->setVisible(false);

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
      sb->setRange(static_cast<double>(lo), static_cast<double>(hi));
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
  m_row_fl = makeWheelRow("Front Left");
  m_row_fr = makeWheelRow("Front Right");
  m_row_rl = makeWheelRow("Rear Left");
  m_row_rr = makeWheelRow("Rear Right");
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

  m_vehicle_name_edit = new QLineEdit();
  m_vehicle_name_edit->setPlaceholderText("e.g. my_truck (lowercase letters, digits, underscores)");
  m_vehicle_name_edit->setValidator(new VehicleNameValidator(m_vehicle_name_edit));
  m_vehicle_name_edit->setMaxLength(kMaxVehicleNameLength);
  QObject::connect(m_vehicle_name_edit, &QLineEdit::editingFinished, m_vehicle_name_edit, [edit = m_vehicle_name_edit]() {
    const QString fixed = sanitize_vehicle_name(edit->text());
    if (fixed != edit->text()) edit->setText(fixed);
  });
  paramsForm->addRow("Vehicle Name", m_vehicle_name_edit);

  m_content_path_edit = new QLineEdit("/Game/Carla/Static/Vehicles/4Wheeled");
  paramsForm->addRow("UE Content Path", m_content_path_edit);

  m_base_vehicle_bp_edit = new QLineEdit(kDefaultBaseVehicleBP);
  m_base_vehicle_bp_edit->setProperty("lastAutoPick", QString(kDefaultBaseVehicleBP));
  m_base_vehicle_bp_edit->setToolTip(
    "Existing CARLA vehicle Blueprint to use as the rigging template.\n"
    "Defaults to BaseUSDImportVehicle - the only stock BP with a real "
    "USkeletalMesh + PhysicsAsset that the importer can consume.");
  paramsForm->addRow("Base Vehicle BP", m_base_vehicle_bp_edit);

  const QSizePolicy expandH(QSizePolicy::Expanding, QSizePolicy::Fixed);
  m_vehicle_name_edit  ->setSizePolicy(expandH);
  m_vehicle_name_edit  ->setAlignment(Qt::AlignRight);
  m_content_path_edit  ->setSizePolicy(expandH);
  m_content_path_edit  ->setAlignment(Qt::AlignRight);
  m_base_vehicle_bp_edit->setSizePolicy(expandH);
  m_base_vehicle_bp_edit->setAlignment(Qt::AlignRight);

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
  m_mass_spin      = makeSpin(100,  100000, 1500, "kg",   2);
  m_steer_spin     = makeSpin(0,    90,     70,   "°",    2);
  m_susp_raise_spin = makeSpin(0,    100,    10,   "cm",   2);
  m_susp_drop_spin  = makeSpin(0,    100,    10,   "cm",   2);
  m_susp_damp_spin  = makeSpin(0.1,  2.0,    0.65, "",     2);
  m_susp_damp_spin->setSingleStep(0.05);
  m_brake_spin     = makeSpin(0,    10000,  1500, "Nm",   2);
  const QString kDriveabilityTip =
      "* Driveability-critical: differs per vehicle (sedan vs SUV vs truck). "
      "If the imported vehicle won't drive forward, try adjusting these. "
      "There is no auto-tune; physics depends on body geometry.";
  auto markDriveability = [&](QDoubleSpinBox *sb) {
    sb->setToolTip(kDriveabilityTip + (sb->toolTip().isEmpty() ? QString()
                                       : "\n" + sb->toolTip()));
  };
  markDriveability(m_mass_spin);
  markDriveability(m_susp_raise_spin);
  markDriveability(m_susp_drop_spin);
  markDriveability(m_susp_damp_spin);
  markDriveability(m_brake_spin);
  paramsForm->addRow("* Vehicle Mass",         m_mass_spin);
  paramsForm->addRow("Max Steer Angle",        m_steer_spin);
  paramsForm->addRow("* Suspension Max Raise", m_susp_raise_spin);
  paramsForm->addRow("* Suspension Max Drop",  m_susp_drop_spin);
  paramsForm->addRow("* Suspension Damping",   m_susp_damp_spin);
  paramsForm->addRow("* Max Brake Torque",     m_brake_spin);
  auto *driveabilityNote = new QLabel(
      "<i>* Driveability-critical fields. Tune per-vehicle if "
      "the import doesn't drive forward; defaults are sedan-tuned.</i>");
  driveabilityNote->setWordWrap(true);
  driveabilityNote->setStyleSheet("color: #888; font-size: 11px;");
  paramsForm->addRow(driveabilityNote);
  paramsGroup->setLayout(paramsForm);
  root->addWidget(paramsGroup);

  auto *importRow = new QHBoxLayout();
  m_ue_status_label = new QLabel("UE Editor: not connected");
  m_ue_status_label->setStyleSheet("color: gray;");

  const QString envEngineHint =
      QString::fromLocal8Bit(qgetenv("CARLA_UNREAL_ENGINE_PATH")).trimmed();
  const QString autoUprojHint = m_find_uproject ? m_find_uproject() : QString();

  m_engine_browse_btn = new QPushButton("Unreal Editor");
  m_engine_browse_btn->setMinimumHeight(28);
  m_engine_browse_btn->setMinimumWidth(120);

  m_uproject_browse_btn = new QPushButton("Carla built from source");
  m_uproject_browse_btn->setMinimumHeight(28);
  m_uproject_browse_btn->setMinimumWidth(140);

  m_import_btn = new QPushButton("Import");
  m_import_btn->setEnabled(false);
  m_import_btn->setMinimumWidth(96);
  m_import_btn->setToolTip(
    "Run the import end-to-end:\n"
    "  • Spawn a headless UE Editor in the background.\n"
    "  • Wait for the VehicleImporter port to come up.\n"
    "  • Send the spec, import the mesh, build wheel + vehicle BPs.");

  m_drop_btn = new QPushButton("Drive");
  m_drop_btn->setEnabled(false);
  m_drop_btn->setMinimumWidth(110);
  m_drop_btn->setToolTip(
    "Register the imported vehicle in VehicleParameters.json, spawn it in the\n"
    "running CARLA simulator, and hand control to the Traffic Manager (SAE L5 -\n"
    "the vehicle drives itself around the city). Requires CARLA started via the\n"
    "Studio START button. If the blueprint isn't yet visible to CARLA, restart\n"
    "the simulator to pick up the new registration.");

  m_mode_banner = new QLabel(this);
  m_mode_banner->setVisible(false);

  m_export_btn = new QPushButton("Export");
  m_export_btn->setEnabled(false);
  m_export_btn->setMinimumWidth(96);
  m_export_btn->setToolTip(
      "Bundle the cooked + driveable vehicle into a single .zip archive named\n"
      "carla_<vehicle>_ue<4|5>_<MMDDYYYY>.zip under ~/.carla_studio/exports/.\n"
      "Useful for redistributing or installing into another CARLA root via the\n"
      "(former) Pre-built Package install.");
  connect(m_export_btn, &QPushButton::clicked, this, &VehicleImportPage::on_export);

  auto *prereqRow = new QHBoxLayout();
  prereqRow->setSpacing(8);
  m_engine_browse_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  m_uproject_browse_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  prereqRow->addWidget(m_engine_browse_btn,   1);
  prereqRow->addWidget(m_uproject_browse_btn, 1);
  root->addLayout(prereqRow);

  m_calibrate_btn = new QPushButton("Calibrate");
  m_calibrate_btn->setEnabled(false);
  m_calibrate_btn->setMinimumWidth(96);
  m_calibrate_btn->setToolTip(
      "Open the Vehicle Preview window for the current import - verify\n"
      "orientation on the calibration grid; use Rot/Mirror buttons + Apply &\n"
      "Re-import if the auto-detected forward axis is wrong.");
  connect(m_calibrate_btn, &QPushButton::clicked, this, [this]() {
#ifdef CARLA_STUDIO_WITH_QT3D
    if (!m_last_asset || m_last_asset->isEmpty()) {
      if (m_log) m_log->appendPlainText(QString("[%1]  Calibrate: no imported asset - run Import first.").arg(stamp()));
      return;
    }
    const QString name = sanitize_vehicle_name(m_vehicle_name_edit->text().trimmed());
    const QString canonical = QString("/tmp/vi_%1_body_canonical.obj").arg(name);
    const QString show_path  = QFileInfo(canonical).exists()
        ? canonical : m_mesh_path_edit->text().trimmed();
    QVector3D fl, fr, rl, rr;
    if (m_detected_spec) {
      fl = QVector3D(m_detected_spec->wheels[0].x, m_detected_spec->wheels[0].y, m_detected_spec->wheels[0].z);
      fr = QVector3D(m_detected_spec->wheels[1].x, m_detected_spec->wheels[1].y, m_detected_spec->wheels[1].z);
      rl = QVector3D(m_detected_spec->wheels[2].x, m_detected_spec->wheels[2].y, m_detected_spec->wheels[2].z);
      rr = QVector3D(m_detected_spec->wheels[3].x, m_detected_spec->wheels[3].y, m_detected_spec->wheels[3].z);
    }
    VehiclePreviewWindow::instance()->show_for(show_path, fl, fr, rl, rr);
#endif
  });

  importRow->setSpacing(8);
  for (QPushButton *b : { m_import_btn, m_calibrate_btn, m_drop_btn, m_export_btn }) {
    b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    b->setMinimumWidth(110);
    b->setStyleSheet(kImportButtonStyle);
  }
  importRow->addWidget(m_import_btn,    1);
  importRow->addWidget(m_calibrate_btn, 1);
  importRow->addWidget(m_drop_btn,      1);
  importRow->addWidget(m_export_btn,    1);
  root->addLayout(importRow);

  m_progress = new QProgressBar();
  m_progress->setRange(0, 100);
  m_progress->setValue(0);
  m_progress->setTextVisible(true);
  m_progress->setFormat("idle");
  m_progress->setVisible(false);
  root->addWidget(m_progress);

  m_asset_label = new QLabel();
  m_asset_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  m_asset_label->setVisible(false);
  m_asset_label->setStyleSheet("color: #2E7D32; font-weight: 600;");
  root->addWidget(m_asset_label);

  m_log = new QPlainTextEdit();
  m_log->setReadOnly(true);
  m_log->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  m_log->setMinimumHeight(160);
  m_log->setPlaceholderText("Import log will appear here…");
  root->addWidget(m_log, 1);
  refresh_dep_check_row();

  auto *footerRow = new QHBoxLayout();
  footerRow->setContentsMargins(0, 0, 0, 0);
  m_ue_status_label->setVisible(true);
  m_ue_status_label->setStyleSheet("color: #555; font-size: 11px;");
  m_ue_status_label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  footerRow->addWidget(m_ue_status_label, 1, Qt::AlignLeft);
  root->addLayout(footerRow);

  apply_disabled_state_styling();

  connect(m_browse_btn,         &QPushButton::clicked, this, &VehicleImportPage::on_browse);
  connect(m_import_btn,         &QPushButton::clicked, this, &VehicleImportPage::on_import);
  connect(m_drop_btn,           &QPushButton::clicked, this, &VehicleImportPage::on_drop);
  connect(m_wheels_browse_btn,   &QPushButton::clicked, this, &VehicleImportPage::on_browse_wheels);
  connect(m_engine_browse_btn,   &QPushButton::clicked, this, &VehicleImportPage::on_browse_engine_path);
  connect(m_uproject_browse_btn, &QPushButton::clicked, this, &VehicleImportPage::on_browse_uproject);

#ifdef CARLA_STUDIO_WITH_QT3D
  connect(VehiclePreviewWindow::instance(), &QDialog::finished, this,
          [this](int) {
            if (m_calibrate_btn && m_calibrate_btn->isEnabled())
              m_calibrate_btn->setStyleSheet(kSuccessButtonStyle);
          });

  if (auto *page = VehiclePreviewWindow::instance()->page()) {
    connect(page, &VehiclePreviewPage::apply_requested, this,
            [this](float yaw_deg, float mirror_x, float mirror_y) {
              if (!m_detected_spec) {
                if (m_log) m_log->appendPlainText(
                    QString("[%1]  Apply: no spec to update.").arg(stamp()));
                return;
              }
              m_detected_spec->adjust_yaw_deg  = yaw_deg;
              m_detected_spec->adjust_mirror_x = mirror_x;
              m_detected_spec->adjust_mirror_y = mirror_y;
              if (m_log) m_log->appendPlainText(QString(
                  "[%1]  Apply calibration → spec (yaw=%2°, mirror=(%3,%4)) - re-importing.")
                    .arg(stamp()).arg(yaw_deg, 0, 'f', 0)
                    .arg(mirror_x > 0 ? "+" : "-")
                    .arg(mirror_y > 0 ? "+" : "-"));
              on_import();
            });
  }
#endif

  refresh_source_indicators();
}

static QString autoConvertBlendIfNeeded(const QString &path, QPlainTextEdit *log);

void VehicleImportPage::on_browse_wheels() {
  const QString seed = m_wheels_path_edit->text().trimmed();
  const QString p = QFileDialog::getOpenFileName(
      this, "Select tire-pack mesh",
      seed.isEmpty() ? QStandardPaths::writableLocation(QStandardPaths::HomeLocation) : seed,
      "Mesh files (*.obj *.gltf *.glb *.dae *.blend *.fbx)");
  if (p.isEmpty()) return;
  const QString resolved = autoConvertBlendIfNeeded(p, m_log);
  m_wheels_path_edit->setText(resolved.isEmpty() ? p : resolved);
  recompute_merged_spec();
}

void VehicleImportPage::recompute_merged_spec() {
  if (!m_detected_spec || !m_detected_spec->detected_from_mesh) return;
  const QString wheelsPath = m_wheels_path_edit ? m_wheels_path_edit->text().trimmed() : QString();
  WheelTemplate tpl;
  if (!wheelsPath.isEmpty() && QFileInfo(wheelsPath).suffix().toLower() == "obj") {
    tpl = wheel_template_from_tires_obj(wheelsPath, *m_scale_to_cm);
    if (!tpl.valid) {
      const MeshGeometry wg = load_mesh_geometry_obj(wheelsPath);
      if (wg.valid) {
        const MeshAnalysisResult wr = analyze_mesh(wg, *m_scale_to_cm);
        tpl = wheel_template_from_analysis(wr);
      }
    }
  }
  const MergeResult mr = merge_body_and_wheels(*m_detected_spec, tpl, !wheelsPath.isEmpty());
  if (!mr.ok) {
    m_log->appendPlainText(QString("[%1]  Merge: %2").arg(stamp(), mr.reason));
    if (m_import_btn)   m_import_btn->setEnabled(false);
      return;
  }
  *m_detected_spec = mr.spec;
  const auto applyRowFromWheel = [](WheelSpinRow &row, const WheelSpec &w) {
    row.x->setValue(static_cast<double>(w.x)); row.y->setValue(static_cast<double>(w.y));
    row.z->setValue(static_cast<double>(w.z)); row.r->setValue(static_cast<double>(w.radius));
  };
  applyRowFromWheel(m_row_fl, m_detected_spec->wheels[0]);
  applyRowFromWheel(m_row_fr, m_detected_spec->wheels[1]);
  applyRowFromWheel(m_row_rl, m_detected_spec->wheels[2]);
  applyRowFromWheel(m_row_rr, m_detected_spec->wheels[3]);
  const QString modeName =
      mr.mode == IngestionMode::Combined      ? "combined"      :
      mr.mode == IngestionMode::TireSwap      ? "tire-swap"     :
      mr.mode == IngestionMode::BodyPlusTires ? "body+tires"    : "?";
  m_log->appendPlainText(QString("[%1]  Merge: %2 (%3).").arg(stamp(), modeName, mr.reason));

  const SpecVerification v = verify_chaos_vehicle_spec(*m_detected_spec);
  for (const QString &warn : v.warnings)
    m_log->appendPlainText(QString("[%1]  Chaos-safety warn: %2").arg(stamp(), warn));
  for (const QString &err : v.errors)
    m_log->appendPlainText(QString("[%1]  Chaos-safety ERR : %2").arg(stamp(), err));
  if (m_import_btn) {
    m_import_btn->setEnabled(v.ok);
    if (!v.ok) {
      m_import_btn->setToolTip(
        "Spec has Chaos-safety errors (see log) - fix before importing.");
    } else {
      m_import_btn->setToolTip(QString());
    }
  }
}

void VehicleImportPage::refresh_dep_check_row() {
  if (!m_log) return;

  struct Probe {
    QString name;
    bool    ok;
    QString hint;
  };
  QList<Probe> probes;

  const QString blender = QStandardPaths::findExecutable("blender");
  probes.append({ "blender", !blender.isEmpty(),
                  "needed for .blend/.fbx/.gltf/.glb/.dae source meshes - "
                  "install with: sudo apt install blender" });

  const QString zip = QStandardPaths::findExecutable("zip");
  probes.append({ "zip", !zip.isEmpty(),
                  "needed for Export-as-Kit - install with: sudo apt install zip" });

#ifdef CARLA_STUDIO_WITH_LIBCARLA
  probes.append({ "libcarla", true, QString() });
#else
  probes.append({ "libcarla", false,
                  "Drive cannot reach a running CARLA - rebuild CarlaStudio "
                  "with -DBUILD_CARLA_CLIENT=ON" });
#endif

  QStringList missing;
  bool libcarla_missing = false;
  for (const Probe &p : probes) {
    if (!p.ok) {
      missing << p.name;
      if (p.name == "libcarla") libcarla_missing = true;
    }
  }

  if (m_drop_btn) m_drop_btn->setEnabled(!libcarla_missing);

  if (missing.isEmpty()) {
    m_log->appendPlainText(
      QString("[dep-check]  OK: %1")
        .arg(QStringList({ "blender", "zip", "libcarla" }).join(" · ")));
  } else {
    m_log->appendPlainText(
      QString("[dep-check]  %1 missing dep(s):").arg(missing.size()));
    for (const Probe &p : probes) {
      if (p.ok) continue;
      m_log->appendPlainText(QString("             missing %1 - %2").arg(p.name, p.hint));
    }
  }
}

void VehicleImportPage::refresh_mode_banner() {
  refresh_source_indicators();
  refresh_dep_check_row();
}

void VehicleImportPage::refresh_source_indicators() {
  const auto find_uproject_under = [](const QString &root) -> QString {
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
  QString uproj = resolved_uproject();
  if (uproj.isEmpty() || !QFileInfo(uproj).isFile()) {
    const QString carla_root = m_find_carla_root ? m_find_carla_root() : QString();
    uproj = find_uproject_under(carla_root);
  }
  if (uproj.isEmpty()) {
    uproj = find_uproject_under(QString::fromLocal8Bit(qgetenv("CARLA_ROOT")).trimmed());
  }
  if (uproj.isEmpty()) {
    uproj = find_uproject_under(QString::fromLocal8Bit(qgetenv("CARLA_SRC_ROOT")).trimmed());
  }
  QString eng = m_engine_path_edit ? m_engine_path_edit->text().trimmed() : QString();
  if (eng.isEmpty()) {
    const QString bin = resolved_editor_binary();
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
  if (m_engine_browse_btn) {
    m_engine_browse_btn->setText(engOk ? QStringLiteral("Unreal Editor") : QStringLiteral("Unreal Editor"));
    m_engine_browse_btn->setStyleSheet(engOk ? okStyle : badStyle);
    m_engine_browse_btn->setToolTip(engOk
        ? QStringLiteral("Unreal Engine root: %1\n(click to override)").arg(eng)
        : QStringLiteral("Unreal Engine 5 not detected. Set CARLA_UNREAL_ENGINE_PATH "
                         "or click to choose the engine root."));
  }
  if (m_uproject_browse_btn) {
    m_uproject_browse_btn->setText(uprojOk ? QStringLiteral("Carla built from source") : QStringLiteral("Carla built from source"));
    m_uproject_browse_btn->setStyleSheet(uprojOk ? okStyle : badStyle);
    m_uproject_browse_btn->setToolTip(uprojOk
        ? QStringLiteral("CarlaUnreal.uproject: %1\n(click to override)").arg(uproj)
        : QStringLiteral("CarlaUnreal.uproject not found. Set CARLA_ROOT or "
                         "CARLA_SRC_ROOT, or click to choose the uproject."));
  }
  if (m_import_btn) {
    const bool ready = engOk && uprojOk;
    m_import_btn->setToolTip(ready
        ? QStringLiteral("Headless UE Editor: -unattended -RenderOffScreen -nosplash -nosound. "
                         "Imports mesh, builds wheel + vehicle BPs, saves assets.")
        : QStringLiteral("Set UE5 + CARLA src paths first (buttons on the left)."));
  }
  if (m_drop_btn) {
    m_drop_btn->setToolTip(QStringLiteral(
        "Spawn imported vehicle in CARLA on Traffic Manager autopilot (SAE L5)."));
  }
}

void VehicleImportPage::on_build_kit() {
  if (!m_detected_spec || !m_detected_spec->detected_from_mesh) {
    m_log->appendPlainText("Build Kit: no mesh-driven spec yet - drop a model first.");
    return;
  }
  VehicleSpec spec_copy = *m_detected_spec;
  spec_copy.name = sanitize_vehicle_name(m_vehicle_name_edit->text().trimmed());
  if (spec_copy.name.isEmpty()) {
    m_log->appendPlainText("Build Kit: vehicle name is empty after sanitization.");
    return;
  }
  const QString tmpRoot = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
  const QString target  = tmpRoot + "/carla_studio_kits/" + spec_copy.name + "_kit";
  const KitBundleResult res = write_vehicle_kit(spec_copy, target);
  if (res.ok) {
    m_log->appendPlainText(QString("[%1]  Vehicle kit written: %2 file(s) at %3")
                            .arg(stamp()).arg(res.written_files.size()).arg(res.output_dir));
    if (m_ue_status_label)
      m_ue_status_label->setText(QString("Kit ready: %1").arg(res.output_dir));
    QDesktopServices::openUrl(QUrl::fromLocalFile(res.output_dir));
  } else {
    m_log->appendPlainText(QString("[%1]  Vehicle kit FAILED: %2")
                            .arg(stamp()).arg(res.detail));
    if (m_ue_status_label)
      m_ue_status_label->setText(QString("Kit failed: %1").arg(res.detail));
  }
}

QString VehicleImportPage::resolved_editor_binary() const {
  const QString override_ = m_engine_path_edit ? m_engine_path_edit->text().trimmed() : QString();
  if (!override_.isEmpty()) {
    const QString candidate = override_ + "/Engine/Binaries/Linux/UnrealEditor";
    if (QFileInfo(candidate).isExecutable()) return candidate;
    if (QFileInfo(override_).isExecutable()) return override_;
  }
  return m_find_editor ? m_find_editor() : QString();
}

QString VehicleImportPage::resolved_uproject() const {
  const QString override_ = m_uproject_edit ? m_uproject_edit->text().trimmed() : QString();
  if (!override_.isEmpty() && QFileInfo(override_).isFile()) return override_;
  return m_find_uproject ? m_find_uproject() : QString();
}

void VehicleImportPage::on_browse_engine_path() {
  const QString seed = m_engine_path_edit->text().trimmed();
  const QString dir  = QFileDialog::getExistingDirectory(
      this, "Select Unreal Engine 5 root",
      seed.isEmpty() ? QStandardPaths::writableLocation(QStandardPaths::HomeLocation) : seed);
  if (!dir.isEmpty()) {
    m_engine_path_edit->setText(dir);
    refresh_source_indicators();
  }
}

void VehicleImportPage::on_browse_uproject() {
  const QString seed = m_uproject_edit->text().trimmed();
  const QString dir  = seed.isEmpty()
      ? QStandardPaths::writableLocation(QStandardPaths::HomeLocation)
      : QFileInfo(seed).absolutePath();
  const QString path = QFileDialog::getOpenFileName(
      this, "Select CarlaUnreal.uproject", dir, "Unreal Project (*.uproject);;All files (*)");
  if (!path.isEmpty()) {
    m_uproject_edit->setText(path);
    refresh_source_indicators();
  }
}

void VehicleImportPage::apply_disabled_state_styling() {
  m_import_btn->setStyleSheet(kImportButtonStyle);
  m_drop_btn  ->setStyleSheet(kImportButtonStyle);
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
  if (ext == "blend") args << path;
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

void VehicleImportPage::on_browse() {
  const QString path = QFileDialog::getOpenFileName(
      this, "Select 3D Model",
      QStandardPaths::writableLocation(QStandardPaths::HomeLocation),
      "Supported 3D Models (*.obj *.gltf *.glb *.dae *.blend *.fbx);;"
      "OBJ (*.obj);;glTF (*.gltf *.glb);;Collada (*.dae);;Blender (*.blend);;FBX (*.fbx);;All files (*)");
  if (path.isEmpty()) return;
  const QString resolved = autoConvertBlendIfNeeded(path, m_log);
  load_mesh_from_path(resolved.isEmpty() ? path : resolved);
}

void VehicleImportPage::load_mesh_from_path(const QString &path) {
  if (path.isEmpty() || !QFileInfo(path).isFile()) return;
  m_mesh_path_edit->setText(path);
  m_import_btn->setEnabled(false);
  m_import_btn->setStyleSheet(kImportButtonStyle);
  if (m_drop_btn) {
    m_drop_btn->setStyleSheet(kImportButtonStyle);
    m_drop_btn->setEnabled(false);
  }
  if (m_calibrate_btn) {
    m_calibrate_btn->setStyleSheet(kImportButtonStyle);
    m_calibrate_btn->setEnabled(false);
  }
  if (m_export_btn) {
    m_export_btn->setStyleSheet(kImportButtonStyle);
    m_export_btn->setEnabled(false);
  }
  if (m_asset_label) m_asset_label->setVisible(false);
  if (m_last_asset)  m_last_asset->clear();

  const QString ext = QFileInfo(path).suffix().toLower();
  MeshAABB bb;
  if      (ext == "obj")                    bb = parse_obj(path);
  else if (ext == "gltf" || ext == "glb")   bb = parse_gltf(path);
  else if (ext == "dae")                    bb = parse_dae(path);

  if (!bb.valid) {
    m_log->appendPlainText(QString("[%1]  Could not read geometry from %2").arg(stamp(), path));
    *m_aabb_valid = false;
    return;
  }

  if (ext == "obj") {
    if (bb.face_count == 0) {
      m_log->appendPlainText(QString("[%1]  No usable faces - import disabled").arg(stamp()));
      m_import_btn->setEnabled(false);
      *m_aabb_valid = false;
      return;
    }
    if (bb.malformed_face_lines > 0) {
      m_log->appendPlainText(QString("[%1]  %2 malformed face line(s) will be skipped on import")
                              .arg(stamp()).arg(bb.malformed_face_lines));
    }
  }

  bb.detect_conventions(ext);
  *m_scale_to_cm   = bb.scale_to_cm;
  *m_up_axis      = bb.up_axis;
  *m_forward_axis = bb.forward_axis;
  *m_aabb_valid   = true;

  float xLo, xHi, yLo, yHi, zLo, zHi;
  bb.to_ue(xLo, xHi, yLo, yHi, zLo, zHi);

  m_log->appendPlainText(QString("[%1]  Bounding box: %2 × %3 × %4 cm  (×%5, up=%6)")
      .arg(stamp())
      .arg(xHi - xLo, 0, 'f', 0)
      .arg(yHi - yLo, 0, 'f', 0)
      .arg(zHi - zLo, 0, 'f', 0)
      .arg(bb.scale_to_cm, 0, 'f', 0)
      .arg(QString("XYZ").at(bb.up_axis)));

  *m_detected_spec = VehicleSpec{};
  bool mesh_driven = false;
  if (ext == "obj") {
    const MeshGeometry g = load_mesh_geometry_obj(path);
    if (g.valid) {
      const MeshAnalysisResult ar = analyze_mesh(g, bb.scale_to_cm);
      if (ar.ok && ar.has_four_wheels) {
        *m_detected_spec = build_spec_from_analysis(ar,
                                               m_vehicle_name_edit->text().trimmed(),
                                               path, bb.scale_to_cm,
                                               bb.up_axis, bb.forward_axis);
        const auto applyRowFromWheel = [](WheelSpinRow &row, const WheelSpec &w) {
          row.x->setValue(static_cast<double>(w.x)); row.y->setValue(static_cast<double>(w.y));
          row.z->setValue(static_cast<double>(w.z)); row.r->setValue(static_cast<double>(w.radius));
        };
        applyRowFromWheel(m_row_fl, m_detected_spec->wheels[0]);
        applyRowFromWheel(m_row_fr, m_detected_spec->wheels[1]);
        applyRowFromWheel(m_row_rl, m_detected_spec->wheels[2]);
        applyRowFromWheel(m_row_rr, m_detected_spec->wheels[3]);
        SizeClass effClass = ar.size_class;
        const SizeClass nameHint = classify_by_name(QFileInfo(path).completeBaseName());
        if (nameHint != SizeClass::Unknown) effClass = nameHint;
        const SizePreset preset = preset_for_size_class(effClass);
        m_mass_spin->setValue(static_cast<double>(preset.mass));
        m_steer_spin->setValue(static_cast<double>(preset.max_steer_angle));
        m_brake_spin->setValue(static_cast<double>(preset.max_brake_torque));
        m_susp_raise_spin->setValue(static_cast<double>(preset.susp_max_raise));
        m_susp_drop_spin->setValue(static_cast<double>(preset.susp_max_drop));
        m_susp_damp_spin->setValue(static_cast<double>(preset.susp_damping));
        QStringList parts;
        parts << QString("4 wheels (%1 cm radius)").arg(static_cast<double>(m_detected_spec->wheels[0].radius), 0, 'f', 1);
        parts << QString("size class: %1%2")
                  .arg(size_class_name(effClass),
                       nameHint != SizeClass::Unknown ? " (filename hint)" : "");
        parts << QString("mass %1 kg").arg(static_cast<double>(preset.mass), 0, 'f', 0);
        parts << QString("torque `%1`").arg(preset.torque_curve_tag);
        if (m_detected_spec->small_vehicle_needs_wheel_shape)
          parts << "small-WheelShape applied (Chaos < 18 cm fix)";
        m_log->appendPlainText(QString("[%1]  Detected: %2").arg(stamp(), parts.join("  ·  ")));
        mesh_driven = true;
      }
    }
  }
  if (!mesh_driven) {
    const auto wheels = guessWheels(xLo, xHi, yLo, yHi, zLo, zHi);
    const auto applyRowFromGuess = [](WheelSpinRow &r, const WheelGuess &w) {
      r.x->setValue(static_cast<double>(w.x)); r.y->setValue(static_cast<double>(w.y));
      r.z->setValue(static_cast<double>(w.z)); r.r->setValue(static_cast<double>(w.radius));
    };
    applyRowFromGuess(m_row_fl, wheels[0]);
    applyRowFromGuess(m_row_fr, wheels[1]);
    applyRowFromGuess(m_row_rl, wheels[2]);
    applyRowFromGuess(m_row_rr, wheels[3]);
    m_log->appendPlainText(QString(
        "[%1]  No wheel clusters auto-detected - wheel positions filled from bbox guess "
        "(mesh-driven analysis is .obj-only for now).").arg(stamp()));
  }

  {
    const QString sanitized = sanitize_vehicle_name(QFileInfo(path).completeBaseName());
    if (!sanitized.isEmpty()) m_vehicle_name_edit->setText(sanitized);
  }

  const QString autoPicked = pick_closest_base_vehicle_bp(xHi - xLo);
  const QString lastAuto   = m_base_vehicle_bp_edit->property("lastAutoPick").toString();
  const QString cur        = m_base_vehicle_bp_edit->text().trimmed();
  if (cur.isEmpty() || cur == lastAuto) {
    m_base_vehicle_bp_edit->setText(autoPicked);
    m_base_vehicle_bp_edit->setProperty("lastAutoPick", autoPicked);
  }

  m_import_btn->setEnabled(true);
}

QJsonObject VehicleImportPage::build_import_spec(const QString &mesh_path_to_send) const {
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
  spec["vehicle_name"]    = sanitize_vehicle_name(m_vehicle_name_edit->text().trimmed());
  spec["mesh_path"]       = mesh_path_to_send;
  spec["content_path"]    = m_content_path_edit->text().trimmed();
  spec["base_vehicle_bp"] = m_base_vehicle_bp_edit->text().trimmed();
  spec["mass"]            = static_cast<double>(m_mass_spin->value());
  spec["susp_damping"]    = static_cast<double>(m_susp_damp_spin->value());
  if (*m_aabb_valid) {
    spec["source_scale_to_cm"]  = static_cast<double>(*m_scale_to_cm);
    spec["source_up_axis"]      = static_cast<double>(*m_up_axis);
    spec["source_forward_axis"] = static_cast<double>(*m_forward_axis);
  }
  spec["wheel_fl"] = wheelObj(m_row_fl, static_cast<double>(m_steer_spin->value()), static_cast<double>(m_brake_spin->value()),
                              static_cast<double>(m_susp_raise_spin->value()), static_cast<double>(m_susp_drop_spin->value()));
  spec["wheel_fr"] = wheelObj(m_row_fr, static_cast<double>(m_steer_spin->value()), static_cast<double>(m_brake_spin->value()),
                              static_cast<double>(m_susp_raise_spin->value()), static_cast<double>(m_susp_drop_spin->value()));
  spec["wheel_rl"] = wheelObj(m_row_rl, 0.0,                  static_cast<double>(m_brake_spin->value()),
                              static_cast<double>(m_susp_raise_spin->value()), static_cast<double>(m_susp_drop_spin->value()));
  spec["wheel_rr"] = wheelObj(m_row_rr, 0.0,                  static_cast<double>(m_brake_spin->value()),
                              static_cast<double>(m_susp_raise_spin->value()), static_cast<double>(m_susp_drop_spin->value()));
  if (m_detected_spec && m_detected_spec->detected_from_mesh) {
    QJsonObject chassis;
    chassis["x_min"] = static_cast<double>(m_detected_spec->chassis_x_min);
    chassis["x_max"] = static_cast<double>(m_detected_spec->chassis_x_max);
    chassis["y_min"] = static_cast<double>(m_detected_spec->chassis_y_min);
    chassis["y_max"] = static_cast<double>(m_detected_spec->chassis_y_max);
    chassis["z_min"] = static_cast<double>(m_detected_spec->chassis_z_min);
    chassis["z_max"] = static_cast<double>(m_detected_spec->chassis_z_max);
    spec["chassis_aabb_cm"] = chassis;
  }
  return spec;
}

void VehicleImportPage::on_import() {
  const QString mesh_path = m_mesh_path_edit->text().trimmed();
  const QString name     = sanitize_vehicle_name(m_vehicle_name_edit->text().trimmed());
  if (mesh_path.isEmpty() || name.isEmpty()) {
    m_log->appendPlainText("Error: vehicle name and mesh path are required.");
    return;
  }
  if (m_vehicle_name_edit->text().trimmed() != name)
    m_vehicle_name_edit->setText(name);

  QString meshToSend = mesh_path;
  const QString tires_path = m_wheels_path_edit ? m_wheels_path_edit->text().trimmed() : QString();
  if (!tires_path.isEmpty()
      && QFileInfo(mesh_path).suffix().toLower() == "obj"
      && QFileInfo(tires_path).suffix().toLower() == "obj"
      && m_detected_spec) {
    const QString out_dir = QStandardPaths::writableLocation(QStandardPaths::TempLocation)
                         + "/carla_studio_merge";
    const ObjMergeResult mr = merge_body_and_tires(mesh_path, tires_path, out_dir, m_detected_spec->wheels);
    if (mr.ok) {
      meshToSend = mr.output_path;
      m_log->appendPlainText(QString("[%1]  Merged body+tires → %2 (%3)")
                              .arg(stamp(), meshToSend, mr.reason));
    } else {
      m_log->appendPlainText(QString("[%1]  Body+tires merge failed: %2 - sending body-only OBJ.")
                              .arg(stamp(), mr.reason));
    }
  }
  if (QFileInfo(meshToSend).suffix().toLower() == "obj") {
    const SanitizeReport rep = sanitize_obj(meshToSend, 0.0f);
    if (rep.ok) {
      meshToSend = rep.output_path;
      m_log->appendPlainText(QString("[%1]  Sanitized OBJ → %2  (auto: ×%3 scale, swap_xy=%4, z_lift=%5cm, skipped %6 malformed face line(s))")
                              .arg(stamp()).arg(meshToSend)
                              .arg(rep.applied_scale, 0, 'f', 2)
                              .arg(rep.swap_xy ? "true" : "false")
                              .arg(rep.z_lift_cm, 0, 'f', 1)
                              .arg(rep.skipped_face_lines));
    }
  }

  const QJsonObject spec = build_import_spec(meshToSend);

  m_import_btn->setEnabled(false);
  m_progress->setVisible(true);
  m_progress->setRange(0, 100);
  m_progress->setValue(0);
  m_progress->setFormat("starting…");
  m_asset_label->clear();
  m_asset_label->setVisible(false);

  QPointer<QPushButton>     btn_ptr      = m_import_btn;
  QPointer<QPushButton>     dropPtr     = m_drop_btn;
  QPointer<QLabel>          statusPtr   = m_ue_status_label;
  QPointer<QPlainTextEdit>  log_ptr      = m_log;
  QPointer<QProgressBar>    progressPtr = m_progress;
  QPointer<QLabel>          assetPtr    = m_asset_label;
  std::shared_ptr<QString>  lastAsset   = m_last_asset;
  const QString    capturedName       = name;
  const QString    capturedSourceMesh = mesh_path;
  QVector3D capturedFL, capturedFR, capturedRL, capturedRR;
  if (m_detected_spec) {
    capturedFL = QVector3D(m_detected_spec->wheels[0].x, m_detected_spec->wheels[0].y, m_detected_spec->wheels[0].z);
    capturedFR = QVector3D(m_detected_spec->wheels[1].x, m_detected_spec->wheels[1].y, m_detected_spec->wheels[1].z);
    capturedRL = QVector3D(m_detected_spec->wheels[2].x, m_detected_spec->wheels[2].y, m_detected_spec->wheels[2].z);
    capturedRR = QVector3D(m_detected_spec->wheels[3].x, m_detected_spec->wheels[3].y, m_detected_spec->wheels[3].z);
  }

  auto sendNow = [=]() {
    if (!log_ptr) return;
    log_ptr->appendPlainText(QString("[%1]  Sending spec to UE Editor…").arg(stamp()));
    if (statusPtr) statusPtr->setText("UE Editor: sending …");
    if (progressPtr) {
      progressPtr->setRange(0, 0);
      progressPtr->setFormat("importing in UE Editor…");
    }
    Q_UNUSED(QtConcurrent::run([=]() {
      const QString response = send_json(spec);
      QMetaObject::invokeMethod(qApp, [=]() {
        if (!log_ptr) return;
        if (btn_ptr) btn_ptr->setEnabled(true);
        if (progressPtr) progressPtr->setRange(0, 100);
        if (response.isEmpty()) {
          if (statusPtr) {
            statusPtr->setText("UE Editor: connection failed");
            statusPtr->setStyleSheet("color: red;");
          }
          if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("connection failed"); }
          log_ptr->appendPlainText(QString("[%1]  Error: no response from UE Editor on port %2.")
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
          if (m_calibrate_btn) m_calibrate_btn->setEnabled(!path.isEmpty());
          if (m_export_btn) m_export_btn->setEnabled(!path.isEmpty());
          if (btn_ptr) {
            btn_ptr->setStyleSheet(
              "QPushButton { padding: 4px 14px; font-weight: 600; min-width: 80px; "
              "background-color: #C8E6C9; color: #1B5E20; "
              "border: 1px solid #81C784; border-radius: 3px; }"
              "QPushButton:hover { background-color: #A5D6A7; }");
          }
          log_ptr->appendPlainText(QString("[%1]  Success! Blueprint created at: %2").arg(stamp(), path));
#ifdef CARLA_STUDIO_WITH_QT3D
          {
            const QString canonical = QString("/tmp/vi_%1_body_canonical.obj").arg(capturedName);
            const QString show_path  = QFileInfo(canonical).exists() ? canonical : capturedSourceMesh;
            VehiclePreviewWindow::instance()->show_for(
                show_path, capturedFL, capturedFR, capturedRL, capturedRR);
            log_ptr->appendPlainText(QString("[%1]  Preview opened (%2).").arg(stamp(), show_path));
          }
#endif
        } else {
          if (statusPtr) {
            statusPtr->setText("UE Editor: import failed");
            statusPtr->setStyleSheet("color: red;");
          }
          if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("failed"); }
          if (btn_ptr) btn_ptr->setStyleSheet(kImportButtonStyle);
          log_ptr->appendPlainText(QString("[%1]  Failed: %2").arg(stamp(), err));
        }
      }, Qt::QueuedConnection);
    }));
  };

  const QString uprojectEarly = resolved_uproject();

  if (probe_importer_port()) {
    const EditorProcessInfo info = find_editor_for_uproject(uprojectEarly);
    if (info.exists && info.is_headless && editor_is_stale(info, uprojectEarly)) {
      if (log_ptr) log_ptr->appendPlainText(QString(
          "[%1]  Plugin libUnrealEditor-CarlaTools.so is newer than running editor "
          "(pid %2) - restarting before send.").arg(stamp()).arg(info.pid));
      if (progressPtr) { progressPtr->setRange(0, 0); progressPtr->setFormat("restarting stale editor…"); }
      kill_editor(info.pid);
    } else {
      if (progressPtr) { progressPtr->setValue(50); progressPtr->setFormat("editor already up - sending"); }
      sendNow();
      return;
    }
  } else {
    const EditorProcessInfo orphan = find_editor_for_uproject(uprojectEarly);
    if (orphan.exists && orphan.is_headless) {
      if (log_ptr) log_ptr->appendPlainText(QString(
          "[%1]  Found orphan headless editor pid %2 not bound to port %3 - killing before relaunch.")
          .arg(stamp()).arg(orphan.pid).arg(kImporterPort));
      kill_editor(orphan.pid);
    } else if (orphan.exists && !orphan.is_headless) {
      if (log_ptr) log_ptr->appendPlainText(QString(
          "[%1]  An interactive UnrealEditor (pid %2) is already running this project but the "
          "import server is not bound on port %3. Close that editor (or use it with the import "
          "plugin loaded) before retrying - refusing to spawn a duplicate.")
          .arg(stamp()).arg(orphan.pid).arg(kImporterPort));
      if (statusPtr) {
        statusPtr->setText("UE Editor: duplicate refused");
        statusPtr->setStyleSheet("color: red;");
      }
      if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("editor already running"); }
      if (btn_ptr) btn_ptr->setEnabled(true);
      return;
    }
  }

  const QString editor_early = resolved_editor_binary();
  QString enginePathEarly = m_engine_path_edit ? m_engine_path_edit->text().trimmed() : QString();
  if (enginePathEarly.isEmpty() && !editor_early.isEmpty()) {
    QDir d(QFileInfo(editor_early).absolutePath());
    for (int i = 0; i < 3 && !d.isRoot(); ++i) d.cdUp();
    enginePathEarly = d.absolutePath();
  }
  if (enginePathEarly.isEmpty()) {
    enginePathEarly = QString::fromLocal8Bit(qgetenv("CARLA_UNREAL_ENGINE_PATH")).trimmed();
  }

  if (uprojectEarly.isEmpty()) {
    if (log_ptr) log_ptr->appendPlainText(QString(
      "[%1]  No .uproject resolved - set CARLA_SRC_ROOT (with `export`!) or pick the .uproject "
      "via the … button next to Import.").arg(stamp()));
    if (statusPtr) {
      statusPtr->setText("UE Editor: no .uproject");
      statusPtr->setStyleSheet("color: red;");
    }
    if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("uproject not set"); }
    if (btn_ptr) btn_ptr->setEnabled(true);
    return;
  }
  if (enginePathEarly.isEmpty()) {
    if (log_ptr) log_ptr->appendPlainText(QString(
      "[%1]  No UE Engine root resolved - set CARLA_UNREAL_ENGINE_PATH (with `export`!) or pick "
      "the engine root via the … button next to Import.").arg(stamp()));
    if (statusPtr) {
      statusPtr->setText("UE Editor: no engine path");
      statusPtr->setStyleSheet("color: red;");
    }
    if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("engine path not set"); }
    if (btn_ptr) btn_ptr->setEnabled(true);
    return;
  }
  if (editor_early.isEmpty()) {
    if (log_ptr) log_ptr->appendPlainText(QString(
      "[%1]  UnrealEditor binary not found at %2/Engine/Binaries/Linux/UnrealEditor - verify the engine path.")
      .arg(stamp(), enginePathEarly));
    if (statusPtr) {
      statusPtr->setText("UE Editor: binary missing");
      statusPtr->setStyleSheet("color: red;");
    }
    if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("editor binary missing"); }
    if (btn_ptr) btn_ptr->setEnabled(true);
    return;
  }

  const QString editor   = editor_early;
  const QString uproject = uprojectEarly;
  const QString editor_log = QStandardPaths::writableLocation(QStandardPaths::TempLocation)
                          + "/carla_studio_ue_editor.log";
  auto shEsc = [](const QString &s) {
    QString out = s; out.replace('\'', "'\\''"); return "'" + out + "'";
  };
  const QString shellCmd = QString("exec %1 %2 -unattended -RenderOffScreen -nosplash -nosound >>%3 2>&1")
      .arg(shEsc(editor), shEsc(uproject), shEsc(editor_log));
  QStringList args;
  args << "-c" << shellCmd;
  qint64 pid = 0;
  if (!QProcess::startDetached("/bin/sh", args, QString(), &pid)) {
    if (log_ptr) log_ptr->appendPlainText(QString("[%1]  Could not start UE Editor: %2").arg(stamp(), editor));
    if (statusPtr) {
      statusPtr->setText("UE Editor: launch failed");
      statusPtr->setStyleSheet("color: red;");
    }
    if (progressPtr) { progressPtr->setValue(0); progressPtr->setFormat("launch failed"); }
    if (btn_ptr) btn_ptr->setEnabled(true);
    return;
  }
  if (log_ptr) log_ptr->appendPlainText(QString("[%1]  Launched headless UE Editor (pid %2, stdout→%3) - waiting for VehicleImporter port %4.")
                                        .arg(stamp()).arg(pid).arg(editor_log).arg(kImporterPort));
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
    if (probe_importer_port()) {
      poll->stop(); poll->deleteLater();
      if (log_ptr) log_ptr->appendPlainText(QString("[%1]  Editor ready after %2 s (port %3 open).")
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
      if (log_ptr) log_ptr->appendPlainText(QString("[%1]  UE Editor process %2 exited after %3 s before opening port %4 - check the editor log under <project>/Saved/Logs/ for the reason.")
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
      if (btn_ptr) btn_ptr->setEnabled(true);
      return;
    }
    if (*attempts == 30 || *attempts == 120 || *attempts == 300 ||
        (*attempts > 300 && *attempts % 300 == 0)) {
      if (log_ptr) log_ptr->appendPlainText(QString("[%1]  Still waiting - UE Editor pid %2 alive, port %3 not yet open (%4 s elapsed).")
                                            .arg(stamp()).arg(launchedPid).arg(kImporterPort).arg(*attempts));
    }
  });
  poll->start();
}

void VehicleImportPage::on_drop() {
  const QString asset_path = *m_last_asset;
  if (asset_path.isEmpty()) {
    m_log->appendPlainText(QString("[%1]  Drive in CARLA: no imported asset yet - run Import first.").arg(stamp()));
    return;
  }
  const QString uproject = resolved_uproject();
  if (uproject.isEmpty()) {
    m_log->appendPlainText(QString("[%1]  Drive in CARLA: uproject path unresolved - set CARLA_SRC_ROOT or pick the .uproject above.").arg(stamp()));
    return;
  }
  const QString name = sanitize_vehicle_name(m_vehicle_name_edit->text().trimmed());
  if (name.isEmpty()) return;
  if (m_vehicle_name_edit->text().trimmed() != name)
    m_vehicle_name_edit->setText(name);

  VehicleRegistration reg;
  reg.uproject_path      = uproject;
  reg.shipping_carla_root = m_find_carla_root ? m_find_carla_root() : QString();
  reg.bp_asset_path       = asset_path;
  reg.make              = "Custom";
  reg.model             = name;
  reg.editor_binary      = resolved_editor_binary();

  m_drop_btn->setEnabled(false);
  m_log->appendPlainText(QString("[%1]  Drive in CARLA: registering %2 in VehicleParameters.json …")
                          .arg(stamp(), asset_path));

  QPointer<QPushButton>    btn_ptr    = m_drop_btn;
  QPointer<QLabel>         statusPtr = m_ue_status_label;
  QPointer<QPlainTextEdit> log_ptr    = m_log;
  const QString            make      = reg.make;
  const QString            model     = reg.model;

  QPointer<QProgressBar> progPtr = m_progress;
  Q_UNUSED(QtConcurrent::run([=]() {
    RegisterResult rr;
    DeployResult   dr;
    SpawnResult    sr;
    QString        unhandled;
    try {
      rr = register_vehicle_in_json(reg);
      if (rr.ok) {
        const EditorProcessInfo importEd = find_editor_for_uproject(uproject);
        if (importEd.exists && importEd.is_headless) {
          QMetaObject::invokeMethod(qApp, [log_ptr, pid = importEd.pid]() {
            if (log_ptr) log_ptr->appendPlainText(QString(
                "[%1]  Killing import editor (pid %2) before cook to free RAM/GPU + clear locks.")
                  .arg(stamp()).arg(pid));
          }, Qt::QueuedConnection);
          kill_editor(importEd.pid);
        }
        QMetaObject::invokeMethod(qApp, [log_ptr, progPtr]() {
          if (log_ptr) log_ptr->appendPlainText(QString(
            "[%1]  Cooking assets for shipping CARLA - first cook is 2-3 min "
            "(shader compile), subsequent cooks ~30 s. UE editor commandlet running…")
            .arg(stamp()));
          if (auto *pb = progPtr.data()) {
            pb->setVisible(true);
            pb->setRange(0, 0);
            pb->setFormat("cooking assets for shipping CARLA…");
          }
        }, Qt::QueuedConnection);
        dr = deploy_vehicle_to_shipping_carla(reg);
        QMetaObject::invokeMethod(qApp, [progPtr, ok = dr.ok]() {
          if (auto *pb = progPtr.data()) {
            pb->setRange(0, 100);
            pb->setValue(ok ? 100 : 0);
            pb->setFormat(ok ? "cook + deploy done" : "cook failed");
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
        sr = spawn_in_running_carla(make, model);
      }
    } catch (const std::exception &e) {
      unhandled = QString("std::exception: %1").arg(QString::fromUtf8(e.what()));
    } catch (...) {
      unhandled = "unknown C++ exception";
    }

    QMetaObject::invokeMethod(qApp, [=]() {
      if (!unhandled.isEmpty()) {
        if (log_ptr) log_ptr->appendPlainText(QString(
          "[%1]  Drive: caught %2 - likely API mismatch between this Studio's "
          "libcarla-client and the running CARLA. Drive aborted; Import side is unaffected.")
          .arg(stamp(), unhandled));
        if (statusPtr) {
          statusPtr->setText("Drive: aborted (LibCarla error)");
          statusPtr->setStyleSheet("color: red;");
        }
        if (btn_ptr) btn_ptr->setEnabled(true);
        return;
      }
      if (!log_ptr) return;
      if (btn_ptr) btn_ptr->setEnabled(true);
      log_ptr->appendPlainText(QString("[%1]  Registration (dev tree): %2  (%3)")
                                .arg(stamp()).arg(rr.ok ? "OK" : "FAIL").arg(rr.detail));
      log_ptr->appendPlainText(QString("[%1]  Deploy to shipping CARLA: %2  (%3)")
                                .arg(stamp()).arg(dr.ok ? "OK" : "SKIP/FAIL").arg(dr.detail));
      if (dr.ok && !dr.dest_dir.isEmpty()) {
        const QString archiveRoot = QStandardPaths::writableLocation(QStandardPaths::HomeLocation)
                                  + "/.carla_studio/archive/" + model;
        QDir().mkpath(archiveRoot);
        QDir srcD(dr.dest_dir);
        const QStringList files = srcD.entryList({"*.uasset", "*.uexp"}, QDir::Files);
        int archived = 0;
        for (const QString &f : files) {
          const QString s = dr.dest_dir + "/" + f;
          const QString t = archiveRoot + "/" + f;
          if (QFile::exists(t)) QFile::remove(t);
          if (QFile::copy(s, t)) ++archived;
        }
        log_ptr->appendPlainText(QString("[%1]  Installed at: %2").arg(stamp(), dr.dest_dir));
        log_ptr->appendPlainText(QString("[%1]  Archived %2 file(s) → %3").arg(stamp())
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
          if (m_drop_btn) {
            m_drop_btn->setStyleSheet(
              "QPushButton { padding: 4px 14px; font-weight: 600; min-width: 80px; "
              "background-color: #FFF59D; color: #6A4F00; "
              "border: 1px solid #FFD54F; border-radius: 3px; }"
              "QPushButton:hover { background-color: #FFE082; }");
          }
          log_ptr->appendPlainText(QString("[%1]  Spawn: OK - %2").arg(stamp(), sr.detail));
          break;
        case SpawnResult::Kind::BlueprintNotFound:
          if (statusPtr) {
            statusPtr->setText("CARLA: restart needed");
            statusPtr->setStyleSheet("color: #BB8800;");
          }
          log_ptr->appendPlainText(QString("[%1]  Spawn: %2").arg(stamp(), sr.detail));
          break;
        case SpawnResult::Kind::NoSimulator:
          if (statusPtr) {
            statusPtr->setText("CARLA: starting simulator …");
            statusPtr->setStyleSheet("color: #BB8800;");
          }
          log_ptr->appendPlainText(QString("[%1]  Spawn: %2").arg(stamp(), sr.detail));
          if (dr.ok && m_request_start_carla) {
            log_ptr->appendPlainText(QString(
              "[%1]  Auto-pressing Studio's START button - will retry spawn once the simulator "
              "is up on port 2000.").arg(stamp()));
            m_request_start_carla();
            auto attempts = std::make_shared<int>(0);
            auto *poll = new QTimer(this);
            poll->setInterval(2000);
            QPointer<QPlainTextEdit> logL = log_ptr;
            QPointer<QLabel>         stL  = statusPtr;
            QPointer<QPushButton>    btnL = btn_ptr;
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
                  try { sr2 = spawn_in_running_carla(mk, md); }
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
              if (*attempts >= 300) {
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
            log_ptr->appendPlainText(QString(
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
              log_ptr->appendPlainText(QString(
                "[%1]  Vehicle deployed. Could not auto-spawn - start CARLA and re-click Drive.")
                .arg(stamp()));
            }
          } else {
            if (statusPtr) {
              statusPtr->setText("Drive: spawn failed");
              statusPtr->setStyleSheet("color: red;");
            }
            log_ptr->appendPlainText(QString("[%1]  Spawn: FAIL - %2").arg(stamp(), sr.detail));
          }
          break;
      }
    }, Qt::QueuedConnection);
  }));
}

void VehicleImportPage::on_export() {
  if (!m_last_asset || m_last_asset->isEmpty()) {
    if (m_log) m_log->appendPlainText(QString("[%1]  Export: no imported asset yet - run Import first.").arg(stamp()));
    return;
  }
  const QString name = sanitize_vehicle_name(m_vehicle_name_edit->text().trimmed());
  if (name.isEmpty()) return;

  const QString carla_root = m_find_carla_root ? m_find_carla_root() : QString();
  const QString uproject  = resolved_uproject();

  VehicleRegistration reg;
  reg.uproject_path      = uproject;
  reg.shipping_carla_root = carla_root;
  reg.bp_asset_path       = *m_last_asset;
  reg.make              = "Custom";
  reg.model             = name;
  reg.editor_binary      = resolved_editor_binary();

  if (m_export_btn) m_export_btn->setEnabled(false);
  if (m_log) m_log->appendPlainText(QString("[%1]  Export: cooking + bundling %2 …").arg(stamp(), name));

  QPointer<QPushButton>    btn_ptr  = m_export_btn;
  QPointer<QPlainTextEdit> log_ptr  = m_log;
  const QString            engineRoot = qEnvironmentVariable("CARLA_UNREAL_ENGINE_PATH");

  Q_UNUSED(QtConcurrent::run([=]() {
    DeployResult dr;
    QString unhandled;
    try {
      const EditorProcessInfo importEd = find_editor_for_uproject(uproject);
      if (importEd.exists && importEd.is_headless) {
        QMetaObject::invokeMethod(qApp, [log_ptr, pid = importEd.pid]() {
          if (log_ptr) log_ptr->appendPlainText(QString(
              "[%1]  Killing import editor (pid %2) before cook.")
                .arg(stamp()).arg(pid));
        }, Qt::QueuedConnection);
        kill_editor(importEd.pid);
      }
      dr = deploy_vehicle_to_shipping_carla(reg);
    } catch (const std::exception &e) {
      unhandled = QString("std::exception: %1").arg(QString::fromUtf8(e.what()));
    } catch (...) {
      unhandled = "unknown C++ exception";
    }

    QString zipPath, zipDetail;
    bool zipOk = false;
    if (dr.ok && !dr.dest_dir.isEmpty()) {
      int ueMajor = 5;
      if (engineRoot.contains("Unreal4") || engineRoot.contains("UE4")) ueMajor = 4;
      const QString date = QDateTime::currentDateTime().toString("MMddyyyy");
      const QString out_dir = QStandardPaths::writableLocation(QStandardPaths::HomeLocation)
                           + "/.carla_studio/exports";
      QDir().mkpath(out_dir);
      zipPath = QString("%1/carla_%2_ue%3_%4.zip").arg(out_dir, name).arg(ueMajor).arg(date);
      QFile::remove(zipPath);
      QProcess proc;
      proc.setWorkingDirectory(QFileInfo(dr.dest_dir).absolutePath());
      const QString folderName = QFileInfo(dr.dest_dir).fileName();
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
      if (btn_ptr) btn_ptr->setEnabled(true);
      if (!log_ptr) return;
      if (!unhandled.isEmpty()) {
        log_ptr->appendPlainText(QString("[%1]  Export: deploy aborted - %2").arg(stamp(), unhandled));
        return;
      }
      log_ptr->appendPlainText(QString("[%1]  Export: deploy %2  (%3)")
                                .arg(stamp()).arg(dr.ok ? "OK" : "FAIL").arg(dr.detail));
      if (zipOk) {
        log_ptr->appendPlainText(QString("[%1]  Export: archive → %2  (%3)").arg(stamp(), zipPath, zipDetail));
        if (m_export_btn) m_export_btn->setStyleSheet(kSuccessButtonStyle);
      } else if (dr.ok) {
        log_ptr->appendPlainText(QString("[%1]  Export: zip failed - %2").arg(stamp(), zipDetail));
      }
    }, Qt::QueuedConnection);
  }));
}

}
