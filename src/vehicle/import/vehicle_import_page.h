// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include "vehicle/import/import_mode.h"
#include "vehicle/import/vehicle_spec.h"

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

  explicit VehicleImportPage(EditorBinaryResolver find_editor,
                             UprojectResolver     find_uproject,
                             CarlaRootResolver    find_carla_root,
                             StartCarlaRequester  request_start_carla,
                             QWidget *parent = nullptr);

  void load_mesh_from_path(const QString &path);

 private:
  void on_browse();
  void on_import();
  void on_drop();
  void on_export();
  void on_build_kit();
  void on_browse_engine_path();
  void on_browse_uproject();
  void on_browse_wheels();
  void refresh_mode_banner();
  void refresh_source_indicators();
  void refresh_dep_check_row();
  void recompute_merged_spec();
  void apply_disabled_state_styling();
  QString resolved_editor_binary() const;
  QString resolved_uproject() const;
  QJsonObject build_import_spec(const QString &mesh_path_to_send) const;

  EditorBinaryResolver m_find_editor;
  UprojectResolver     m_find_uproject;
  CarlaRootResolver    m_find_carla_root;
  StartCarlaRequester  m_request_start_carla;

  QLineEdit       *m_engine_path_edit     = nullptr;
  QPushButton     *m_engine_browse_btn    = nullptr;
  QLineEdit       *m_uproject_edit       = nullptr;
  QPushButton     *m_uproject_browse_btn  = nullptr;
  QLineEdit       *m_mesh_path_edit       = nullptr;
  QPushButton     *m_browse_btn          = nullptr;
  QLineEdit       *m_wheels_path_edit     = nullptr;
  QPushButton     *m_wheels_browse_btn    = nullptr;
  QLabel          *m_wheels_hint_label    = nullptr;
  QLabel          *m_aabb_label          = nullptr;
  QLabel          *m_validation_label    = nullptr;
  WheelSpinRow     m_row_fl, m_row_fr, m_row_rl, m_row_rr;
  QLineEdit       *m_vehicle_name_edit    = nullptr;
  QLineEdit       *m_content_path_edit    = nullptr;
  QLineEdit       *m_base_vehicle_bp_edit  = nullptr;
  QDoubleSpinBox  *m_mass_spin           = nullptr;
  QDoubleSpinBox  *m_steer_spin          = nullptr;
  QDoubleSpinBox  *m_susp_raise_spin      = nullptr;
  QDoubleSpinBox  *m_susp_drop_spin       = nullptr;
  QDoubleSpinBox  *m_susp_damp_spin       = nullptr;
  QDoubleSpinBox  *m_brake_spin          = nullptr;
  QLabel          *m_ue_status_label      = nullptr;
  QPushButton     *m_import_btn          = nullptr;
  QPushButton     *m_calibrate_btn       = nullptr;
  QPushButton     *m_drop_btn            = nullptr;
  QPushButton     *m_export_btn          = nullptr;
  QPushButton     *m_build_kit_btn        = nullptr;
  QLabel          *m_mode_banner         = nullptr;
  QProgressBar    *m_progress           = nullptr;
  QLabel          *m_asset_label         = nullptr;
  QPlainTextEdit  *m_log                = nullptr;

  std::shared_ptr<float>   m_scale_to_cm   = std::make_shared<float>(1.0f);
  std::shared_ptr<int>     m_up_axis      = std::make_shared<int>(2);
  std::shared_ptr<int>     m_forward_axis = std::make_shared<int>(0);
  std::shared_ptr<bool>    m_aabb_valid   = std::make_shared<bool>(false);
  std::shared_ptr<QString> m_last_asset   = std::make_shared<QString>();
  std::shared_ptr<VehicleSpec> m_detected_spec = std::make_shared<VehicleSpec>();
  std::shared_ptr<ImportMode>  m_active_mode   = std::make_shared<ImportMode>(ImportMode::Lite);
  QLabel                  *m_detection_label  = nullptr;
};

}
