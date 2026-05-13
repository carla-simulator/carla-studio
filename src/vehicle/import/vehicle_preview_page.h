// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
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
class QStackedWidget;
class QToolButton;

namespace Qt3DCore   { class QEntity; class QTransform; }
namespace Qt3DRender { class QSceneLoader; class QPointLight; class QDirectionalLight; }
namespace Qt3DExtras { class Qt3DWindow; class QOrbitCameraController;
                       class QPhongMaterial; class QSphereMesh; class QPlaneMesh;
                       class QCylinderMesh; }

namespace carla_studio::vehicle_import {

class ViewGizmo;

class VehiclePreviewPage : public QWidget {
  Q_OBJECT
 public:
  enum class ToolMode { Import, SensorAssembly };

  explicit VehiclePreviewPage(QWidget *parent = nullptr);

 public slots:
  void load_mesh(const QString &path);
  void set_wheel_positions_cm(const QVector3D &fl, const QVector3D &fr,
                           const QVector3D &rl, const QVector3D &rr);
  void resetCamera();
  void set_tool_mode(ToolMode mode);

  void view_top();
  void view_side();
  void view_front();
  void view_3d();
  void fit_camera();
  void rotate_minus90();
  void rotate_plus90();
  void rotate180();
  void mirror_x();
  void mirror_y();
  void recenter_mesh();
  void reset_mesh_transform();
  void apply_to_spec();
  void capture_window();

 protected:
  bool eventFilter(QObject *obj, QEvent *ev) override;
  void resizeEvent(QResizeEvent *ev) override;

 signals:
  void apply_requested(float yaw_deg, float mirror_x, float mirror_y);

 private slots:
  void on_browse();
  void on_load_clicked();
  void on_scene_status_changed(int status);

 private:
  void build_scene();
  void build_grid_floor(Qt3DCore::QEntity *root, float half_extent_cm, float cell_cm);
  void build_axis_gizmo(Qt3DCore::QEntity *root, float length_cm);
  void build_reference_outlines(Qt3DCore::QEntity *root);
  void build_wheel_markers(Qt3DCore::QEntity *root);
  void build_alignment_rods(Qt3DCore::QEntity *root);
  void update_alignment_lines();
  void update_mesh_bounds();
  void apply_adjustment(const QString &label);

  float m_adjust_yaw_deg = 0.f;
  float m_adjust_mirror_x = 1.f;
  float m_adjust_mirror_y = 1.f;

  Qt3DExtras::Qt3DWindow            *m_view          = nullptr;
  Qt3DCore::QEntity                 *m_root          = nullptr;
  Qt3DCore::QEntity                 *m_mesh_entity    = nullptr;
  Qt3DCore::QTransform              *m_mesh_transform = nullptr;
  Qt3DRender::QSceneLoader          *m_scene_loader   = nullptr;
  QString                            m_pending_mesh_path;
  Qt3DExtras::QOrbitCameraController *m_cam_ctl       = nullptr;

  std::array<Qt3DCore::QEntity*, 4>  m_wheel_markers{};
  std::array<Qt3DCore::QTransform*, 4> m_wheel_xforms{};
  bool                                m_wheel_positions_external = false;

  struct AlignRod {
    Qt3DCore::QEntity          *entity = nullptr;
    Qt3DExtras::QCylinderMesh  *mesh   = nullptr;
    Qt3DCore::QTransform       *xform  = nullptr;
  };

  std::array<AlignRod, 10> m_align_rods{};
  QVector3D                m_last_wheel_pos[4]{};

  ToolMode        m_tool_mode       = ToolMode::Import;
  ViewGizmo      *m_view_gizmo      = nullptr;
  QStackedWidget *m_left_stack      = nullptr;
  QStackedWidget *m_right_stack     = nullptr;
  QPushButton    *m_import_mode_btn  = nullptr;
  QPushButton    *m_sensor_mode_btn  = nullptr;
  QPushButton    *m_apply_btn       = nullptr;

  QWidget        *m_container      = nullptr;
  QLineEdit      *m_path_edit       = nullptr;
  QPushButton    *m_browse_btn      = nullptr;
  QPushButton    *m_load_btn        = nullptr;
  QPushButton    *m_reset_cam_btn    = nullptr;
  QLabel         *m_status_label    = nullptr;
  QLabel         *m_calibration_badge = nullptr;
  QLabel         *m_legend         = nullptr;
  QPlainTextEdit *m_info_box        = nullptr;
};

}
