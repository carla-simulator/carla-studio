// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QLabel>
#include <QPixmap>
#include <QVector3D>
#include <array>

namespace Qt3DRender { class QCamera; }

namespace carla_studio::vehicle_import {

class ViewGizmo : public QLabel {
  Q_OBJECT
 public:
  explicit ViewGizmo(QWidget *parent = nullptr);
  QSize sizeHint() const override { return {110, 110}; }
  void  bind_camera(Qt3DRender::QCamera *cam);

 signals:
  void snap_requested(const QVector3D &pos, const QVector3D &center,
                     const QVector3D &up);

 protected:
  void mousePressEvent(QMouseEvent *) override;

 private:
  struct Handle {
    QVector3D world;
    QColor    color;
    QString   label;
    float     depth    = 0.f;
    QPoint    screen_pt = {};
  };
  std::array<Handle, 6> m_handles;
  QVector3D m_cam_pos{800, -800, 500};
  QVector3D m_cam_center{0, 0, 80};
  QVector3D m_cam_up{0, 0, 1};

  void reproject();
  void redraw();
};

}
