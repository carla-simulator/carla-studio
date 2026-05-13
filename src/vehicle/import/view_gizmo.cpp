// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "view_gizmo.h"

#include <QMouseEvent>
#include <QPainter>
#include <QRadialGradient>
#include <Qt3DRender/QCamera>

#include <algorithm>
#include <array>
#include <cmath>

namespace carla_studio::vehicle_import {

ViewGizmo::ViewGizmo(QWidget *parent) : QLabel(parent)
{
  setFixedSize(110, 110);
  setAlignment(Qt::AlignCenter);
  setStyleSheet("background: transparent; border: none;");

  m_handles = {{
    { {1,0,0},  QColor(220, 60, 60),   "+X" },
    { {-1,0,0}, QColor(110, 25, 25),   "-X" },
    { {0,1,0},  QColor(60, 200, 60),   "+Y" },
    { {0,-1,0}, QColor(20, 90, 20),    "-Y" },
    { {0,0,1},  QColor(80, 130, 255),  "+Z" },
    { {0,0,-1}, QColor(30, 55, 140),   "-Z" },
  }};
  reproject();
  redraw();
}

void ViewGizmo::bind_camera(Qt3DRender::QCamera *cam)
{
  if (!cam) return;
  auto sync = [this, cam]() {
    m_cam_pos    = cam->position();
    m_cam_center = cam->viewCenter();
    m_cam_up     = cam->upVector();
    reproject();
    redraw();
  };
  connect(cam, &Qt3DRender::QCamera::positionChanged,   this, sync);
  connect(cam, &Qt3DRender::QCamera::viewCenterChanged, this, sync);
  connect(cam, &Qt3DRender::QCamera::upVectorChanged,   this, sync);
  sync();
}

void ViewGizmo::reproject()
{
  QVector3D dir = (m_cam_center - m_cam_pos).normalized();
  if (dir.isNull()) dir = QVector3D(0, -1, 0);
  QVector3D right = QVector3D::crossProduct(dir, m_cam_up).normalized();
  if (right.isNull()) right = QVector3D(1, 0, 0);
  const QVector3D up = QVector3D::crossProduct(right, dir).normalized();

  const float cx = 55.f;
  const float cy = 55.f;
  const float R  = 39.f;

  for (auto &h : m_handles) {
    const float sx = QVector3D::dotProduct(h.world, right);
    const float sy = QVector3D::dotProduct(h.world, up);
    h.depth    = QVector3D::dotProduct(h.world, dir);
    h.screen_pt = { int(cx + R * sx), int(cy - R * sy) };
  }
}

void ViewGizmo::redraw()
{
  const int W = 110;
  const int H = 110;
  QPixmap pix(W, H);
  pix.fill(QColor(0x25, 0x25, 0x25));

  QPainter p(&pix);
  p.setRenderHint(QPainter::Antialiasing);

  const int cx = W / 2;
  const int cy = H / 2;
  const int R  = 43;

  QRadialGradient bg(cx - 10, cy - 10, R * 1.3f);
  bg.setColorAt(0, QColor(72, 74, 86));
  bg.setColorAt(1, QColor(22, 23, 28));
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(QPoint(cx, cy), R, R);

  p.setPen(QColor(100, 100, 120, 100));
  p.setBrush(Qt::NoBrush);
  p.drawEllipse(QPoint(cx, cy), R, R);

  p.setPen(QColor(80, 80, 100, 55));
  p.drawLine(cx - R, cy, cx + R, cy);
  p.drawLine(cx, cy - R, cx, cy + R);

  std::array<int, 6> order = {0, 1, 2, 3, 4, 5};
  std::sort(order.begin(), order.end(), [this](int a, int b) {
    return m_handles[static_cast<size_t>(a)].depth < m_handles[static_cast<size_t>(b)].depth;
  });

  const int dotR = 13;
  for (int idx : order) {
    const Handle &h = m_handles[static_cast<size_t>(idx)];
    const bool  back  = h.depth < 0.f;
    const float alpha = back ? 0.32f : 1.0f;
    const int   r     = back ? dotR - 3 : dotR;

    if (!back) {
      p.setPen(Qt::NoPen);
      p.setBrush(QColor(0, 0, 0, 55));
      p.drawEllipse(QPoint(h.screen_pt.x() + 2, h.screen_pt.y() + 2), r, r);
    }

    QColor col = h.color;
    col.setAlphaF(alpha);
    p.setPen(Qt::NoPen);
    p.setBrush(col);
    p.drawEllipse(h.screen_pt, r, r);

    QFont f = p.font();
    f.setBold(true);
    f.setPixelSize(9);
    p.setFont(f);
    p.setPen(QColor(255, 255, 255, int(200 * alpha)));
    const QRect tr(h.screen_pt.x() - r, h.screen_pt.y() - r, r * 2, r * 2);
    p.drawText(tr, Qt::AlignCenter, h.label);
  }

  setPixmap(pix);
}

void ViewGizmo::mousePressEvent(QMouseEvent *ev)
{
  const QPoint pt = ev->pos();
  float bestD = 20.f;
  int   bestI = -1;
  for (int i = 0; i < 6; ++i) {
    const float dx = float(pt.x() - m_handles[static_cast<size_t>(i)].screen_pt.x());
    const float dy = float(pt.y() - m_handles[static_cast<size_t>(i)].screen_pt.y());
    const float d  = std::sqrt(dx * dx + dy * dy);
    if (d < bestD) { bestD = d; bestI = i; }
  }
  if (bestI < 0) { QLabel::mousePressEvent(ev); return; }

  const QVector3D snapPos[6] = {
    {1500, 0, 100}, {-1500, 0, 100},
    {0, 1500, 100}, {0, -1500, 100},
    {0, 0, 1500},   {0, 0, -1500},
  };
  const QVector3D snapUp[6] = {
    {0,0,1}, {0,0,1},
    {0,0,1}, {0,0,1},
    {0,1,0}, {0,1,0},
  };
  emit snap_requested(snapPos[bestI], QVector3D(0, 0, 80), snapUp[bestI]);
}

}
