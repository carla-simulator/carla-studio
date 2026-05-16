// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "mesh_preview_renderer.h"
#include "mesh_geometry.h"

#include <QPainter>
#include <QPainterPath>
#include <QFont>
#include <QFontMetrics>

#include <algorithm>
#include <cmath>

namespace carla_studio::vehicle_import {

namespace {

void detectUnits(float maxExt, float &scale_to_cm, QString &units)
{
  if (maxExt >= 50.f && maxExt <= 1000.f)         { scale_to_cm = 1.0f;   units = "cm"; }
  else if (maxExt >= 0.05f && maxExt < 50.f)      { scale_to_cm = 100.0f; units = "m";  }
  else if (maxExt >= 1000.f && maxExt <= 10000.f) { scale_to_cm = 0.1f;   units = "mm"; }
  else                                            { scale_to_cm = 1.0f;   units = "raw"; }
}

const char *classify(float volM3)
{
  if (volM3 < 3.0f)   return "kart";
  if (volM3 < 12.0f)  return "sedan";
  if (volM3 < 25.0f)  return "suv";
  return "truck";
}

void axisLabel(int axis, char &out) { out = (axis == 0) ? 'X' : (axis == 1) ? 'Y' : 'Z'; }

const char *viewName(PreviewView v) {
  switch (v) {
    case PreviewView::Top:   return "TOP (XY)";
    case PreviewView::Side:  return "SIDE (XZ)";
    case PreviewView::Front: return "FRONT (YZ)";
  }
  return "?";
}

void axesFor(PreviewView v, int forward_axis, int up_axis,
             int &h, int &vAxis, char &hL, char &vL)
{
  if (forward_axis < 0 || forward_axis > 2) forward_axis = 0;
  if (up_axis      < 0 || up_axis      > 2) up_axis      = 2;
  if (up_axis == forward_axis) up_axis = (forward_axis + 1) % 3;
  const int lateralAxis = 3 - forward_axis - up_axis;
  static const char axisChar[3] = { 'X', 'Y', 'Z' };
  switch (v) {
    case PreviewView::Top:
      h = forward_axis; vAxis = lateralAxis;
      hL = axisChar[h]; vL = axisChar[vAxis]; break;
    case PreviewView::Side:
      h = forward_axis; vAxis = up_axis;
      hL = axisChar[h]; vL = axisChar[vAxis]; break;
    case PreviewView::Front:
      h = lateralAxis; vAxis = up_axis;
      hL = axisChar[h]; vL = axisChar[vAxis]; break;
  }
}

}

PreviewSummary analyse_mesh(const MeshGeometry &g)
{
  PreviewSummary s;
  if (!g.valid || g.vertex_count() == 0) return s;
  s.valid = true;
  s.vert_count = g.vertex_count();
  s.face_count = g.face_count();
  s.malformed_faces = g.malformed_faces;

  float minV[3] = { 1e30f, 1e30f, 1e30f };
  float maxV[3] = {-1e30f,-1e30f,-1e30f };
  for (int i = 0; i < g.vertex_count(); ++i) {
    float x, y, z; g.vertex(i, x, y, z);
    if (x < minV[0]) minV[0] = x;
    if (x > maxV[0]) maxV[0] = x;
    if (y < minV[1]) minV[1] = y;
    if (y > maxV[1]) maxV[1] = y;
    if (z < minV[2]) minV[2] = z;
    if (z > maxV[2]) maxV[2] = z;
  }
  const float ext[3] = { maxV[0]-minV[0], maxV[1]-minV[1], maxV[2]-minV[2] };
  const float maxExt = std::max({ext[0], ext[1], ext[2]});
  detectUnits(maxExt, s.scale_to_cm, s.units_hint);
  for (int k = 0; k < 3; ++k) s.ext_cm[k] = ext[k] * s.scale_to_cm;
  s.volume_m3 = (s.ext_cm[0] * s.ext_cm[1] * s.ext_cm[2]) / 1.0e6f;
  s.size_class = classify(s.volume_m3);

  s.forward_axis = (ext[0] >= ext[1] && ext[0] >= ext[2]) ? 0
                : (ext[1] >= ext[2]) ? 1 : 2;
  {
    int cand[2], k = 0;
    for (int i = 0; i < 3; ++i) if (i != s.forward_axis) cand[k++] = i;
    s.up_axis = (ext[cand[0]] <= ext[cand[1]]) ? cand[0] : cand[1];
  }
  const float ctr[3] = { 0.5f*(minV[0]+maxV[0]), 0.5f*(minV[1]+maxV[1]), 0.5f*(minV[2]+maxV[2]) };
  for (int i = 0; i < g.vertex_count(); ++i) {
    float x, y, z; g.vertex(i, x, y, z);
    const float v = (s.forward_axis == 0) ? x : (s.forward_axis == 1) ? y : z;
    if (v > ctr[s.forward_axis]) ++s.verts_ahead; else ++s.verts_behind;
  }
  s.forward_sign = s.verts_ahead >= s.verts_behind ? +1 : -1;
  return s;
}

QImage render_preview(const MeshGeometry &g,
                     const PreviewSummary &sum,
                     PreviewView view,
                     int pixels,
                     const PreviewWheels &wheels)
{
  QImage img(pixels, pixels, QImage::Format_ARGB32);
  img.fill(QColor(255, 255, 255));
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.setRenderHint(QPainter::TextAntialiasing, true);

  const float half_cm = std::max({
      std::fabs(sum.ext_cm[0]) * 0.6f + 100.f,
      std::fabs(sum.ext_cm[1]) * 0.6f + 100.f,
      std::fabs(sum.ext_cm[2]) * 0.6f + 100.f,
      300.f
  });
  const float pxPerCm = (static_cast<float>(pixels) * 0.5f) / half_cm;
  const QPointF center(static_cast<qreal>(pixels) * 0.5, static_cast<qreal>(pixels) * 0.5);

  auto world_to_img = [&](float wx, float wy) -> QPointF {
    const qreal qpx = static_cast<qreal>(pxPerCm);
    return QPointF(center.x() + static_cast<qreal>(wx) * qpx,
                   center.y() - static_cast<qreal>(wy) * qpx);
  };

  const float cell_cm = 100.f;
  const int cells = static_cast<int>(std::ceil(half_cm / cell_cm)) + 1;
  for (int i = -cells; i < cells; ++i) {
    for (int j = -cells; j < cells; ++j) {
      const QPointF a = world_to_img(static_cast<float>(i) * cell_cm, static_cast<float>(j) * cell_cm);
      const QPointF b = world_to_img(static_cast<float>(i+1) * cell_cm, static_cast<float>(j+1) * cell_cm);
      const QRectF rect(QPointF(std::min(a.x(), b.x()), std::min(a.y(), b.y())),
                        QPointF(std::max(a.x(), b.x()), std::max(a.y(), b.y())));
      const QColor c = ((i + j) & 1) ? QColor(232, 232, 235) : QColor(248, 248, 252);
      p.fillRect(rect, c);
    }
  }
  p.setPen(QPen(QColor(190, 190, 195), 1));
  for (int i = -cells; i <= cells; ++i) {
    QPointF h0 = world_to_img(static_cast<float>(-cells) * cell_cm, static_cast<float>(i) * cell_cm);
    QPointF h1 = world_to_img(static_cast<float>( cells) * cell_cm, static_cast<float>(i) * cell_cm);
    QPointF v0 = world_to_img(static_cast<float>(i) * cell_cm, static_cast<float>(-cells) * cell_cm);
    QPointF v1 = world_to_img(static_cast<float>(i) * cell_cm, static_cast<float>( cells) * cell_cm);
    p.drawLine(h0, h1);
    p.drawLine(v0, v1);
  }
  p.setPen(QPen(QColor(120, 120, 120), 1.5));
  p.drawLine(world_to_img(-half_cm, 0), world_to_img(half_cm, 0));
  p.drawLine(world_to_img(0, -half_cm), world_to_img(0, half_cm));

  int hAxis, vAxis;
  char hLab, vLab;
  axesFor(view, sum.forward_axis, sum.up_axis, hAxis, vAxis, hLab, vLab);

  p.setPen(QColor(40, 90, 160, 160));
  const int nV = g.vertex_count();
  const int stride = std::max(1, nV / 8000);
  for (int i = 0; i < nV; i += stride) {
    float x, y, z; g.vertex(i, x, y, z);
    const float coords[3] = { x * sum.scale_to_cm, y * sum.scale_to_cm, z * sum.scale_to_cm };
    const QPointF q = world_to_img(coords[hAxis], coords[vAxis]);
    p.drawPoint(q);
  }

  auto drawArrow = [&](char which, const QColor &col) {
    int axis = (which == 'X') ? 0 : (which == 'Y') ? 1 : 2;
    const float len = half_cm * 0.55f;
    QPointF tip_world;
    if (axis == hAxis)      tip_world = QPointF(static_cast<qreal>(len), 0);
    else if (axis == vAxis) tip_world = QPointF(0, static_cast<qreal>(len));
    else return;
    QPointF tip = world_to_img(static_cast<float>(tip_world.x()), static_cast<float>(tip_world.y()));
    QPointF base = center;
    p.setPen(QPen(col, 3));
    p.drawLine(base, tip);
    QPointF dir = tip - base;
    const qreal L = std::hypot(dir.x(), dir.y());
    if (L > 1e-3) {
      dir /= L;
      QPointF perp(-dir.y(), dir.x());
      QPointF h1 = tip - dir * 16 + perp * 8;
      QPointF h2 = tip - dir * 16 - perp * 8;
      p.setBrush(col);
      p.drawPolygon(QPolygonF{ tip, h1, h2 });
    }
    p.setPen(col);
    QFont f = p.font(); f.setBold(true); p.setFont(f);
    p.drawText(tip + QPointF(6, -6), QString("+") + which);
  };
  drawArrow('X', QColor(220,  60,  60));
  drawArrow('Y', QColor( 60, 200,  60));
  drawArrow('Z', QColor( 80, 130, 255));

  {
    int fa = sum.forward_axis;
    if (fa == hAxis || fa == vAxis) {
      const float len = half_cm * 0.7f * static_cast<float>(sum.forward_sign);
      QPointF tipW = (fa == hAxis) ? QPointF(static_cast<qreal>(len), 0) : QPointF(0, static_cast<qreal>(len));
      QPointF tip = world_to_img(static_cast<float>(tipW.x()), static_cast<float>(tipW.y()));
      p.setPen(QPen(QColor(255, 80, 0), 4, Qt::DashLine));
      p.drawLine(center, tip);
      char fn; axisLabel(fa, fn);
      p.setPen(QColor(255, 80, 0));
      QFont f = p.font(); f.setBold(true); f.setPointSize(11); p.setFont(f);
      p.drawText(tip + QPointF(8, 8),
                 QString("forward = %1%2").arg(sum.forward_sign>0?'+':'-').arg(fn));
    }
  }

  if (wheels.set && view == PreviewView::Top) {
    const QVector3D pts[4] = { wheels.fl, wheels.fr, wheels.rl, wheels.rr };
    const QColor cols[4] = { QColor(255,200,0), QColor(255,120,0),
                             QColor(0,200,255), QColor(0,120,255) };
    const QString labels[4] = { "FL","FR","RL","RR" };
    for (int i = 0; i < 4; ++i) {
      const QPointF q = world_to_img(pts[i].x(), pts[i].y());
      p.setBrush(cols[i]);
      p.setPen(QPen(QColor(20,20,20), 1.5));
      p.drawEllipse(q, 8, 8);
      p.setPen(QColor(20, 20, 20));
      p.drawText(q + QPointF(10, 4), labels[i]);
    }
  }

  p.setPen(QColor(20, 20, 20));
  QFont h = p.font(); h.setBold(true); h.setPointSize(12); p.setFont(h);
  p.fillRect(QRectF(0, 0, static_cast<qreal>(pixels), 28), QColor(245, 245, 245, 230));
  p.drawText(QRectF(8, 4, static_cast<qreal>(pixels - 16), 22), Qt::AlignVCenter,
             QString("%1   |   %2   |   %3 × %4 × %5 m   |   units=%6")
                 .arg(viewName(view))
                 .arg(sum.size_class)
                 .arg(static_cast<double>(sum.ext_cm[0])/100.0, 0, 'f', 2)
                 .arg(static_cast<double>(sum.ext_cm[1])/100.0, 0, 'f', 2)
                 .arg(static_cast<double>(sum.ext_cm[2])/100.0, 0, 'f', 2)
                 .arg(sum.units_hint));

  p.end();
  return img;
}

}
