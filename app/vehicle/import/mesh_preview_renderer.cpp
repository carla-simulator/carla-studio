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

void detectUnits(float maxExt, float &scaleToCm, QString &units)
{
  if (maxExt >= 50.f && maxExt <= 1000.f)         { scaleToCm = 1.0f;   units = "cm"; }
  else if (maxExt >= 0.05f && maxExt < 50.f)      { scaleToCm = 100.0f; units = "m";  }
  else if (maxExt >= 1000.f && maxExt <= 10000.f) { scaleToCm = 0.1f;   units = "mm"; }
  else                                            { scaleToCm = 1.0f;   units = "raw"; }
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

void axesFor(PreviewView v, int forwardAxis, int upAxis,
             int &h, int &vAxis, char &hL, char &vL)
{
  if (forwardAxis < 0 || forwardAxis > 2) forwardAxis = 0;
  if (upAxis      < 0 || upAxis      > 2) upAxis      = 2;
  if (upAxis == forwardAxis) upAxis = (forwardAxis + 1) % 3;
  const int lateralAxis = 3 - forwardAxis - upAxis;
  static const char axisChar[3] = { 'X', 'Y', 'Z' };
  switch (v) {
    case PreviewView::Top:
      h = forwardAxis; vAxis = lateralAxis;
      hL = axisChar[h]; vL = axisChar[vAxis]; break;
    case PreviewView::Side:
      h = forwardAxis; vAxis = upAxis;
      hL = axisChar[h]; vL = axisChar[vAxis]; break;
    case PreviewView::Front:
      h = lateralAxis; vAxis = upAxis;
      hL = axisChar[h]; vL = axisChar[vAxis]; break;
  }
}

}  // namespace

PreviewSummary analyseMesh(const MeshGeometry &g)
{
  PreviewSummary s;
  if (!g.valid || g.vertexCount() == 0) return s;
  s.valid = true;
  s.vertCount = g.vertexCount();
  s.faceCount = g.faceCount();
  s.malformedFaces = g.malformedFaces;

  float minV[3] = { 1e30f, 1e30f, 1e30f };
  float maxV[3] = {-1e30f,-1e30f,-1e30f };
  for (int i = 0; i < g.vertexCount(); ++i) {
    float x, y, z; g.vertex(i, x, y, z);
    if (x < minV[0]) minV[0] = x; if (x > maxV[0]) maxV[0] = x;
    if (y < minV[1]) minV[1] = y; if (y > maxV[1]) maxV[1] = y;
    if (z < minV[2]) minV[2] = z; if (z > maxV[2]) maxV[2] = z;
  }
  const float ext[3] = { maxV[0]-minV[0], maxV[1]-minV[1], maxV[2]-minV[2] };
  const float maxExt = std::max({ext[0], ext[1], ext[2]});
  detectUnits(maxExt, s.scaleToCm, s.unitsHint);
  for (int k = 0; k < 3; ++k) s.extCm[k] = ext[k] * s.scaleToCm;
  s.volumeM3 = (s.extCm[0] * s.extCm[1] * s.extCm[2]) / 1.0e6f;
  s.sizeClass = classify(s.volumeM3);

  s.forwardAxis = (ext[0] >= ext[1] && ext[0] >= ext[2]) ? 0
                : (ext[1] >= ext[2]) ? 1 : 2;
  {
    int cand[2], k = 0;
    for (int i = 0; i < 3; ++i) if (i != s.forwardAxis) cand[k++] = i;
    s.upAxis = (ext[cand[0]] <= ext[cand[1]]) ? cand[0] : cand[1];
  }
  const float ctr[3] = { 0.5f*(minV[0]+maxV[0]), 0.5f*(minV[1]+maxV[1]), 0.5f*(minV[2]+maxV[2]) };
  for (int i = 0; i < g.vertexCount(); ++i) {
    float x, y, z; g.vertex(i, x, y, z);
    const float v = (s.forwardAxis == 0) ? x : (s.forwardAxis == 1) ? y : z;
    if (v > ctr[s.forwardAxis]) ++s.vertsAhead; else ++s.vertsBehind;
  }
  s.forwardSign = s.vertsAhead >= s.vertsBehind ? +1 : -1;
  return s;
}

QImage renderPreview(const MeshGeometry &g,
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

  const float halfCm = std::max({
      std::fabs(sum.extCm[0]) * 0.6f + 100.f,
      std::fabs(sum.extCm[1]) * 0.6f + 100.f,
      std::fabs(sum.extCm[2]) * 0.6f + 100.f,
      300.f
  });
  const float pxPerCm = (pixels * 0.5f) / halfCm;
  const QPointF center(pixels * 0.5f, pixels * 0.5f);

  auto worldToImg = [&](float wx, float wy) -> QPointF {
    return QPointF(center.x() + wx * pxPerCm,
                   center.y() - wy * pxPerCm);  // y inverted
  };

  const float cellCm = 100.f;
  const int cells = static_cast<int>(std::ceil(halfCm / cellCm)) + 1;
  for (int i = -cells; i < cells; ++i) {
    for (int j = -cells; j < cells; ++j) {
      const QPointF a = worldToImg(i * cellCm, j * cellCm);
      const QPointF b = worldToImg((i+1) * cellCm, (j+1) * cellCm);
      const QRectF rect(QPointF(std::min(a.x(), b.x()), std::min(a.y(), b.y())),
                        QPointF(std::max(a.x(), b.x()), std::max(a.y(), b.y())));
      const QColor c = ((i + j) & 1) ? QColor(232, 232, 235) : QColor(248, 248, 252);
      p.fillRect(rect, c);
    }
  }
  p.setPen(QPen(QColor(190, 190, 195), 1));
  for (int i = -cells; i <= cells; ++i) {
    QPointF h0 = worldToImg(-cells * cellCm, i * cellCm);
    QPointF h1 = worldToImg( cells * cellCm, i * cellCm);
    QPointF v0 = worldToImg(i * cellCm, -cells * cellCm);
    QPointF v1 = worldToImg(i * cellCm,  cells * cellCm);
    p.drawLine(h0, h1);
    p.drawLine(v0, v1);
  }
  p.setPen(QPen(QColor(120, 120, 120), 1.5));
  p.drawLine(worldToImg(-halfCm, 0), worldToImg(halfCm, 0));
  p.drawLine(worldToImg(0, -halfCm), worldToImg(0, halfCm));

  int hAxis, vAxis;
  char hLab, vLab;
  axesFor(view, sum.forwardAxis, sum.upAxis, hAxis, vAxis, hLab, vLab);

  p.setPen(QColor(40, 90, 160, 160));
  const int nV = g.vertexCount();
  const int stride = std::max(1, nV / 8000);
  for (int i = 0; i < nV; i += stride) {
    float x, y, z; g.vertex(i, x, y, z);
    const float coords[3] = { x * sum.scaleToCm, y * sum.scaleToCm, z * sum.scaleToCm };
    const QPointF q = worldToImg(coords[hAxis], coords[vAxis]);
    p.drawPoint(q);
  }

  auto drawArrow = [&](char which, const QColor &col) {
    int axis = (which == 'X') ? 0 : (which == 'Y') ? 1 : 2;
    const float len = halfCm * 0.55f;
    QPointF tipWorld;
    if (axis == hAxis)      tipWorld = QPointF(len, 0);
    else if (axis == vAxis) tipWorld = QPointF(0, len);
    else return;
    QPointF tip = worldToImg(tipWorld.x(), tipWorld.y());
    QPointF base = center;
    p.setPen(QPen(col, 3));
    p.drawLine(base, tip);
    QPointF dir = tip - base;
    const float L = std::hypot(dir.x(), dir.y());
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
    int fa = sum.forwardAxis;
    if (fa == hAxis || fa == vAxis) {
      const float len = halfCm * 0.7f * sum.forwardSign;
      QPointF tipW = (fa == hAxis) ? QPointF(len, 0) : QPointF(0, len);
      QPointF tip = worldToImg(tipW.x(), tipW.y());
      p.setPen(QPen(QColor(255, 80, 0), 4, Qt::DashLine));
      p.drawLine(center, tip);
      char fn; axisLabel(fa, fn);
      p.setPen(QColor(255, 80, 0));
      QFont f = p.font(); f.setBold(true); f.setPointSize(11); p.setFont(f);
      p.drawText(tip + QPointF(8, 8),
                 QString("forward = %1%2").arg(sum.forwardSign>0?'+':'-').arg(fn));
    }
  }

  if (wheels.set && view == PreviewView::Top) {
    const QVector3D pts[4] = { wheels.fl, wheels.fr, wheels.rl, wheels.rr };
    const QColor cols[4] = { QColor(255,200,0), QColor(255,120,0),
                             QColor(0,200,255), QColor(0,120,255) };
    const QString labels[4] = { "FL","FR","RL","RR" };
    for (int i = 0; i < 4; ++i) {
      const QPointF q = worldToImg(pts[i].x(), pts[i].y());
      p.setBrush(cols[i]);
      p.setPen(QPen(QColor(20,20,20), 1.5));
      p.drawEllipse(q, 8, 8);
      p.setPen(QColor(20, 20, 20));
      p.drawText(q + QPointF(10, 4), labels[i]);
    }
  }

  p.setPen(QColor(20, 20, 20));
  QFont h = p.font(); h.setBold(true); h.setPointSize(12); p.setFont(h);
  p.fillRect(QRectF(0, 0, pixels, 28), QColor(245, 245, 245, 230));
  p.drawText(QRectF(8, 4, pixels - 16, 22), Qt::AlignVCenter,
             QString("%1   |   %2   |   %3 × %4 × %5 m   |   units=%6")
                 .arg(viewName(view))
                 .arg(sum.sizeClass)
                 .arg(sum.extCm[0]/100.0, 0, 'f', 2)
                 .arg(sum.extCm[1]/100.0, 0, 'f', 2)
                 .arg(sum.extCm[2]/100.0, 0, 'f', 2)
                 .arg(sum.unitsHint));

  p.end();
  return img;
}

}  // namespace carla_studio::vehicle_import
