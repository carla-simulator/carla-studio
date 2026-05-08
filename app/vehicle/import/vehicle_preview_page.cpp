// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_preview_page.h"
#include "view_gizmo.h"

#ifndef CARLA_STUDIO_WITH_QT3D
namespace carla_studio::vehicle_import {
VehiclePreviewPage::VehiclePreviewPage(QWidget *parent) : QWidget(parent) {}
void VehiclePreviewPage::loadMesh(const QString&) {}
void VehiclePreviewPage::setWheelPositionsCm(const QVector3D&, const QVector3D&,
                                             const QVector3D&, const QVector3D&) {}
void VehiclePreviewPage::resetCamera() {}
void VehiclePreviewPage::setToolMode(ToolMode) {}
void VehiclePreviewPage::onBrowse() {}
void VehiclePreviewPage::onLoadClicked() {}
void VehiclePreviewPage::onSceneStatusChanged(int) {}
void VehiclePreviewPage::buildScene() {}
void VehiclePreviewPage::buildGridFloor(Qt3DCore::QEntity*, float, float) {}
void VehiclePreviewPage::buildAxisGizmo(Qt3DCore::QEntity*, float) {}
void VehiclePreviewPage::buildReferenceOutlines(Qt3DCore::QEntity*) {}
void VehiclePreviewPage::buildWheelMarkers(Qt3DCore::QEntity*) {}
void VehiclePreviewPage::buildAlignmentRods(Qt3DCore::QEntity*) {}
void VehiclePreviewPage::updateAlignmentLines() {}
void VehiclePreviewPage::updateMeshBounds() {}
void VehiclePreviewPage::viewTop() {}
void VehiclePreviewPage::viewSide() {}
void VehiclePreviewPage::viewFront() {}
void VehiclePreviewPage::view3D() {}
void VehiclePreviewPage::fitCamera() {}
void VehiclePreviewPage::resizeEvent(QResizeEvent *ev) { QWidget::resizeEvent(ev); }
void VehiclePreviewPage::rotateMinus90() {}
void VehiclePreviewPage::rotatePlus90() {}
void VehiclePreviewPage::rotate180() {}
void VehiclePreviewPage::mirrorX() {}
void VehiclePreviewPage::mirrorY() {}
void VehiclePreviewPage::recenterMesh() {}
void VehiclePreviewPage::resetMeshTransform() {}
void VehiclePreviewPage::applyAdjustment(const QString&) {}
}
#else

#include "mesh_geometry.h"

#include <QClipboard>
#include <QDateTime>
#include <QDir>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QGuiApplication>
#include <QPixmap>
#include <QResizeEvent>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QStackedWidget>
#include <QVBoxLayout>

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QPhongAlphaMaterial>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QExtrudedTextMesh>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QPointLight>
#include <Qt3DRender/QDirectionalLight>
#include <Qt3DRender/QSceneLoader>
#include <QUrl>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QCameraLens>
#include <Qt3DRender/QGeometryRenderer>
#include <Qt3DCore/QGeometry>
#include <Qt3DCore/QAttribute>
#include <Qt3DCore/QBuffer>
#include <Qt3DExtras/QForwardRenderer>

#include <algorithm>
#include <cmath>

namespace carla_studio::vehicle_import {

namespace {

Qt3DCore::QEntity *makeAxis(Qt3DCore::QEntity *parent,
                            const QVector3D &axisCm,
                            const QColor &color)
{
  auto *e = new Qt3DCore::QEntity(parent);
  auto *m = new Qt3DExtras::QCylinderMesh();
  const float L = axisCm.length();
  m->setRadius(2.0f);
  m->setLength(L);
  m->setRings(2);
  m->setSlices(8);
  auto *mat = new Qt3DExtras::QPhongMaterial();
  mat->setDiffuse(color);
  mat->setAmbient(color);
  mat->setSpecular(QColor(255, 255, 255));
  mat->setShininess(20.0f);
  auto *t = new Qt3DCore::QTransform();
  const QVector3D dir = axisCm.normalized();
  const QVector3D yUp(0, 1, 0);
  const QVector3D rotAxis = QVector3D::crossProduct(yUp, dir);
  const float rotAngle = std::acos(QVector3D::dotProduct(yUp, dir)) * 180.0f / float(M_PI);
  if (rotAxis.length() > 1e-4f) {
    t->setRotation(QQuaternion::fromAxisAndAngle(rotAxis.normalized(), rotAngle));
  }
  t->setTranslation(axisCm * 0.5f);
  e->addComponent(m);
  e->addComponent(mat);
  e->addComponent(t);
  return e;
}

Qt3DCore::QEntity *makeRodXY(Qt3DCore::QEntity *parent,
                             const QVector3D &a, const QVector3D &b,
                             float radius, const QColor &color)
{
  auto *e = new Qt3DCore::QEntity(parent);
  auto *m = new Qt3DExtras::QCylinderMesh();
  const QVector3D d = b - a;
  const float L = d.length();
  m->setRadius(radius);
  m->setLength(L);
  m->setRings(2);
  m->setSlices(8);
  auto *mat = new Qt3DExtras::QPhongMaterial();
  mat->setDiffuse(color);
  mat->setAmbient(color);
  auto *t = new Qt3DCore::QTransform();
  if (L > 1e-3f) {
    const QVector3D dir = d / L;
    const QVector3D yUp(0, 1, 0);
    const QVector3D rotAxis = QVector3D::crossProduct(yUp, dir);
    const float dot = QVector3D::dotProduct(yUp, dir);
    const float ang = std::acos(std::clamp(dot, -1.f, 1.f)) * 180.0f / float(M_PI);
    if (rotAxis.length() > 1e-4f)
      t->setRotation(QQuaternion::fromAxisAndAngle(rotAxis.normalized(), ang));
    else if (dot < 0)
      t->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1,0,0), 180.f));
  }
  t->setTranslation((a + b) * 0.5f);
  e->addComponent(m);
  e->addComponent(mat);
  e->addComponent(t);
  return e;
}

void makeStencilRect(Qt3DCore::QEntity *parent,
                     float halfW, float halfL, float zLift,
                     float radius, const QColor &color)
{
  const QVector3D fl(-halfW,  halfL, zLift), fr( halfW,  halfL, zLift);
  const QVector3D rl(-halfW, -halfL, zLift), rr( halfW, -halfL, zLift);
  makeRodXY(parent, fl, fr, radius, color);   // front
  makeRodXY(parent, rl, rr, radius, color);   // rear
  makeRodXY(parent, fl, rl, radius, color);   // left
  makeRodXY(parent, fr, rr, radius, color);   // right
}

// Lay flat text on the ground plane, readable from above (top view) and
// from forward-down (3D view). The text mesh defaults to standing upright
// in the XY plane, so we rotate it -90° about X to lie on the floor with
// the baseline along +X (lateral).
Qt3DCore::QEntity *makeTextLabelOnGround(Qt3DCore::QEntity *parent,
                                         const QString &text,
                                         const QVector3D &posCm,
                                         float heightCm,
                                         const QColor &color)
{
  auto *e = new Qt3DCore::QEntity(parent);
  auto *mesh = new Qt3DExtras::QExtrudedTextMesh();
  mesh->setText(text);
  mesh->setDepth(0.4f);
  QFont f("Sans", 14);
  f.setBold(true);
  mesh->setFont(f);
  auto *mat = new Qt3DExtras::QPhongMaterial();
  mat->setDiffuse(color);
  mat->setAmbient(color);
  mat->setSpecular(QColor(255, 255, 255));
  mat->setShininess(8.f);
  auto *t = new Qt3DCore::QTransform();
  // QExtrudedTextMesh's local font units are ~design units, much smaller
  // than 1 cm. Scale so the rendered glyph is `heightCm` tall.
  const float scale = heightCm / 14.f;
  t->setScale(scale);
  // Lie flat on ground, baseline along +X.
  QQuaternion qx = QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), -90.f);
  t->setRotation(qx);
  t->setTranslation(posCm);
  e->addComponent(mesh);
  e->addComponent(mat);
  e->addComponent(t);
  return e;
}

void updateAlignRod(Qt3DExtras::QCylinderMesh *mesh,
                    Qt3DCore::QTransform *xform,
                    const QVector3D &a, const QVector3D &b)
{
  const QVector3D d = b - a;
  const float L = d.length();
  mesh->setLength(std::max(L, 0.1f));
  if (L > 1e-3f) {
    const QVector3D dir = d / L;
    const QVector3D yUp(0, 1, 0);
    const QVector3D rotAxis = QVector3D::crossProduct(yUp, dir);
    const float dot = QVector3D::dotProduct(yUp, dir);
    const float ang = std::acos(std::clamp(dot, -1.f, 1.f)) * 180.f / float(M_PI);
    if (rotAxis.length() > 1e-4f)
      xform->setRotation(QQuaternion::fromAxisAndAngle(rotAxis.normalized(), ang));
    else if (dot < 0)
      xform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1,0,0), 180.f));
    else
      xform->setRotation(QQuaternion());
  }
  xform->setTranslation((a + b) * 0.5f);
}

Qt3DCore::QEntity *makeCheckerCell(Qt3DCore::QEntity *parent,
                                   float xCm, float yCm, float cellCm,
                                   const QColor &color)
{
  auto *e = new Qt3DCore::QEntity(parent);
  auto *m = new Qt3DExtras::QPlaneMesh();
  m->setWidth(cellCm);
  m->setHeight(cellCm);
  auto *mat = new Qt3DExtras::QPhongMaterial();
  mat->setDiffuse(color);
  mat->setAmbient(color);
  mat->setSpecular(QColor(40, 40, 40));
  mat->setShininess(2.0f);
  auto *t = new Qt3DCore::QTransform();
  t->setRotationX(-90.0f);
  t->setTranslation(QVector3D(xCm + cellCm * 0.5f, yCm + cellCm * 0.5f, 0.0f));
  e->addComponent(m);
  e->addComponent(mat);
  e->addComponent(t);
  return e;
}

}  // namespace

VehiclePreviewPage::VehiclePreviewPage(QWidget *parent) : QWidget(parent)
{
  // ── Global dark theme (Photoshop-style) ──────────────────────────────────
  setStyleSheet(
    "QWidget {"
    "  background-color: #1E1E1E; color: #C8C8C8;"
    "  font-family: 'Segoe UI', sans-serif; font-size: 12px; }"
    "QPushButton {"
    "  background-color: #3A3A3A; color: #C8C8C8;"
    "  border: 1px solid #555; padding: 3px 10px;"
    "  border-radius: 2px; }"
    "QPushButton:hover  { background-color: #505050; border-color: #888; }"
    "QPushButton:pressed { background-color: #0078D4; color: #FFF; border-color: #0078D4; }"
    "QLineEdit {"
    "  background-color: #2A2A2A; color: #CCC;"
    "  border: 1px solid #555; padding: 3px 6px; border-radius: 2px; }"
    "QLabel { background: transparent; color: #C8C8C8; }"
    "QPlainTextEdit {"
    "  background-color: #141414; color: #A0A0A0;"
    "  border: 1px solid #333; font-family: monospace; font-size: 11px; }");

  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(0);

  // ── Top bar: file path ────────────────────────────────────────────────────
  auto *topBar = new QWidget(this);
  topBar->setFixedHeight(34);
  topBar->setStyleSheet("QWidget { background-color: #252525;"
                        "  border-bottom: 1px solid #333; }");
  auto *topRow = new QHBoxLayout(topBar);
  topRow->setContentsMargins(8, 3, 8, 3);
  topRow->setSpacing(6);
  auto *meshLbl = new QLabel("Mesh", topBar);
  meshLbl->setStyleSheet("color: #888; font-size: 11px;");
  topRow->addWidget(meshLbl);
  mPathEdit = new QLineEdit(topBar);
  mPathEdit->setPlaceholderText("vehicle.obj / .fbx");
  mPathEdit->setStyleSheet("QLineEdit { background:#2A2A2A; color:#CCC;"
                           "  border:1px solid #444; padding:2px 6px; }");
  topRow->addWidget(mPathEdit, 1);
  mBrowseBtn = new QPushButton("Browse…", topBar);
  mLoadBtn   = new QPushButton("Load",    topBar);
  mBrowseBtn->setFixedHeight(24);
  mLoadBtn->setFixedHeight(24);
  topRow->addWidget(mBrowseBtn);
  topRow->addWidget(mLoadBtn);
  outer->addWidget(topBar);

  // ── View toolbar ─────────────────────────────────────────────────────────
  auto *viewBar = new QWidget(this);
  viewBar->setFixedHeight(30);
  viewBar->setStyleSheet("QWidget { background-color: #2A2A2A;"
                         "  border-bottom: 1px solid #333; }");
  auto *viewRow2 = new QHBoxLayout(viewBar);
  viewRow2->setContentsMargins(8, 2, 8, 2);
  viewRow2->setSpacing(3);
  auto mkViewBtn = [&](const QString &lbl, const QString &tip,
                        void (VehiclePreviewPage::*slot)()) {
    auto *b = new QPushButton(lbl, viewBar);
    b->setToolTip(tip);
    b->setFixedHeight(22);
    b->setStyleSheet("QPushButton { min-width:38px; font-size:11px; }");
    connect(b, &QPushButton::clicked, this, slot);
    viewRow2->addWidget(b);
  };
  mkViewBtn("Top",   "Top-down view",       &VehiclePreviewPage::viewTop);
  mkViewBtn("Side",  "Side view",           &VehiclePreviewPage::viewSide);
  mkViewBtn("Front", "Front view",          &VehiclePreviewPage::viewFront);
  mkViewBtn("3D",    "Perspective / orbit", &VehiclePreviewPage::view3D);
  mkViewBtn("Fit",   "Fit camera to mesh",  &VehiclePreviewPage::fitCamera);
  mResetCamBtn = new QPushButton("⟳ Reset", viewBar);
  mResetCamBtn->setToolTip("Reset camera");
  mResetCamBtn->setFixedHeight(22);
  mResetCamBtn->setStyleSheet("QPushButton { min-width:48px; font-size:11px; }");
  connect(mResetCamBtn, &QPushButton::clicked, this, &VehiclePreviewPage::resetCamera);
  viewRow2->addWidget(mResetCamBtn);

  viewRow2->addSpacing(10);
  auto addSep = [&]() {
    auto *s = new QLabel("│", viewBar); s->setStyleSheet("color:#555;");
    viewRow2->addWidget(s);
  };
  addSep();
  viewRow2->addSpacing(4);

  auto *captureBtn = new QPushButton("⬤ Capture", viewBar);
  captureBtn->setToolTip("Save PNG (path copied to clipboard)");
  captureBtn->setFixedHeight(22);
  captureBtn->setStyleSheet("QPushButton { min-width:70px; font-size:11px; color:#80C8FF; }");
  connect(captureBtn, &QPushButton::clicked, this, &VehiclePreviewPage::captureWindow);
  viewRow2->addWidget(captureBtn);

  viewRow2->addStretch();

  // Mode toggle - Import / Sensor Assembly
  const QString modeBase =
    "QPushButton { font-size:10px; font-weight:600; padding:0 10px; height:22px;"
    "  border:1px solid #555; background:#2E2E2E; color:#999; }"
    "QPushButton:checked { background:#0078D4; color:#FFF; border-color:#0078D4; }"
    "QPushButton:hover:!checked { background:#444; }";
  mImportModeBtn = new QPushButton("Import", viewBar);
  mImportModeBtn->setCheckable(true);
  mImportModeBtn->setChecked(true);
  mImportModeBtn->setFixedHeight(22);
  mImportModeBtn->setStyleSheet(modeBase + "QPushButton { border-radius:2px 0 0 2px; }");
  mSensorModeBtn = new QPushButton("Sensor Assembly", viewBar);
  mSensorModeBtn->setCheckable(true);
  mSensorModeBtn->setChecked(false);
  mSensorModeBtn->setFixedHeight(22);
  mSensorModeBtn->setStyleSheet(modeBase + "QPushButton { border-radius:0 2px 2px 0;"
                                           "  border-left:none; }");
  connect(mImportModeBtn, &QPushButton::clicked, this, [this]() {
    mImportModeBtn->setChecked(true);
    mSensorModeBtn->setChecked(false);
    setToolMode(ToolMode::Import);
  });
  connect(mSensorModeBtn, &QPushButton::clicked, this, [this]() {
    mSensorModeBtn->setChecked(true);
    mImportModeBtn->setChecked(false);
    setToolMode(ToolMode::SensorAssembly);
  });
  viewRow2->addWidget(mImportModeBtn);
  viewRow2->addWidget(mSensorModeBtn);
  viewRow2->addSpacing(10);
  addSep();
  viewRow2->addSpacing(6);

  mApplyBtn = new QPushButton("▶  Apply & Re-import", viewBar);
  mApplyBtn->setToolTip("Send adjustments to importer and re-cook");
  mApplyBtn->setFixedHeight(22);
  mApplyBtn->setStyleSheet(
    "QPushButton { background:#0078D4; color:#FFF; font-weight:600;"
    "  border:none; border-radius:2px; padding:0 12px; font-size:11px; }"
    "QPushButton:hover { background:#0090F0; }"
    "QPushButton:pressed { background:#005FAD; }");
  connect(mApplyBtn, &QPushButton::clicked, this, &VehiclePreviewPage::applyToSpec);
  viewRow2->addWidget(mApplyBtn);
  outer->addWidget(viewBar);

  // ── Main area: [left stack] [3D viewport] [right stack] ──────────────────
  buildScene();
  mContainer = QWidget::createWindowContainer(mView, this);
  mContainer->setMinimumSize(480, 380);
  mContainer->setFocusPolicy(Qt::StrongFocus);

  // ── Left stack: page 0 = Import ADJ tools, page 1 = Sensor tools ─────────
  mLeftStack = new QStackedWidget(this);
  mLeftStack->setFixedWidth(54);

  // Page 0: Import - Adjust operations
  auto *importLeftPage = new QWidget();
  importLeftPage->setStyleSheet("QWidget { background-color: #252525;"
                                "  border-right: 1px solid #333; }");
  auto *importLeftLayout = new QVBoxLayout(importLeftPage);
  importLeftLayout->setContentsMargins(4, 8, 4, 8);
  importLeftLayout->setSpacing(3);
  auto mkToolBtn = [&](QWidget *parent, QVBoxLayout *layout,
                        const QString &lbl, const QString &tip,
                        void (VehiclePreviewPage::*slot)()) {
    auto *b = new QPushButton(lbl, parent);
    b->setToolTip(tip);
    b->setFixedSize(46, 34);
    b->setStyleSheet(
      "QPushButton { font-size:10px; padding:0; border-radius:3px;"
      "  background:#2E2E2E; border:1px solid #444; }"
      "QPushButton:hover  { background:#484848; border-color:#888; }"
      "QPushButton:pressed { background:#0078D4; border-color:#0078D4; }");
    connect(b, &QPushButton::clicked, this, slot);
    layout->addWidget(b);
  };
  auto *adjLbl = new QLabel("ADJ", importLeftPage);
  adjLbl->setAlignment(Qt::AlignCenter);
  adjLbl->setStyleSheet("color:#666; font-size:9px; font-weight:bold;");
  importLeftLayout->addWidget(adjLbl);
  mkToolBtn(importLeftPage, importLeftLayout,
            "↺ -90",  "Rotate -90°",          &VehiclePreviewPage::rotateMinus90);
  mkToolBtn(importLeftPage, importLeftLayout,
            "↻ +90",  "Rotate +90°",          &VehiclePreviewPage::rotatePlus90);
  mkToolBtn(importLeftPage, importLeftLayout,
            "↕ 180",  "Flip 180°",            &VehiclePreviewPage::rotate180);
  importLeftLayout->addSpacing(6);
  mkToolBtn(importLeftPage, importLeftLayout,
            "⟺ X",   "Mirror across X axis",  &VehiclePreviewPage::mirrorX);
  mkToolBtn(importLeftPage, importLeftLayout,
            "⟺ Y",   "Mirror across Y axis",  &VehiclePreviewPage::mirrorY);
  importLeftLayout->addSpacing(6);
  mkToolBtn(importLeftPage, importLeftLayout,
            "⊕ Ctr", "Recenter mesh",         &VehiclePreviewPage::recenterMesh);
  mkToolBtn(importLeftPage, importLeftLayout,
            "✕ Rst", "Reset all adjustments", &VehiclePreviewPage::resetMeshTransform);
  importLeftLayout->addStretch();
  mLeftStack->addWidget(importLeftPage);

  // Page 1: Sensor Assembly - scene manipulation stubs
  auto *sensorLeftPage = new QWidget();
  sensorLeftPage->setStyleSheet("QWidget { background-color: #252525;"
                                "  border-right: 1px solid #333; }");
  auto *sensorLeftLayout = new QVBoxLayout(sensorLeftPage);
  sensorLeftLayout->setContentsMargins(4, 8, 4, 8);
  sensorLeftLayout->setSpacing(3);
  auto *senLbl = new QLabel("SEN", sensorLeftPage);
  senLbl->setAlignment(Qt::AlignCenter);
  senLbl->setStyleSheet("color:#666; font-size:9px; font-weight:bold;");
  sensorLeftLayout->addWidget(senLbl);
  const QString stubBtn =
    "QPushButton { font-size:10px; padding:0; border-radius:3px;"
    "  background:#2E2E2E; border:1px solid #444; color:#888; }"
    "QPushButton:hover { background:#383838; border-color:#666; }";
  auto mkSenBtn = [&](const QString &lbl, const QString &tip) {
    auto *b = new QPushButton(lbl, sensorLeftPage);
    b->setToolTip(tip);
    b->setFixedSize(46, 34);
    b->setStyleSheet(stubBtn);
    sensorLeftLayout->addWidget(b);
  };
  mkSenBtn("↔ Mov", "Move selected sensor");
  mkSenBtn("↻ Rot", "Rotate selected sensor");
  sensorLeftLayout->addSpacing(6);
  mkSenBtn("✕ Del", "Remove selected item");
  mkSenBtn("⊘ Clr", "Clear sensor scene");
  sensorLeftLayout->addStretch();
  mLeftStack->addWidget(sensorLeftPage);

  // ── Right stack: page 0 = Import legend, page 1 = Sensor Assembly panel ──
  mRightStack = new QStackedWidget(this);
  mRightStack->setFixedWidth(210);

  // Page 0: Import legend with orientation gizmo at top
  auto *importRightPage = new QWidget();
  importRightPage->setStyleSheet(
    "QWidget { background-color: #252525; border-left: 1px solid #333; }");
  auto *importRightLayout = new QVBoxLayout(importRightPage);
  importRightLayout->setContentsMargins(0, 0, 0, 0);
  importRightLayout->setSpacing(0);

  // Orientation gizmo - drawn with QPainter, safe from GL native surface
  mViewGizmo = new ViewGizmo(importRightPage);
  mViewGizmo->setFixedSize(110, 110);
  auto *gizmoRow = new QHBoxLayout();
  gizmoRow->setContentsMargins(0, 8, 0, 0);
  gizmoRow->addStretch();
  gizmoRow->addWidget(mViewGizmo);
  gizmoRow->addStretch();
  importRightLayout->addLayout(gizmoRow);

  auto *gizmoHint = new QLabel("click axis to snap", importRightPage);
  gizmoHint->setAlignment(Qt::AlignCenter);
  gizmoHint->setStyleSheet("color:#555; font-size:9px; padding:2px 0 6px 0;");
  importRightLayout->addWidget(gizmoHint);

  auto *divider = new QWidget(importRightPage);
  divider->setFixedHeight(1);
  divider->setStyleSheet("background:#333;");
  importRightLayout->addWidget(divider);

  mLegend = new QLabel(importRightPage);
  mLegend->setTextFormat(Qt::RichText);
  mLegend->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  mLegend->setWordWrap(false);
  mLegend->setStyleSheet(
    "QLabel { background-color: transparent; color: #C0C0C0;"
    "  padding: 8px 10px; font-size: 11px; }");
  mLegend->setText(
    "<b style='color:#EEE'>Legend</b><br>"
    "<span style='color:#666'>──────────</span><br>"
    "<span style='color:#F0AA66'>▪</span> orange - SUV 250×600<br>"
    "<span style='color:#50C8DC'>▪</span> cyan - sedan 250×500<br>"
    "<span style='color:#78DC82'>▪</span> green - sm-sedan 184×460<br>"
    "<span style='color:#AADCFF'>▪</span> blue - kart 60×170<br>"
    "<br><b style='color:#EEE'>Wheel anchors</b><br>"
    "<span style='color:#666'>──────────</span><br>"
    "<span style='color:#FFC800'>▪</span> colored tile - center<br>"
    "<span style='color:#FFFFC8'>▪</span> cream rod - fwd dir<br>"
    "<br><b style='color:#EEE'>Alignment</b><br>"
    "<span style='color:#666'>──────────</span><br>"
    "<span style='color:#FFFFFF'>▪</span> white - axle lines<br>"
    "<span style='color:#50D2FF'>▪</span> cyan - centerline<br>"
    "<span style='color:#C8C8C8'>▪</span> gray - wheelbase<br>"
    "<span style='color:#50FF78'>▪</span> green - body front<br>"
    "<br><b style='color:#EEE'>Axes</b><br>"
    "<span style='color:#666'>──────────</span><br>"
    "<span style='color:#DC3C3C'>▪</span> red - +X lateral<br>"
    "<span style='color:#3CC83C'>▪</span> green - +Y forward<br>"
    "<span style='color:#5082FF'>▪</span> blue - +Z up");
  importRightLayout->addWidget(mLegend, 1);
  mRightStack->addWidget(importRightPage);

  // Page 1: Sensor Assembly panel
  auto *sensorRightPage = new QWidget();
  sensorRightPage->setStyleSheet(
    "QWidget { background-color: #252525; border-left: 1px solid #333; }");
  auto *sensorLayout = new QVBoxLayout(sensorRightPage);
  sensorLayout->setContentsMargins(10, 10, 10, 10);
  sensorLayout->setSpacing(8);
  auto mkSectionHdr = [&](const QString &title) {
    auto *hdr = new QLabel(title, sensorRightPage);
    hdr->setStyleSheet("color:#EEE; font-weight:bold; font-size:11px;"
                       " border-bottom:1px solid #444; padding-bottom:3px;");
    sensorLayout->addWidget(hdr);
  };
  auto mkRow = [&](const QString &label, QWidget *ctrl) {
    auto *row = new QHBoxLayout();
    row->setSpacing(6);
    auto *lbl = new QLabel(label, sensorRightPage);
    lbl->setStyleSheet("color:#999; font-size:10px;");
    lbl->setFixedWidth(60);
    row->addWidget(lbl);
    row->addWidget(ctrl, 1);
    sensorLayout->addLayout(row);
  };
  const QString spinStyle = "QSpinBox,QDoubleSpinBox,QComboBox {"
    " background:#2A2A2A; color:#CCC; border:1px solid #444;"
    " border-radius:2px; padding:2px 4px; font-size:11px; }";
  sensorRightPage->setStyleSheet(sensorRightPage->styleSheet() + spinStyle);

  mkSectionHdr("Checkerboard");
  auto *cbRows = new QSpinBox(sensorRightPage);
  cbRows->setRange(2, 20); cbRows->setValue(9);
  mkRow("Rows", cbRows);
  auto *cbCols = new QSpinBox(sensorRightPage);
  cbCols->setRange(2, 20); cbCols->setValue(7);
  mkRow("Cols", cbCols);
  auto *cbSq = new QDoubleSpinBox(sensorRightPage);
  cbSq->setRange(1.0, 100.0); cbSq->setValue(10.0); cbSq->setSuffix(" cm");
  mkRow("Square", cbSq);
  auto *cbPlaceBtn = new QPushButton("Place in Scene", sensorRightPage);
  cbPlaceBtn->setStyleSheet(
    "QPushButton { background:#1A4A1A; color:#6DC86D; border:1px solid #3A7A3A;"
    "  border-radius:2px; padding:4px 8px; font-size:11px; }"
    "QPushButton:hover { background:#245024; }");
  sensorLayout->addWidget(cbPlaceBtn);

  sensorLayout->addSpacing(4);
  mkSectionHdr("Props");
  auto *propType = new QComboBox(sensorRightPage);
  propType->addItems({"Traffic Cone", "Barrier", "Box (50×50×50)", "Stop Sign"});
  mkRow("Type", propType);
  auto *propAddBtn = new QPushButton("Add Prop", sensorRightPage);
  propAddBtn->setStyleSheet(cbPlaceBtn->styleSheet());
  sensorLayout->addWidget(propAddBtn);

  sensorLayout->addSpacing(4);
  mkSectionHdr("Sensors");
  const QString sensorBtnStyle =
    "QPushButton { background:#1A1A3A; color:#8080DC; border:1px solid #334;"
    "  border-radius:2px; padding:3px 8px; font-size:10px; }"
    "QPushButton:hover { background:#252550; }";
  for (const QString &s : { "Add RGB Camera", "Add LiDAR", "Add RADAR" }) {
    auto *b = new QPushButton(s, sensorRightPage);
    b->setStyleSheet(sensorBtnStyle);
    sensorLayout->addWidget(b);
  }

  sensorLayout->addStretch();
  auto *sensorNote = new QLabel("Scene items appear in the\n3D viewport.", sensorRightPage);
  sensorNote->setStyleSheet("color:#555; font-size:10px; font-style:italic;");
  sensorNote->setAlignment(Qt::AlignCenter);
  sensorLayout->addWidget(sensorNote);
  mRightStack->addWidget(sensorRightPage);

  // ── Assemble main row ─────────────────────────────────────────────────────
  auto *mainRow = new QHBoxLayout();
  mainRow->setContentsMargins(0, 0, 0, 0);
  mainRow->setSpacing(0);
  mainRow->addWidget(mLeftStack,  0);
  mainRow->addWidget(mContainer,  1);
  mainRow->addWidget(mRightStack, 0);
  outer->addLayout(mainRow, 1);

  // Calibration badge (child of mContainer so it reflows with it)
  mCalibrationBadge = new QLabel(mContainer);
  mCalibrationBadge->setStyleSheet(
    "QLabel { background-color: rgba(10,10,10,210); color: #DDD;"
    "  padding: 5px 10px; border-radius: 3px; font-weight:700; font-size:12px; }");
  mCalibrationBadge->hide();

  // Connect gizmo snap to camera
  connect(mViewGizmo, &ViewGizmo::snapRequested,
          this, [this](const QVector3D &pos, const QVector3D &ctr, const QVector3D &up) {
    if (!mView) return;
    auto *cam = mView->camera();
    cam->setPosition(pos);
    cam->setViewCenter(ctr);
    cam->setUpVector(up);
  });
  if (mView) mViewGizmo->bindCamera(mView->camera());

  installEventFilter(this);

  // ── Bottom status bar ─────────────────────────────────────────────────────
  auto *statusBar = new QWidget(this);
  statusBar->setFixedHeight(22);
  statusBar->setStyleSheet("QWidget { background:#1A1A1A; border-top:1px solid #333; }");
  auto *statusRow = new QHBoxLayout(statusBar);
  statusRow->setContentsMargins(8, 0, 8, 0);
  mStatusLabel = new QLabel("Idle.", statusBar);
  mStatusLabel->setStyleSheet("color:#777; font-size:11px;");
  statusRow->addWidget(mStatusLabel, 1);
  outer->addWidget(statusBar);

  mInfoBox = new QPlainTextEdit();
  mInfoBox->setReadOnly(true);
  mInfoBox->setMaximumHeight(100);
  outer->addWidget(mInfoBox);

  connect(mBrowseBtn, &QPushButton::clicked, this, &VehiclePreviewPage::onBrowse);
  connect(mLoadBtn,   &QPushButton::clicked, this, &VehiclePreviewPage::onLoadClicked);
}

void VehiclePreviewPage::buildScene()
{
  mView = new Qt3DExtras::Qt3DWindow();
  mView->defaultFrameGraph()->setClearColor(QColor(20, 22, 28));

  mRoot = new Qt3DCore::QEntity();
  mView->setRootEntity(mRoot);

  auto *cam = mView->camera();
  cam->lens()->setPerspectiveProjection(45.0f, 16.0f / 9.0f, 1.0f, 50000.0f);
  cam->setUpVector(QVector3D(0, 0, 1));
  cam->setPosition(QVector3D(800, -800, 500));
  cam->setViewCenter(QVector3D(0, 0, 80));

  mCamCtl = new Qt3DExtras::QOrbitCameraController(mRoot);
  mCamCtl->setCamera(cam);
  mCamCtl->setLinearSpeed(2000.0f);
  mCamCtl->setLookSpeed(180.0f);

  auto *sun = new Qt3DRender::QDirectionalLight(mRoot);
  sun->setIntensity(0.8f);
  sun->setColor(QColor(255, 245, 230));
  sun->setWorldDirection(QVector3D(-0.5f, -0.5f, -0.7f).normalized());
  mRoot->addComponent(sun);

  auto *fill = new Qt3DRender::QPointLight(mRoot);
  fill->setIntensity(0.6f);
  fill->setColor(QColor(180, 200, 255));
  auto *fillEnt = new Qt3DCore::QEntity(mRoot);
  auto *fillT = new Qt3DCore::QTransform();
  fillT->setTranslation(QVector3D(-1000, 1000, 1500));
  fillEnt->addComponent(fill);
  fillEnt->addComponent(fillT);

  buildGridFloor(mRoot, 800.0f, 100.0f);
  buildAxisGizmo(mRoot, 200.0f);
  buildReferenceOutlines(mRoot);
  buildWheelMarkers(mRoot);
  buildAlignmentRods(mRoot);

  mMeshEntity    = new Qt3DCore::QEntity(mRoot);
  mMeshTransform = new Qt3DCore::QTransform();
  mMeshEntity->addComponent(mMeshTransform);

  // Qt3D's QSceneLoader (Assimp-backed) loads OBJ + MTL + textures into a
  // sub-scene parented to mMeshEntity, so the calibration view shows real
  // materials when the user passes --mtl (or when the OBJ has a `mtllib`
  // line that resolves next to it). Falls back to the wireframe path if
  // Assimp isn't available or the file doesn't load.
  mSceneLoader = new Qt3DRender::QSceneLoader(mMeshEntity);
  mMeshEntity->addComponent(mSceneLoader);
  connect(mSceneLoader, &Qt3DRender::QSceneLoader::statusChanged, this,
          [this](Qt3DRender::QSceneLoader::Status s) {
            onSceneStatusChanged(static_cast<int>(s));
          });
}

void VehiclePreviewPage::buildGridFloor(Qt3DCore::QEntity *root,
                                        float halfExtentCm, float cellCm)
{
  const int n = static_cast<int>(std::ceil(halfExtentCm / cellCm));
  for (int i = -n; i < n; ++i) {
    for (int j = -n; j < n; ++j) {
      const QColor c = ((i + j) & 1) ? QColor(225, 225, 230) : QColor(245, 245, 250);
      makeCheckerCell(root, i * cellCm, j * cellCm, cellCm, c);
    }
  }
}

void VehiclePreviewPage::buildAxisGizmo(Qt3DCore::QEntity *root, float lengthCm)
{
  makeAxis(root, QVector3D(lengthCm, 0, 0),  QColor(220,  60,  60));   // +X red
  makeAxis(root, QVector3D(0, lengthCm, 0),  QColor( 60, 200,  60));   // +Y green
  makeAxis(root, QVector3D(0, 0, lengthCm),  QColor( 80, 130, 255));   // +Z blue
  // Axis tip labels - placed just past each tip so they remain readable
  // from the top + 3D views.
  const float lift = 1.5f;
  const float labelH = 14.f;
  makeTextLabelOnGround(root, "+X (lat)",
                        QVector3D(lengthCm + 4.f, -labelH * 0.5f, lift),
                        labelH, QColor(220, 60, 60));
  makeTextLabelOnGround(root, "+Y (fwd)",
                        QVector3D(-labelH * 0.4f, lengthCm + 4.f, lift),
                        labelH, QColor(60, 200, 60));
  makeTextLabelOnGround(root, "+Z (up)",
                        QVector3D(4.f, -labelH * 0.5f, lengthCm + 1.f),
                        labelH, QColor(80, 130, 255));
}

void VehiclePreviewPage::buildReferenceOutlines(Qt3DCore::QEntity *root)
{
  const float zLift = 0.6f;
  struct Tpl { float halfW, halfL; QColor color; const char *name; };
  const Tpl tpls[] = {
    {  30.f,  85.f, QColor(170, 220, 255), "Kart 60×170"        },
    {  92.f, 230.f, QColor(120, 220, 130), "Small Sedan 184×460"},
    { 125.f, 250.f, QColor( 80, 200, 220), "Sedan 250×500"      },
    { 125.f, 300.f, QColor(240, 170, 100), "SUV 250×600"        },
  };
  for (const Tpl &t : tpls) {
    makeStencilRect(root, t.halfW, t.halfL, zLift, 1.5f, t.color);
    // Label sits just outside the rear-right corner (front of vehicle is +Y),
    // baseline +X so it reads naturally from the top view.
    makeTextLabelOnGround(root, QString::fromUtf8(t.name),
                          QVector3D(t.halfW + 5.f, -t.halfL - 12.f, zLift),
                          10.f, t.color);
  }
}

void VehiclePreviewPage::buildWheelMarkers(Qt3DCore::QEntity *root)
{
  const QColor colors[4] = { QColor(255,200,0), QColor(255,120,0),
                             QColor(0,200,255), QColor(0,120,255) };
  const QString labels[4] = { "FL","FR","RL","RR" };
  for (int i = 0; i < 4; ++i) {
    auto *e = new Qt3DCore::QEntity(root);
    auto *m = new Qt3DExtras::QPlaneMesh();
    m->setWidth(25.0f);    // X (lateral)
    m->setHeight(60.0f);   // forward-back (typical tire ~60 cm long footprint)
    auto *mat = new Qt3DExtras::QPhongMaterial();
    mat->setDiffuse(colors[i]);
    mat->setAmbient(colors[i]);
    auto *t = new Qt3DCore::QTransform();
    t->setRotationX(-90.0f);  // plane lies in XZ default; rotate to XY
    t->setTranslation(QVector3D(0, 0, -1000));
    e->addComponent(m);
    e->addComponent(mat);
    e->addComponent(t);
    mWheelMarkers[i] = e;
    mWheelXforms[i]  = t;
    e->setObjectName(labels[i]);

    // Forward-direction stripe: front half of footprint in bright cream.
    // The parent tile is Width=25 (X), Height=60 (local Z → world Y after
    // parent's rotateX(-90°)). A child at local translate(0, -0.5, 15):
    //   RotateX(-90°) * (0, -0.5, 15) = (0, +15, +0.5) in world-offset,
    // so the stripe lands at the forward 30 cm, 0.5 cm above the parent tile.
    auto *stripe = new Qt3DCore::QEntity(e);
    auto *sm = new Qt3DExtras::QPlaneMesh();
    sm->setWidth(25.0f);
    sm->setHeight(30.0f);
    auto *smat = new Qt3DExtras::QPhongMaterial();
    smat->setDiffuse(QColor(255, 255, 200));
    smat->setAmbient(QColor(255, 255, 200));
    auto *st = new Qt3DCore::QTransform();
    st->setTranslation(QVector3D(0.f, -0.5f, 15.f));
    stripe->addComponent(sm);
    stripe->addComponent(smat);
    stripe->addComponent(st);

    // FL/FR/RL/RR text label parented to the marker so it follows
    // setWheelPositionsCm. Sits just outboard of the marker's outer edge.
    const float dx = (i == 0 || i == 2) ? -22.f : +22.f;  // outboard
    makeTextLabelOnGround(e, labels[i],
                          QVector3D(dx, 0.f, 0.5f),
                          12.f, colors[i]);
  }
}

void VehiclePreviewPage::buildAlignmentRods(Qt3DCore::QEntity *root)
{
  // 0=front-axle  1=rear-axle  2=center-spine  3=left-wb  4=right-wb
  // 5=body-front  6=FL-fwd  7=FR-fwd  8=RL-fwd  9=RR-fwd
  const QColor colors[10] = {
    QColor(255, 255, 255),   // front axle
    QColor(255, 255, 255),   // rear axle
    QColor( 80, 210, 255),   // center spine
    QColor(200, 200, 200),   // left wheelbase
    QColor(200, 200, 200),   // right wheelbase
    QColor( 80, 255, 120),   // body front face
    QColor(255, 255, 160),   // FL forward arrow
    QColor(255, 255, 160),   // FR forward arrow
    QColor(255, 255, 160),   // RL forward arrow
    QColor(255, 255, 160),   // RR forward arrow
  };
  const float radii[10] = { 1.8f, 1.8f, 2.2f, 1.4f, 1.4f, 2.5f,
                             3.5f, 3.5f, 3.5f, 3.5f };
  for (int i = 0; i < 10; ++i) {
    auto *e = new Qt3DCore::QEntity(root);
    auto *m = new Qt3DExtras::QCylinderMesh();
    m->setRadius(radii[i]);
    m->setLength(1.0f);
    m->setRings(2);
    m->setSlices(8);
    auto *mat = new Qt3DExtras::QPhongMaterial();
    mat->setDiffuse(colors[i]);
    mat->setAmbient(colors[i]);
    auto *t = new Qt3DCore::QTransform();
    t->setTranslation(QVector3D(0, 0, -9999));
    e->addComponent(m);
    e->addComponent(mat);
    e->addComponent(t);
    mAlignRods[i] = { e, m, t };
  }
}

void VehiclePreviewPage::updateAlignmentLines()
{
  const QVector3D &fl = mLastWheelPos[0];
  const QVector3D &fr = mLastWheelPos[1];
  const QVector3D &rl = mLastWheelPos[2];
  const QVector3D &rr = mLastWheelPos[3];
  if (mAlignRods[0].mesh) updateAlignRod(mAlignRods[0].mesh, mAlignRods[0].xform, fl, fr);
  if (mAlignRods[1].mesh) updateAlignRod(mAlignRods[1].mesh, mAlignRods[1].xform, rl, rr);
  if (mAlignRods[2].mesh) {
    const QVector3D fMid = (fl + fr) * 0.5f;
    const QVector3D rMid = (rl + rr) * 0.5f;
    updateAlignRod(mAlignRods[2].mesh, mAlignRods[2].xform, fMid, rMid);
  }
  if (mAlignRods[3].mesh) updateAlignRod(mAlignRods[3].mesh, mAlignRods[3].xform, fl, rl);
  if (mAlignRods[4].mesh) updateAlignRod(mAlignRods[4].mesh, mAlignRods[4].xform, fr, rr);

  // Tire forward arrows: cream rods from each wheel extending 70 cm forward
  // in +Y (world forward). Drawn at ground level (Z=mLastWheelPos[i].z which
  // is already ground-projected from setWheelPositionsCm).
  const QVector3D fwdDelta(0, 70.f, 0);
  const QVector3D wheels[4] = { fl, fr, rl, rr };
  for (int i = 0; i < 4; ++i) {
    if (mAlignRods[6 + i].mesh)
      updateAlignRod(mAlignRods[6 + i].mesh, mAlignRods[6 + i].xform,
                     wheels[i], wheels[i] + fwdDelta);
  }
}

static bool installMeshGeometry(Qt3DCore::QEntity *meshEntity,
                                Qt3DCore::QTransform *meshTransform,
                                const carla_studio::vehicle_import::MeshGeometry &g)
{
  using namespace Qt3DCore;
  using namespace Qt3DRender;
  if (!meshEntity || !g.valid || g.vertexCount() == 0 || g.faceCount() == 0)
    return false;

  const auto comps = meshEntity->components();
  for (auto *c : comps) {
    if (c == meshTransform) continue;
    meshEntity->removeComponent(c);
    c->deleteLater();
  }

  auto *geom = new QGeometry(meshEntity);

  const int nV = g.vertexCount();
  QByteArray vBuf;
  vBuf.resize(nV * 3 * int(sizeof(float)));
  std::memcpy(vBuf.data(), g.verts.data(), vBuf.size());
  auto *vBuffer = new QBuffer(geom);
  vBuffer->setData(vBuf);
  auto *posAttr = new QAttribute(geom);
  posAttr->setName(QAttribute::defaultPositionAttributeName());
  posAttr->setVertexBaseType(QAttribute::Float);
  posAttr->setVertexSize(3);
  posAttr->setAttributeType(QAttribute::VertexAttribute);
  posAttr->setBuffer(vBuffer);
  posAttr->setByteStride(3 * sizeof(float));
  posAttr->setCount(nV);
  geom->addAttribute(posAttr);

  const int nF = g.faceCount();
  QByteArray iBuf;
  iBuf.resize(nF * 3 * int(sizeof(quint32)));
  auto *iPtr = reinterpret_cast<quint32 *>(iBuf.data());
  for (int i = 0; i < nF * 3; ++i) iPtr[i] = quint32(g.faces[i]);
  auto *iBuffer = new QBuffer(geom);
  iBuffer->setData(iBuf);
  auto *idxAttr = new QAttribute(geom);
  idxAttr->setVertexBaseType(QAttribute::UnsignedInt);
  idxAttr->setAttributeType(QAttribute::IndexAttribute);
  idxAttr->setBuffer(iBuffer);
  idxAttr->setCount(nF * 3);
  geom->addAttribute(idxAttr);

  auto *renderer = new QGeometryRenderer(meshEntity);
  renderer->setGeometry(geom);
  renderer->setPrimitiveType(QGeometryRenderer::Triangles);
  meshEntity->addComponent(renderer);

  auto *mat = new Qt3DExtras::QPhongMaterial(meshEntity);
  mat->setDiffuse(QColor(160, 175, 200));
  mat->setAmbient(QColor(60, 70, 90));
  mat->setSpecular(QColor(255, 255, 255));
  mat->setShininess(20.0f);
  meshEntity->addComponent(mat);

  if (meshTransform) {
    meshTransform->setScale(1.0f);
    meshTransform->setTranslation(QVector3D(0, 0, 0));
  }
  return true;
}

static bool objHasMtllib(const QString &path)
{
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
  QTextStream ts(&f);
  int n = 0;
  while (!ts.atEnd() && n < 400) {
    const QString line = ts.readLine();
    ++n;
    if (line.startsWith("mtllib ") || line.startsWith("mtllib\t")) {
      const int sp = line.indexOf(' ') < 0 ? line.indexOf('\t') : line.indexOf(' ');
      if (sp <= 0) break;
      const QString name = line.mid(sp + 1).trimmed();
      if (name.isEmpty()) break;
      const QString sib = QDir(QFileInfo(path).absolutePath()).absoluteFilePath(name);
      return QFileInfo(sib).isFile();
    }
    if (line.startsWith("v ") || line.startsWith("v\t")) break;
  }
  return false;
}

void VehiclePreviewPage::loadMesh(const QString &path)
{
  mPathEdit->setText(path);
  mStatusLabel->setText("Loading: " + path);
  mInfoBox->clear();
  mPendingMeshPath = path;

  MeshGeometry g = loadMeshGeometry(path);
  if (!g.valid) {
    mStatusLabel->setText("Load FAILED.");
    mInfoBox->setPlainText("loadMeshGeometry: returned invalid mesh for " + path);
    return;
  }
  // If the OBJ has a resolvable `mtllib FOO.mtl` line *and* QSceneLoader is
  // wired (assimp plugin present), defer entirely to QSceneLoader so the
  // body renders with real materials instead of the flat-shaded fallback.
  // Without this, both the manual QGeometryRenderer (single QPhongMaterial,
  // diffuse=#a0afd0) and the sub-entities created by QSceneLoader render
  // simultaneously - and the flat one wins, hiding the textured one.
  // Fallback path (no mtllib, or sceneloader errors) keeps the flat mesh.
  const bool useSceneLoaderOnly = mSceneLoader && objHasMtllib(path);
  if (!useSceneLoaderOnly) {
    if (!installMeshGeometry(mMeshEntity, mMeshTransform, g)) {
      mStatusLabel->setText("Load FAILED (no renderable geometry).");
      return;
    }
  } else {
    // Strip prior fallback components so they don't double-render with
    // QSceneLoader's sub-entities.
    const auto comps = mMeshEntity->components();
    for (auto *c : comps) {
      if (c == mMeshTransform) continue;
      if (c == mSceneLoader)   continue;
      mMeshEntity->removeComponent(c);
      c->deleteLater();
    }
  }
  if (mSceneLoader) {
    mSceneLoader->setSource(QUrl::fromLocalFile(QFileInfo(path).absoluteFilePath()));
  }
  mStatusLabel->setText(useSceneLoaderOnly
      ? "Loaded geometry - waiting on materials…"
      : "Loaded.");
  updateMeshBounds();
}

void VehiclePreviewPage::setWheelPositionsCm(const QVector3D &fl,
                                             const QVector3D &fr,
                                             const QVector3D &rl,
                                             const QVector3D &rr)
{
  // Spec Z is the wheel-centre height. Project to ground for the visual
  // tile and alignment lines so they don't float inside the car body.
  constexpr float kGroundZ = 1.5f;
  const QVector3D gnd[4] = {
    { fl.x(), fl.y(), kGroundZ },
    { fr.x(), fr.y(), kGroundZ },
    { rl.x(), rl.y(), kGroundZ },
    { rr.x(), rr.y(), kGroundZ },
  };
  for (int i = 0; i < 4; ++i)
    if (mWheelXforms[i]) mWheelXforms[i]->setTranslation(gnd[i]);
  mLastWheelPos[0] = gnd[0]; mLastWheelPos[1] = gnd[1];
  mLastWheelPos[2] = gnd[2]; mLastWheelPos[3] = gnd[3];
  updateAlignmentLines();
  mWheelPositionsExternal = true;
}

void VehiclePreviewPage::resetCamera()
{
  if (!mView) return;
  auto *cam = mView->camera();
  cam->setUpVector(QVector3D(0, 0, 1));
  cam->setPosition(QVector3D(800, -800, 500));
  cam->setViewCenter(QVector3D(0, 0, 80));
}

void VehiclePreviewPage::onBrowse()
{
  const QString path = QFileDialog::getOpenFileName(
      this, "Pick mesh",
      mPathEdit->text().isEmpty() ? QDir::homePath() : QFileInfo(mPathEdit->text()).absolutePath(),
      "Mesh files (*.obj *.fbx *.glb *.gltf *.dae);;All files (*)");
  if (!path.isEmpty()) mPathEdit->setText(path);
}

void VehiclePreviewPage::onLoadClicked()
{
  const QString p = mPathEdit->text().trimmed();
  if (p.isEmpty()) {
    mStatusLabel->setText("No path.");
    return;
  }
  loadMesh(p);
}

void VehiclePreviewPage::onSceneStatusChanged(int statusInt)
{
  using S = Qt3DRender::QSceneLoader::Status;
  const auto s = static_cast<S>(statusInt);
  switch (s) {
    case S::None:    mStatusLabel->setText("No mesh loaded."); break;
    case S::Loading: mStatusLabel->setText("Loading…"); break;
    case S::Ready:   mStatusLabel->setText("Loaded (with materials)."); updateMeshBounds(); break;
    case S::Error:
      mStatusLabel->setText("Materials unavailable - Qt3D scene importer "
                            "plugin missing. Install qt6-3d-assimpsceneimport-plugin "
                            "(Ubuntu 24) for OBJ+MTL rendering. Falling back to flat shading.");
      // QSceneLoader couldn't load the scene - re-install the manual flat
      // mesh so the calibration view at least shows geometry. Without this
      // the user sees an empty stage on assimp errors.
      if (!mPendingMeshPath.isEmpty()) {
        MeshGeometry g = loadMeshGeometry(mPendingMeshPath);
        if (g.valid) installMeshGeometry(mMeshEntity, mMeshTransform, g);
      }
      break;
  }
}

void VehiclePreviewPage::updateMeshBounds()
{
  const QString path = mPathEdit->text();
  MeshGeometry g = loadMeshGeometry(path);
  if (!g.valid) {
    mInfoBox->appendPlainText("MeshGeometry: load failed (Qt3D side may still render).");
    return;
  }

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

  float scaleHint = 1.0f;
  QString unitsHint = "raw";
  if (maxExt >= 200.f && maxExt <= 800.f)        { scaleHint = 1.0f;   unitsHint = "cm"; }
  else if (maxExt >= 2.f && maxExt <= 8.f)        { scaleHint = 100.0f; unitsHint = "m";  }
  else if (maxExt >= 2000.f && maxExt <= 8000.f)  { scaleHint = 0.1f;   unitsHint = "mm"; }

  const float ctrCm[3] = {
      0.5f * (minV[0] + maxV[0]) * scaleHint,
      0.5f * (minV[1] + maxV[1]) * scaleHint,
      0.5f * (minV[2] + maxV[2]) * scaleHint
  };
  const float minZcm = minV[2] * scaleHint;
  if (mMeshTransform) {
    mMeshTransform->setScale3D(QVector3D(scaleHint * mAdjustMirrorX,
                                         scaleHint * mAdjustMirrorY,
                                         scaleHint));
    mMeshTransform->setTranslation(QVector3D(-ctrCm[0], -ctrCm[1], -minZcm));
    QQuaternion q = QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), mAdjustYawDeg);
    mMeshTransform->setRotation(q);
  }

  const float lenX = (maxV[0]-minV[0]) * scaleHint;
  const float lenY = (maxV[1]-minV[1]) * scaleHint;
  const float lenZ = (maxV[2]-minV[2]) * scaleHint;
  if (mView && mView->camera()) {
    const float maxExtCm = std::max({lenX, lenY, lenZ});
    const float reach = std::max(250.0f, maxExtCm * 1.6f);
    auto *cam = mView->camera();
    cam->setUpVector(QVector3D(0, 0, 1));
    cam->setPosition(QVector3D(reach, -reach, reach * 0.6f));
    cam->setViewCenter(QVector3D(0, 0, maxExtCm * 0.25f));
  }

  const float halfX = lenX * 0.5f;
  const float halfY = lenY * 0.5f;
  const bool yIsForward = lenY >= lenX;
  const float fwdHalf = yIsForward ? halfY : halfX;
  const float latHalf = yIsForward ? halfX : halfY;
  const float groundZ = 0.8f;   // tire footprints sit just above the grid floor

  auto wheelAt = [&](float lateralSign, float fwdSign) {
    QVector3D p;
    if (yIsForward) {
      p = QVector3D(lateralSign * latHalf * 0.85f,
                    fwdSign     * fwdHalf * 0.75f,
                    groundZ);
    } else {
      p = QVector3D(fwdSign     * fwdHalf * 0.75f,
                    lateralSign * latHalf * 0.85f,
                    groundZ);
    }
    return p;
  };
  if (!mWheelPositionsExternal) {
    const QVector3D wfl = wheelAt(-1.f, +1.f);
    const QVector3D wfr = wheelAt(+1.f, +1.f);
    const QVector3D wrl = wheelAt(-1.f, -1.f);
    const QVector3D wrr = wheelAt(+1.f, -1.f);
    if (mWheelXforms[0]) mWheelXforms[0]->setTranslation(wfl);
    if (mWheelXforms[1]) mWheelXforms[1]->setTranslation(wfr);
    if (mWheelXforms[2]) mWheelXforms[2]->setTranslation(wrl);
    if (mWheelXforms[3]) mWheelXforms[3]->setTranslation(wrr);
    mLastWheelPos[0] = wfl; mLastWheelPos[1] = wfr;
    mLastWheelPos[2] = wrl; mLastWheelPos[3] = wrr;
    updateAlignmentLines();
  }

  // Body-front indicator rod: green line at the front face of the bbox,
  // sitting at ground level so it's visible from any camera angle.
  if (mAlignRods[5].mesh) {
    constexpr float kRodZ = 4.f;
    const float fwdY  = yIsForward ?  halfY :  halfX;
    const float latHf = yIsForward ?  halfX :  halfY;
    QVector3D left, right;
    if (yIsForward) {
      left  = QVector3D(-latHf, fwdY, kRodZ);
      right = QVector3D( latHf, fwdY, kRodZ);
    } else {
      left  = QVector3D( fwdY, -latHf, kRodZ);
      right = QVector3D( fwdY,  latHf, kRodZ);
    }
    updateAlignRod(mAlignRods[5].mesh, mAlignRods[5].xform, left, right);
  }

  if (mCalibrationBadge) {
    QString text;
    QString bg;
    const bool fwdYdominant = (lenY >= lenX) && (lenY >= lenZ);
    const bool zIsShortest  = (lenZ < lenX) && (lenZ < lenY);
    if (fwdYdominant && zIsShortest) {
      text = QStringLiteral("✓  No calibration required");
      bg   = "rgba(20, 110, 50, 220)";
    } else if (lenX > lenY && zIsShortest) {
      text = QStringLiteral("✗  Forward axis is X - try Rot +90 or Rot -90");
      bg   = "rgba(170, 30, 30, 220)";
    } else if (!zIsShortest) {
      text = QStringLiteral("✗  Up-axis looks wrong (Z not shortest) - try Rot +90");
      bg   = "rgba(170, 30, 30, 220)";
    } else {
      text = QStringLiteral("⚠  Geometry unusual - verify orientation manually");
      bg   = "rgba(170, 120, 30, 220)";
    }
    mCalibrationBadge->setStyleSheet(QString(
        "QLabel { background-color: %1; color: white; padding: 6px 12px; "
        "border-radius: 4px; font-weight: 700; font-size: 13px; }").arg(bg));
    mCalibrationBadge->setText(text);
    mCalibrationBadge->adjustSize();
    if (auto *parent = qobject_cast<QWidget*>(mCalibrationBadge->parent())) {
      const int margin = 12;
      mCalibrationBadge->move(
          parent->width()  - mCalibrationBadge->width()  - margin,
          parent->height() - mCalibrationBadge->height() - margin);
    }
    mCalibrationBadge->raise();
    mCalibrationBadge->show();
  }

  const float extCm[3] = { ext[0]*scaleHint, ext[1]*scaleHint, ext[2]*scaleHint };
  const float volM3 = (extCm[0] * extCm[1] * extCm[2]) / 1.0e6f;
  const char *cls =
      volM3 < 3.0f  ? "kart" :
      volM3 < 12.0f ? "sedan" :
      volM3 < 25.0f ? "suv" : "truck";

  int fwdAxis = (ext[0] >= ext[1] && ext[0] >= ext[2]) ? 0
              : (ext[1] >= ext[2]) ? 1 : 2;
  const float ctr[3] = { 0.5f*(minV[0]+maxV[0]), 0.5f*(minV[1]+maxV[1]), 0.5f*(minV[2]+maxV[2]) };
  long front = 0, back = 0;
  for (int i = 0; i < g.vertexCount(); ++i) {
    float x, y, z; g.vertex(i, x, y, z);
    const float v = (fwdAxis == 0) ? x : (fwdAxis == 1) ? y : z;
    if (v > ctr[fwdAxis]) ++front; else ++back;
  }
  const int fwdSign = front >= back ? +1 : -1;
  const char fwdName = (fwdAxis == 0) ? 'X' : (fwdAxis == 1) ? 'Y' : 'Z';

  QString info;
  info += QString("Verts: %1   Faces: %2   Malformed faces: %3\n")
              .arg(g.vertexCount()).arg(g.faceCount()).arg(g.malformedFaces);
  info += QString("Source units: %1  (display rescaled by ×%2)\n").arg(unitsHint).arg(scaleHint);
  info += QString("Extent (cm): %1 × %2 × %3   →  %4 × %5 × %6 m\n")
              .arg(extCm[0],0,'f',1).arg(extCm[1],0,'f',1).arg(extCm[2],0,'f',1)
              .arg(extCm[0]/100.0,0,'f',2).arg(extCm[1]/100.0,0,'f',2).arg(extCm[2]/100.0,0,'f',2);
  info += QString("Volume: %1 m^3   →  class=%2\n").arg(volM3,0,'f',2).arg(cls);
  info += QString("Forward axis: %1%2  (%3 verts ahead, %4 behind)\n")
              .arg(fwdSign>0?'+':'-').arg(fwdName).arg(front).arg(back);
  mInfoBox->setPlainText(info);
}


void VehiclePreviewPage::viewTop()
{
  if (!mView || !mView->camera()) return;
  auto *cam = mView->camera();
  cam->setUpVector(QVector3D(0, 1, 0));   // for top-down, use +Y as "up" on screen
  cam->setPosition(QVector3D(0, 0, 1500));
  cam->setViewCenter(QVector3D(0, 0, 0));
}

void VehiclePreviewPage::viewSide()
{
  if (!mView || !mView->camera()) return;
  auto *cam = mView->camera();
  cam->setUpVector(QVector3D(0, 0, 1));
  cam->setPosition(QVector3D(1500, 0, 100));
  cam->setViewCenter(QVector3D(0, 0, 80));
}

void VehiclePreviewPage::viewFront()
{
  if (!mView || !mView->camera()) return;
  auto *cam = mView->camera();
  cam->setUpVector(QVector3D(0, 0, 1));
  cam->setPosition(QVector3D(0, -1500, 100));
  cam->setViewCenter(QVector3D(0, 0, 80));
}

void VehiclePreviewPage::view3D()
{
  resetCamera();
}

void VehiclePreviewPage::fitCamera()
{
  if (!mPathEdit->text().isEmpty()) updateMeshBounds();
}


void VehiclePreviewPage::applyAdjustment(const QString &label)
{
  if (mMeshTransform) {
    QQuaternion q = QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), mAdjustYawDeg);
    mMeshTransform->setRotation(q);
    mMeshTransform->setScale3D(QVector3D(mAdjustMirrorX, mAdjustMirrorY, 1.0f));
  }
  if (mInfoBox) {
    mInfoBox->appendPlainText(QString("[adjust] %1   yaw=%2°  mirror=(%3,%4)")
                                .arg(label)
                                .arg(mAdjustYawDeg, 0, 'f', 0)
                                .arg(mAdjustMirrorX > 0 ? "+" : "-")
                                .arg(mAdjustMirrorY > 0 ? "+" : "-"));
  }
}

void VehiclePreviewPage::rotateMinus90() { mAdjustYawDeg -= 90.f; applyAdjustment("rotate -90"); }
void VehiclePreviewPage::rotatePlus90()  { mAdjustYawDeg += 90.f; applyAdjustment("rotate +90"); }
void VehiclePreviewPage::rotate180()     { mAdjustYawDeg += 180.f; applyAdjustment("flip 180"); }
void VehiclePreviewPage::mirrorX()       { mAdjustMirrorX *= -1.f; applyAdjustment("mirror X"); }
void VehiclePreviewPage::mirrorY()       { mAdjustMirrorY *= -1.f; applyAdjustment("mirror Y"); }

void VehiclePreviewPage::recenterMesh()
{
  if (!mPathEdit->text().isEmpty()) updateMeshBounds();
}

void VehiclePreviewPage::resetMeshTransform()
{
  mAdjustYawDeg  = 0.f;
  mAdjustMirrorX = 1.f;
  mAdjustMirrorY = 1.f;
  applyAdjustment("reset");
  if (!mPathEdit->text().isEmpty()) updateMeshBounds();
}

bool VehiclePreviewPage::eventFilter(QObject *obj, QEvent *ev)
{
  if (ev->type() == QEvent::Resize) {
    if (mCalibrationBadge && mCalibrationBadge->isVisible()) {
      if (auto *parent = qobject_cast<QWidget*>(mCalibrationBadge->parent())) {
        const int margin = 12;
        mCalibrationBadge->move(
            parent->width()  - mCalibrationBadge->width()  - margin,
            parent->height() - mCalibrationBadge->height() - margin);
      }
    }
  }
  return QWidget::eventFilter(obj, ev);
}

void VehiclePreviewPage::resizeEvent(QResizeEvent *ev)
{
  QWidget::resizeEvent(ev);
}

void VehiclePreviewPage::setToolMode(ToolMode mode)
{
  mToolMode = mode;
  const int idx = (mode == ToolMode::Import) ? 0 : 1;
  if (mLeftStack)  mLeftStack->setCurrentIndex(idx);
  if (mRightStack) mRightStack->setCurrentIndex(idx);
  if (mApplyBtn)   mApplyBtn->setVisible(mode == ToolMode::Import);
  if (mImportModeBtn) mImportModeBtn->setChecked(mode == ToolMode::Import);
  if (mSensorModeBtn) mSensorModeBtn->setChecked(mode == ToolMode::SensorAssembly);
}

void VehiclePreviewPage::applyToSpec()
{
  if (mInfoBox) {
    mInfoBox->appendPlainText(QString("[apply→spec] yaw=%1°  mirror=(%2,%3) - re-import requested")
                                .arg(mAdjustYawDeg, 0, 'f', 0)
                                .arg(mAdjustMirrorX > 0 ? "+" : "-")
                                .arg(mAdjustMirrorY > 0 ? "+" : "-"));
  }
  emit applyRequested(mAdjustYawDeg, mAdjustMirrorX, mAdjustMirrorY);
}

void VehiclePreviewPage::captureWindow()
{
  // Capture this entire calibration page (Qt3D viewport + legend + status
  // bar + info box) into a PNG, all in-process via QWidget::grab - no
  // scrot / xdotool. Output dir: $CWD/.cs_import/calib_shots/ if writable,
  // else /tmp/qt3d_shots/.
  QWidget *target = window() ? window() : this;
  QPixmap pix = target->grab();
  if (pix.isNull()) {
    if (mInfoBox) mInfoBox->appendPlainText("[capture] grab returned null pixmap.");
    return;
  }
  const QString cwdShots = QDir::currentPath() + "/.cs_import/calib_shots";
  const QString fallback = "/tmp/qt3d_shots";
  QString outDir = cwdShots;
  if (!QDir().mkpath(outDir)) outDir = fallback;
  if (!QDir().mkpath(outDir)) {
    if (mInfoBox) mInfoBox->appendPlainText("[capture] could not create output dir.");
    return;
  }
  const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
  const QString stem = QFileInfo(mPathEdit->text()).completeBaseName();
  const QString outPath = QString("%1/%2_%3.png").arg(outDir, stem.isEmpty() ? "calib" : stem, stamp);
  if (!pix.save(outPath, "PNG")) {
    if (mInfoBox) mInfoBox->appendPlainText("[capture] save failed: " + outPath);
    return;
  }
  QGuiApplication::clipboard()->setText(outPath);
  if (mInfoBox) mInfoBox->appendPlainText("[capture] saved → " + outPath + "  (path copied to clipboard)");
  if (mStatusLabel) mStatusLabel->setText("Captured → " + outPath);
}

}  // namespace carla_studio::vehicle_import

#endif  // CARLA_STUDIO_WITH_QT3D
