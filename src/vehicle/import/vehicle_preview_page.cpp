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
void VehiclePreviewPage::load_mesh(const QString&) {}
void VehiclePreviewPage::set_wheel_positions_cm(const QVector3D&, const QVector3D&,
                                             const QVector3D&, const QVector3D&) {}
void VehiclePreviewPage::resetCamera() {}
void VehiclePreviewPage::set_tool_mode(ToolMode) {}
void VehiclePreviewPage::on_browse() {}
void VehiclePreviewPage::on_load_clicked() {}
void VehiclePreviewPage::on_scene_status_changed(int) {}
void VehiclePreviewPage::build_scene() {}
void VehiclePreviewPage::build_grid_floor(Qt3DCore::QEntity*, float, float) {}
void VehiclePreviewPage::build_axis_gizmo(Qt3DCore::QEntity*, float) {}
void VehiclePreviewPage::build_reference_outlines(Qt3DCore::QEntity*) {}
void VehiclePreviewPage::build_wheel_markers(Qt3DCore::QEntity*) {}
void VehiclePreviewPage::build_alignment_rods(Qt3DCore::QEntity*) {}
void VehiclePreviewPage::update_alignment_lines() {}
void VehiclePreviewPage::update_mesh_bounds() {}
void VehiclePreviewPage::view_top() {}
void VehiclePreviewPage::view_side() {}
void VehiclePreviewPage::view_front() {}
void VehiclePreviewPage::view_3d() {}
void VehiclePreviewPage::fit_camera() {}
void VehiclePreviewPage::resizeEvent(QResizeEvent *ev) { QWidget::resizeEvent(ev); }
void VehiclePreviewPage::rotate_minus90() {}
void VehiclePreviewPage::rotate_plus90() {}
void VehiclePreviewPage::rotate180() {}
void VehiclePreviewPage::mirror_x() {}
void VehiclePreviewPage::mirror_y() {}
void VehiclePreviewPage::recenter_mesh() {}
void VehiclePreviewPage::reset_mesh_transform() {}
void VehiclePreviewPage::apply_adjustment(const QString&) {}
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
  makeRodXY(parent, fl, fr, radius, color);
  makeRodXY(parent, rl, rr, radius, color);
  makeRodXY(parent, fl, rl, radius, color);
  makeRodXY(parent, fr, rr, radius, color);
}

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

  const float scale = heightCm / 14.f;
  t->setScale(scale);

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
                                   float xCm, float yCm, float cell_cm,
                                   const QColor &color)
{
  auto *e = new Qt3DCore::QEntity(parent);
  auto *m = new Qt3DExtras::QPlaneMesh();
  m->setWidth(cell_cm);
  m->setHeight(cell_cm);
  auto *mat = new Qt3DExtras::QPhongMaterial();
  mat->setDiffuse(color);
  mat->setAmbient(color);
  mat->setSpecular(QColor(40, 40, 40));
  mat->setShininess(2.0f);
  auto *t = new Qt3DCore::QTransform();
  t->setRotationX(-90.0f);
  t->setTranslation(QVector3D(xCm + cell_cm * 0.5f, yCm + cell_cm * 0.5f, 0.0f));
  e->addComponent(m);
  e->addComponent(mat);
  e->addComponent(t);
  return e;
}

}

VehiclePreviewPage::VehiclePreviewPage(QWidget *parent) : QWidget(parent)
{

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
  m_path_edit = new QLineEdit(topBar);
  m_path_edit->setPlaceholderText("vehicle.obj / .fbx");
  m_path_edit->setStyleSheet("QLineEdit { background:#2A2A2A; color:#CCC;"
                           "  border:1px solid #444; padding:2px 6px; }");
  topRow->addWidget(m_path_edit, 1);
  m_browse_btn = new QPushButton("Browse…", topBar);
  m_load_btn   = new QPushButton("Load",    topBar);
  m_browse_btn->setFixedHeight(24);
  m_load_btn->setFixedHeight(24);
  topRow->addWidget(m_browse_btn);
  topRow->addWidget(m_load_btn);
  outer->addWidget(topBar);

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
  mkViewBtn("Top",   "Top-down view",       &VehiclePreviewPage::view_top);
  mkViewBtn("Side",  "Side view",           &VehiclePreviewPage::view_side);
  mkViewBtn("Front", "Front view",          &VehiclePreviewPage::view_front);
  mkViewBtn("3D",    "Perspective / orbit", &VehiclePreviewPage::view_3d);
  mkViewBtn("Fit",   "Fit camera to mesh",  &VehiclePreviewPage::fit_camera);
  m_reset_cam_btn = new QPushButton("⟳ Reset", viewBar);
  m_reset_cam_btn->setToolTip("Reset camera");
  m_reset_cam_btn->setFixedHeight(22);
  m_reset_cam_btn->setStyleSheet("QPushButton { min-width:48px; font-size:11px; }");
  connect(m_reset_cam_btn, &QPushButton::clicked, this, &VehiclePreviewPage::resetCamera);
  viewRow2->addWidget(m_reset_cam_btn);

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
  connect(captureBtn, &QPushButton::clicked, this, &VehiclePreviewPage::capture_window);
  viewRow2->addWidget(captureBtn);

  viewRow2->addStretch();

  const QString modeBase =
    "QPushButton { font-size:10px; font-weight:600; padding:0 10px; height:22px;"
    "  border:1px solid #555; background:#2E2E2E; color:#999; }"
    "QPushButton:checked { background:#0078D4; color:#FFF; border-color:#0078D4; }"
    "QPushButton:hover:!checked { background:#444; }";
  m_import_mode_btn = new QPushButton("Import", viewBar);
  m_import_mode_btn->setCheckable(true);
  m_import_mode_btn->setChecked(true);
  m_import_mode_btn->setFixedHeight(22);
  m_import_mode_btn->setStyleSheet(modeBase + "QPushButton { border-radius:2px 0 0 2px; }");
  m_sensor_mode_btn = new QPushButton("Sensor Assembly", viewBar);
  m_sensor_mode_btn->setCheckable(true);
  m_sensor_mode_btn->setChecked(false);
  m_sensor_mode_btn->setFixedHeight(22);
  m_sensor_mode_btn->setStyleSheet(modeBase + "QPushButton { border-radius:0 2px 2px 0;"
                                           "  border-left:none; }");
  connect(m_import_mode_btn, &QPushButton::clicked, this, [this]() {
    m_import_mode_btn->setChecked(true);
    m_sensor_mode_btn->setChecked(false);
    set_tool_mode(ToolMode::Import);
  });
  connect(m_sensor_mode_btn, &QPushButton::clicked, this, [this]() {
    m_sensor_mode_btn->setChecked(true);
    m_import_mode_btn->setChecked(false);
    set_tool_mode(ToolMode::SensorAssembly);
  });
  viewRow2->addWidget(m_import_mode_btn);
  viewRow2->addWidget(m_sensor_mode_btn);
  viewRow2->addSpacing(10);
  addSep();
  viewRow2->addSpacing(6);

  m_apply_btn = new QPushButton("▶  Apply & Re-import", viewBar);
  m_apply_btn->setToolTip("Send adjustments to importer and re-cook");
  m_apply_btn->setFixedHeight(22);
  m_apply_btn->setStyleSheet(
    "QPushButton { background:#0078D4; color:#FFF; font-weight:600;"
    "  border:none; border-radius:2px; padding:0 12px; font-size:11px; }"
    "QPushButton:hover { background:#0090F0; }"
    "QPushButton:pressed { background:#005FAD; }");
  connect(m_apply_btn, &QPushButton::clicked, this, &VehiclePreviewPage::apply_to_spec);
  viewRow2->addWidget(m_apply_btn);
  outer->addWidget(viewBar);

  build_scene();
  m_container = QWidget::createWindowContainer(m_view, this);
  m_container->setMinimumSize(480, 380);
  m_container->setFocusPolicy(Qt::StrongFocus);

  m_left_stack = new QStackedWidget(this);
  m_left_stack->setFixedWidth(54);

  auto *importLeftPage = new QWidget();
  importLeftPage->setStyleSheet("QWidget { background-color: #252525;"
                                "  border-right: 1px solid #333; }");
  auto *importLeftLayout = new QVBoxLayout(importLeftPage);
  importLeftLayout->setContentsMargins(4, 8, 4, 8);
  importLeftLayout->setSpacing(3);
  auto mkToolBtn = [&](QWidget *par, QVBoxLayout *layout,
                        const QString &lbl, const QString &tip,
                        void (VehiclePreviewPage::*slot)()) {
    auto *b = new QPushButton(lbl, par);
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
            "↺ -90",  "Rotate -90°",          &VehiclePreviewPage::rotate_minus90);
  mkToolBtn(importLeftPage, importLeftLayout,
            "↻ +90",  "Rotate +90°",          &VehiclePreviewPage::rotate_plus90);
  mkToolBtn(importLeftPage, importLeftLayout,
            "↕ 180",  "Flip 180°",            &VehiclePreviewPage::rotate180);
  importLeftLayout->addSpacing(6);
  mkToolBtn(importLeftPage, importLeftLayout,
            "⟺ X",   "Mirror across X axis",  &VehiclePreviewPage::mirror_x);
  mkToolBtn(importLeftPage, importLeftLayout,
            "⟺ Y",   "Mirror across Y axis",  &VehiclePreviewPage::mirror_y);
  importLeftLayout->addSpacing(6);
  mkToolBtn(importLeftPage, importLeftLayout,
            "⊕ Ctr", "Recenter mesh",         &VehiclePreviewPage::recenter_mesh);
  mkToolBtn(importLeftPage, importLeftLayout,
            "✕ Rst", "Reset all adjustments", &VehiclePreviewPage::reset_mesh_transform);
  importLeftLayout->addStretch();
  m_left_stack->addWidget(importLeftPage);

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
  m_left_stack->addWidget(sensorLeftPage);

  m_right_stack = new QStackedWidget(this);
  m_right_stack->setFixedWidth(210);

  auto *importRightPage = new QWidget();
  importRightPage->setStyleSheet(
    "QWidget { background-color: #252525; border-left: 1px solid #333; }");
  auto *importRightLayout = new QVBoxLayout(importRightPage);
  importRightLayout->setContentsMargins(0, 0, 0, 0);
  importRightLayout->setSpacing(0);

  m_view_gizmo = new ViewGizmo(importRightPage);
  m_view_gizmo->setFixedSize(110, 110);
  auto *gizmoRow = new QHBoxLayout();
  gizmoRow->setContentsMargins(0, 8, 0, 0);
  gizmoRow->addStretch();
  gizmoRow->addWidget(m_view_gizmo);
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

  m_legend = new QLabel(importRightPage);
  m_legend->setTextFormat(Qt::RichText);
  m_legend->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  m_legend->setWordWrap(false);
  m_legend->setStyleSheet(
    "QLabel { background-color: transparent; color: #C0C0C0;"
    "  padding: 8px 10px; font-size: 11px; }");
  m_legend->setText(
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
  importRightLayout->addWidget(m_legend, 1);
  m_right_stack->addWidget(importRightPage);

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
  for (const char *s : { "Add RGB Camera", "Add LiDAR", "Add RADAR" }) {
    auto *b = new QPushButton(s, sensorRightPage);
    b->setStyleSheet(sensorBtnStyle);
    sensorLayout->addWidget(b);
  }

  sensorLayout->addStretch();
  auto *sensorNote = new QLabel("Scene items appear in the\n3D viewport.", sensorRightPage);
  sensorNote->setStyleSheet("color:#555; font-size:10px; font-style:italic;");
  sensorNote->setAlignment(Qt::AlignCenter);
  sensorLayout->addWidget(sensorNote);
  m_right_stack->addWidget(sensorRightPage);

  auto *mainRow = new QHBoxLayout();
  mainRow->setContentsMargins(0, 0, 0, 0);
  mainRow->setSpacing(0);
  mainRow->addWidget(m_left_stack,  0);
  mainRow->addWidget(m_container,  1);
  mainRow->addWidget(m_right_stack, 0);
  outer->addLayout(mainRow, 1);

  m_calibration_badge = new QLabel(m_container);
  m_calibration_badge->setStyleSheet(
    "QLabel { background-color: rgba(10,10,10,210); color: #DDD;"
    "  padding: 5px 10px; border-radius: 3px; font-weight:700; font-size:12px; }");
  m_calibration_badge->hide();

  connect(m_view_gizmo, &ViewGizmo::snap_requested,
          this, [this](const QVector3D &pos, const QVector3D &ctr, const QVector3D &up) {
    if (!m_view) return;
    auto *cam = m_view->camera();
    cam->setPosition(pos);
    cam->setViewCenter(ctr);
    cam->setUpVector(up);
  });
  if (m_view) m_view_gizmo->bind_camera(m_view->camera());

  installEventFilter(this);

  auto *statusBar = new QWidget(this);
  statusBar->setFixedHeight(22);
  statusBar->setStyleSheet("QWidget { background:#1A1A1A; border-top:1px solid #333; }");
  auto *statusRow = new QHBoxLayout(statusBar);
  statusRow->setContentsMargins(8, 0, 8, 0);
  m_status_label = new QLabel("Idle.", statusBar);
  m_status_label->setStyleSheet("color:#777; font-size:11px;");
  statusRow->addWidget(m_status_label, 1);
  outer->addWidget(statusBar);

  m_info_box = new QPlainTextEdit();
  m_info_box->setReadOnly(true);
  m_info_box->setMaximumHeight(100);
  outer->addWidget(m_info_box);

  connect(m_browse_btn, &QPushButton::clicked, this, &VehiclePreviewPage::on_browse);
  connect(m_load_btn,   &QPushButton::clicked, this, &VehiclePreviewPage::on_load_clicked);
}

void VehiclePreviewPage::build_scene()
{
  m_view = new Qt3DExtras::Qt3DWindow();
  m_view->defaultFrameGraph()->setClearColor(QColor(20, 22, 28));

  m_root = new Qt3DCore::QEntity();
  m_view->setRootEntity(m_root);

  auto *cam = m_view->camera();
  cam->lens()->setPerspectiveProjection(45.0f, 16.0f / 9.0f, 1.0f, 50000.0f);
  cam->setUpVector(QVector3D(0, 0, 1));
  cam->setPosition(QVector3D(800, -800, 500));
  cam->setViewCenter(QVector3D(0, 0, 80));

  m_cam_ctl = new Qt3DExtras::QOrbitCameraController(m_root);
  m_cam_ctl->setCamera(cam);
  m_cam_ctl->setLinearSpeed(2000.0f);
  m_cam_ctl->setLookSpeed(180.0f);

  auto *sun = new Qt3DRender::QDirectionalLight(m_root);
  sun->setIntensity(0.8f);
  sun->setColor(QColor(255, 245, 230));
  sun->setWorldDirection(QVector3D(-0.5f, -0.5f, -0.7f).normalized());
  m_root->addComponent(sun);

  auto *fill = new Qt3DRender::QPointLight(m_root);
  fill->setIntensity(0.6f);
  fill->setColor(QColor(180, 200, 255));
  auto *fillEnt = new Qt3DCore::QEntity(m_root);
  auto *fillT = new Qt3DCore::QTransform();
  fillT->setTranslation(QVector3D(-1000, 1000, 1500));
  fillEnt->addComponent(fill);
  fillEnt->addComponent(fillT);

  build_grid_floor(m_root, 800.0f, 100.0f);
  build_axis_gizmo(m_root, 200.0f);
  build_reference_outlines(m_root);
  build_wheel_markers(m_root);
  build_alignment_rods(m_root);

  m_mesh_entity    = new Qt3DCore::QEntity(m_root);
  m_mesh_transform = new Qt3DCore::QTransform();
  m_mesh_entity->addComponent(m_mesh_transform);

  m_scene_loader = new Qt3DRender::QSceneLoader(m_mesh_entity);
  m_mesh_entity->addComponent(m_scene_loader);
  connect(m_scene_loader, &Qt3DRender::QSceneLoader::statusChanged, this,
          [this](Qt3DRender::QSceneLoader::Status s) {
            on_scene_status_changed(static_cast<int>(s));
          });
}

void VehiclePreviewPage::build_grid_floor(Qt3DCore::QEntity *root,
                                        float half_extent_cm, float cell_cm)
{
  const int n = static_cast<int>(std::ceil(half_extent_cm / cell_cm));
  for (int i = -n; i < n; ++i) {
    for (int j = -n; j < n; ++j) {
      const QColor c = ((i + j) & 1) ? QColor(225, 225, 230) : QColor(245, 245, 250);
      makeCheckerCell(root, static_cast<float>(i) * cell_cm, static_cast<float>(j) * cell_cm, cell_cm, c);
    }
  }
}

void VehiclePreviewPage::build_axis_gizmo(Qt3DCore::QEntity *root, float length_cm)
{
  makeAxis(root, QVector3D(length_cm, 0, 0),  QColor(220,  60,  60));
  makeAxis(root, QVector3D(0, length_cm, 0),  QColor( 60, 200,  60));
  makeAxis(root, QVector3D(0, 0, length_cm),  QColor( 80, 130, 255));

  const float lift = 1.5f;
  const float labelH = 14.f;
  makeTextLabelOnGround(root, "+X (lat)",
                        QVector3D(length_cm + 4.f, -labelH * 0.5f, lift),
                        labelH, QColor(220, 60, 60));
  makeTextLabelOnGround(root, "+Y (fwd)",
                        QVector3D(-labelH * 0.4f, length_cm + 4.f, lift),
                        labelH, QColor(60, 200, 60));
  makeTextLabelOnGround(root, "+Z (up)",
                        QVector3D(4.f, -labelH * 0.5f, length_cm + 1.f),
                        labelH, QColor(80, 130, 255));
}

void VehiclePreviewPage::build_reference_outlines(Qt3DCore::QEntity *root)
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

    makeTextLabelOnGround(root, QString::fromUtf8(t.name),
                          QVector3D(t.halfW + 5.f, -t.halfL - 12.f, zLift),
                          10.f, t.color);
  }
}

void VehiclePreviewPage::build_wheel_markers(Qt3DCore::QEntity *root)
{
  const QColor colors[4] = { QColor(255,200,0), QColor(255,120,0),
                             QColor(0,200,255), QColor(0,120,255) };
  const QString labels[4] = { "FL","FR","RL","RR" };
  for (size_t i = 0; i < 4; ++i) {
    auto *e = new Qt3DCore::QEntity(root);
    auto *m = new Qt3DExtras::QPlaneMesh();
    m->setWidth(25.0f);
    m->setHeight(60.0f);
    auto *mat = new Qt3DExtras::QPhongMaterial();
    mat->setDiffuse(colors[i]);
    mat->setAmbient(colors[i]);
    auto *t = new Qt3DCore::QTransform();
    t->setRotationX(-90.0f);
    t->setTranslation(QVector3D(0, 0, -1000));
    e->addComponent(m);
    e->addComponent(mat);
    e->addComponent(t);
    m_wheel_markers[i] = e;
    m_wheel_xforms[i]  = t;
    e->setObjectName(labels[i]);

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

    const float dx = (i == 0 || i == 2) ? -22.f : +22.f;
    makeTextLabelOnGround(e, labels[i],
                          QVector3D(dx, 0.f, 0.5f),
                          12.f, colors[i]);
  }
}

void VehiclePreviewPage::build_alignment_rods(Qt3DCore::QEntity *root)
{

  const QColor colors[10] = {
    QColor(255, 255, 255),
    QColor(255, 255, 255),
    QColor( 80, 210, 255),
    QColor(200, 200, 200),
    QColor(200, 200, 200),
    QColor( 80, 255, 120),
    QColor(255, 255, 160),
    QColor(255, 255, 160),
    QColor(255, 255, 160),
    QColor(255, 255, 160),
  };
  const float radii[10] = { 1.8f, 1.8f, 2.2f, 1.4f, 1.4f, 2.5f,
                             3.5f, 3.5f, 3.5f, 3.5f };
  for (size_t i = 0; i < 10; ++i) {
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
    m_align_rods[i] = { e, m, t };
  }
}

void VehiclePreviewPage::update_alignment_lines()
{
  const QVector3D &fl = m_last_wheel_pos[0];
  const QVector3D &fr = m_last_wheel_pos[1];
  const QVector3D &rl = m_last_wheel_pos[2];
  const QVector3D &rr = m_last_wheel_pos[3];
  if (m_align_rods[0].mesh) updateAlignRod(m_align_rods[0].mesh, m_align_rods[0].xform, fl, fr);
  if (m_align_rods[1].mesh) updateAlignRod(m_align_rods[1].mesh, m_align_rods[1].xform, rl, rr);
  if (m_align_rods[2].mesh) {
    const QVector3D fMid = (fl + fr) * 0.5f;
    const QVector3D rMid = (rl + rr) * 0.5f;
    updateAlignRod(m_align_rods[2].mesh, m_align_rods[2].xform, fMid, rMid);
  }
  if (m_align_rods[3].mesh) updateAlignRod(m_align_rods[3].mesh, m_align_rods[3].xform, fl, rl);
  if (m_align_rods[4].mesh) updateAlignRod(m_align_rods[4].mesh, m_align_rods[4].xform, fr, rr);

  const QVector3D fwdDelta(0, 70.f, 0);
  const QVector3D wheels[4] = { fl, fr, rl, rr };
  for (size_t i = 0; i < 4; ++i) {
    if (m_align_rods[6 + i].mesh)
      updateAlignRod(m_align_rods[6 + i].mesh, m_align_rods[6 + i].xform,
                     wheels[i], wheels[i] + fwdDelta);
  }
}

static bool installMeshGeometry(Qt3DCore::QEntity *meshEntity,
                                Qt3DCore::QTransform *meshTransform,
                                const carla_studio::vehicle_import::MeshGeometry &g)
{
  using namespace Qt3DCore;
  using namespace Qt3DRender;
  if (!meshEntity || !g.valid || g.vertex_count() == 0 || g.face_count() == 0)
    return false;

  const auto comps = meshEntity->components();
  for (auto *c : comps) {
    if (c == meshTransform) continue;
    meshEntity->removeComponent(c);
    c->deleteLater();
  }

  auto *geom = new QGeometry(meshEntity);

  const int nV = g.vertex_count();
  QByteArray vBuf;
  vBuf.resize(nV * 3 * static_cast<int>(sizeof(float)));
  std::memcpy(vBuf.data(), g.verts.data(), static_cast<size_t>(vBuf.size()));
  auto *vBuffer = new QBuffer(geom);
  vBuffer->setData(vBuf);
  auto *posAttr = new QAttribute(geom);
  posAttr->setName(QAttribute::defaultPositionAttributeName());
  posAttr->setVertexBaseType(QAttribute::Float);
  posAttr->setVertexSize(3);
  posAttr->setAttributeType(QAttribute::VertexAttribute);
  posAttr->setBuffer(vBuffer);
  posAttr->setByteStride(3 * sizeof(float));
  posAttr->setCount(static_cast<uint>(nV));
  geom->addAttribute(posAttr);

  const int nF = g.face_count();
  QByteArray iBuf;
  iBuf.resize(nF * 3 * static_cast<int>(sizeof(quint32)));
  auto *iPtr = reinterpret_cast<quint32 *>(iBuf.data());
  for (int i = 0; i < nF * 3; ++i) iPtr[i] = static_cast<quint32>(g.faces[static_cast<size_t>(i)]);
  auto *iBuffer = new QBuffer(geom);
  iBuffer->setData(iBuf);
  auto *idxAttr = new QAttribute(geom);
  idxAttr->setVertexBaseType(QAttribute::UnsignedInt);
  idxAttr->setAttributeType(QAttribute::IndexAttribute);
  idxAttr->setBuffer(iBuffer);
  idxAttr->setCount(static_cast<uint>(nF * 3));
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
      const qsizetype sp = line.indexOf(' ') < 0 ? line.indexOf('\t') : line.indexOf(' ');
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

void VehiclePreviewPage::load_mesh(const QString &path)
{
  m_path_edit->setText(path);
  m_status_label->setText("Loading: " + path);
  m_info_box->clear();
  m_pending_mesh_path = path;

  MeshGeometry g = load_mesh_geometry(path);
  if (!g.valid) {
    m_status_label->setText("Load FAILED.");
    m_info_box->setPlainText("load_mesh_geometry: returned invalid mesh for " + path);
    return;
  }

  const bool useSceneLoaderOnly = m_scene_loader && objHasMtllib(path);
  if (!useSceneLoaderOnly) {
    if (!installMeshGeometry(m_mesh_entity, m_mesh_transform, g)) {
      m_status_label->setText("Load FAILED (no renderable geometry).");
      return;
    }
  } else {

    const auto comps = m_mesh_entity->components();
    for (auto *c : comps) {
      if (c == m_mesh_transform) continue;
      if (c == m_scene_loader)   continue;
      m_mesh_entity->removeComponent(c);
      c->deleteLater();
    }
  }
  if (m_scene_loader) {
    m_scene_loader->setSource(QUrl::fromLocalFile(QFileInfo(path).absoluteFilePath()));
  }
  m_status_label->setText(useSceneLoaderOnly
      ? "Loaded geometry - waiting on materials…"
      : "Loaded.");
  update_mesh_bounds();
}

void VehiclePreviewPage::set_wheel_positions_cm(const QVector3D &fl,
                                             const QVector3D &fr,
                                             const QVector3D &rl,
                                             const QVector3D &rr)
{

  constexpr float kGroundZ = 1.5f;
  const QVector3D gnd[4] = {
    { fl.x(), fl.y(), kGroundZ },
    { fr.x(), fr.y(), kGroundZ },
    { rl.x(), rl.y(), kGroundZ },
    { rr.x(), rr.y(), kGroundZ },
  };
  for (size_t i = 0; i < 4; ++i)
    if (m_wheel_xforms[i]) m_wheel_xforms[i]->setTranslation(gnd[i]);
  m_last_wheel_pos[0] = gnd[0]; m_last_wheel_pos[1] = gnd[1];
  m_last_wheel_pos[2] = gnd[2]; m_last_wheel_pos[3] = gnd[3];
  update_alignment_lines();
  m_wheel_positions_external = true;
}

void VehiclePreviewPage::resetCamera()
{
  if (!m_view) return;
  auto *cam = m_view->camera();
  cam->setUpVector(QVector3D(0, 0, 1));
  cam->setPosition(QVector3D(800, -800, 500));
  cam->setViewCenter(QVector3D(0, 0, 80));
}

void VehiclePreviewPage::on_browse()
{
  const QString path = QFileDialog::getOpenFileName(
      this, "Pick mesh",
      m_path_edit->text().isEmpty() ? QDir::homePath() : QFileInfo(m_path_edit->text()).absolutePath(),
      "Mesh files (*.obj *.fbx *.glb *.gltf *.dae);;All files (*)");
  if (!path.isEmpty()) m_path_edit->setText(path);
}

void VehiclePreviewPage::on_load_clicked()
{
  const QString p = m_path_edit->text().trimmed();
  if (p.isEmpty()) {
    m_status_label->setText("No path.");
    return;
  }
  load_mesh(p);
}

void VehiclePreviewPage::on_scene_status_changed(int statusInt)
{
  using S = Qt3DRender::QSceneLoader::Status;
  const auto s = static_cast<S>(statusInt);
  switch (s) {
    case S::None:    m_status_label->setText("No mesh loaded."); break;
    case S::Loading: m_status_label->setText("Loading…"); break;
    case S::Ready:   m_status_label->setText("Loaded (with materials)."); update_mesh_bounds(); break;
    case S::Error:
      m_status_label->setText("Materials unavailable - Qt3D scene importer "
                            "plugin missing. Install qt6-3d-assimpsceneimport-plugin "
                            "(Ubuntu 24) for OBJ+MTL rendering. Falling back to flat shading.");

      if (!m_pending_mesh_path.isEmpty()) {
        MeshGeometry g = load_mesh_geometry(m_pending_mesh_path);
        if (g.valid) installMeshGeometry(m_mesh_entity, m_mesh_transform, g);
      }
      break;
  }
}

void VehiclePreviewPage::update_mesh_bounds()
{
  const QString path = m_path_edit->text();
  MeshGeometry g = load_mesh_geometry(path);
  if (!g.valid) {
    m_info_box->appendPlainText("MeshGeometry: load failed (Qt3D side may still render).");
    return;
  }

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

  float scaleHint = 1.0f;
  QString units_hint = "raw";
  if (maxExt >= 200.f && maxExt <= 800.f)        { scaleHint = 1.0f;   units_hint = "cm"; }
  else if (maxExt >= 2.f && maxExt <= 8.f)        { scaleHint = 100.0f; units_hint = "m";  }
  else if (maxExt >= 2000.f && maxExt <= 8000.f)  { scaleHint = 0.1f;   units_hint = "mm"; }

  const float ctrCm[3] = {
      0.5f * (minV[0] + maxV[0]) * scaleHint,
      0.5f * (minV[1] + maxV[1]) * scaleHint,
      0.5f * (minV[2] + maxV[2]) * scaleHint
  };
  const float minZcm = minV[2] * scaleHint;
  if (m_mesh_transform) {
    m_mesh_transform->setScale3D(QVector3D(scaleHint * m_adjust_mirror_x,
                                         scaleHint * m_adjust_mirror_y,
                                         scaleHint));
    m_mesh_transform->setTranslation(QVector3D(-ctrCm[0], -ctrCm[1], -minZcm));
    QQuaternion q = QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), m_adjust_yaw_deg);
    m_mesh_transform->setRotation(q);
  }

  const float lenX = (maxV[0]-minV[0]) * scaleHint;
  const float lenY = (maxV[1]-minV[1]) * scaleHint;
  const float lenZ = (maxV[2]-minV[2]) * scaleHint;
  if (m_view && m_view->camera()) {
    const float maxExtCm = std::max({lenX, lenY, lenZ});
    const float reach = std::max(250.0f, maxExtCm * 1.6f);
    auto *cam = m_view->camera();
    cam->setUpVector(QVector3D(0, 0, 1));
    cam->setPosition(QVector3D(reach, -reach, reach * 0.6f));
    cam->setViewCenter(QVector3D(0, 0, maxExtCm * 0.25f));
  }

  const float halfX = lenX * 0.5f;
  const float halfY = lenY * 0.5f;
  const bool yIsForward = lenY >= lenX;
  const float fwdHalf = yIsForward ? halfY : halfX;
  const float latHalf = yIsForward ? halfX : halfY;
  const float groundZ = 0.8f;

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
  if (!m_wheel_positions_external) {
    const QVector3D wfl = wheelAt(-1.f, +1.f);
    const QVector3D wfr = wheelAt(+1.f, +1.f);
    const QVector3D wrl = wheelAt(-1.f, -1.f);
    const QVector3D wrr = wheelAt(+1.f, -1.f);
    if (m_wheel_xforms[0]) m_wheel_xforms[0]->setTranslation(wfl);
    if (m_wheel_xforms[1]) m_wheel_xforms[1]->setTranslation(wfr);
    if (m_wheel_xforms[2]) m_wheel_xforms[2]->setTranslation(wrl);
    if (m_wheel_xforms[3]) m_wheel_xforms[3]->setTranslation(wrr);
    m_last_wheel_pos[0] = wfl; m_last_wheel_pos[1] = wfr;
    m_last_wheel_pos[2] = wrl; m_last_wheel_pos[3] = wrr;
    update_alignment_lines();
  }

  if (m_align_rods[5].mesh) {
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
    updateAlignRod(m_align_rods[5].mesh, m_align_rods[5].xform, left, right);
  }

  if (m_calibration_badge) {
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
    m_calibration_badge->setStyleSheet(QString(
        "QLabel { background-color: %1; color: white; padding: 6px 12px; "
        "border-radius: 4px; font-weight: 700; font-size: 13px; }").arg(bg));
    m_calibration_badge->setText(text);
    m_calibration_badge->adjustSize();
    if (auto *parent = qobject_cast<QWidget*>(m_calibration_badge->parent())) {
      const int margin = 12;
      m_calibration_badge->move(
          parent->width()  - m_calibration_badge->width()  - margin,
          parent->height() - m_calibration_badge->height() - margin);
    }
    m_calibration_badge->raise();
    m_calibration_badge->show();
  }

  const float ext_cm[3] = { ext[0]*scaleHint, ext[1]*scaleHint, ext[2]*scaleHint };
  const float volM3 = (ext_cm[0] * ext_cm[1] * ext_cm[2]) / 1.0e6f;
  const char *cls =
      volM3 < 3.0f  ? "kart" :
      volM3 < 12.0f ? "sedan" :
      volM3 < 25.0f ? "suv" : "truck";

  int fwdAxis = (ext[0] >= ext[1] && ext[0] >= ext[2]) ? 0
              : (ext[1] >= ext[2]) ? 1 : 2;
  const float ctr[3] = { 0.5f*(minV[0]+maxV[0]), 0.5f*(minV[1]+maxV[1]), 0.5f*(minV[2]+maxV[2]) };
  long front = 0, back = 0;
  for (int i = 0; i < g.vertex_count(); ++i) {
    float x, y, z; g.vertex(i, x, y, z);
    const float v = (fwdAxis == 0) ? x : (fwdAxis == 1) ? y : z;
    if (v > ctr[fwdAxis]) ++front; else ++back;
  }
  const int fwdSign = front >= back ? +1 : -1;
  const char fwdName = (fwdAxis == 0) ? 'X' : (fwdAxis == 1) ? 'Y' : 'Z';

  QString info;
  info += QString("Verts: %1   Faces: %2   Malformed faces: %3\n")
              .arg(g.vertex_count()).arg(g.face_count()).arg(g.malformed_faces);
  info += QString("Source units: %1  (display rescaled by ×%2)\n").arg(units_hint).arg(scaleHint);
  info += QString("Extent (cm): %1 × %2 × %3   →  %4 × %5 × %6 m\n")
              .arg(ext_cm[0],0,'f',1).arg(ext_cm[1],0,'f',1).arg(ext_cm[2],0,'f',1)
              .arg(ext_cm[0]/100.0f,0,'f',2).arg(ext_cm[1]/100.0f,0,'f',2).arg(ext_cm[2]/100.0f,0,'f',2);
  info += QString("Volume: %1 m^3   →  class=%2\n").arg(volM3,0,'f',2).arg(cls);
  info += QString("Forward axis: %1%2  (%3 verts ahead, %4 behind)\n")
              .arg(fwdSign>0?'+':'-').arg(fwdName).arg(front).arg(back);
  m_info_box->setPlainText(info);
}

void VehiclePreviewPage::view_top()
{
  if (!m_view || !m_view->camera()) return;
  auto *cam = m_view->camera();
  cam->setUpVector(QVector3D(0, 1, 0));
  cam->setPosition(QVector3D(0, 0, 1500));
  cam->setViewCenter(QVector3D(0, 0, 0));
}

void VehiclePreviewPage::view_side()
{
  if (!m_view || !m_view->camera()) return;
  auto *cam = m_view->camera();
  cam->setUpVector(QVector3D(0, 0, 1));
  cam->setPosition(QVector3D(1500, 0, 100));
  cam->setViewCenter(QVector3D(0, 0, 80));
}

void VehiclePreviewPage::view_front()
{
  if (!m_view || !m_view->camera()) return;
  auto *cam = m_view->camera();
  cam->setUpVector(QVector3D(0, 0, 1));
  cam->setPosition(QVector3D(0, -1500, 100));
  cam->setViewCenter(QVector3D(0, 0, 80));
}

void VehiclePreviewPage::view_3d()
{
  resetCamera();
}

void VehiclePreviewPage::fit_camera()
{
  if (!m_path_edit->text().isEmpty()) update_mesh_bounds();
}

void VehiclePreviewPage::apply_adjustment(const QString &label)
{
  if (m_mesh_transform) {
    QQuaternion q = QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), m_adjust_yaw_deg);
    m_mesh_transform->setRotation(q);
    m_mesh_transform->setScale3D(QVector3D(m_adjust_mirror_x, m_adjust_mirror_y, 1.0f));
  }
  if (m_info_box) {
    m_info_box->appendPlainText(QString("[adjust] %1   yaw=%2°  mirror=(%3,%4)")
                                .arg(label)
                                .arg(m_adjust_yaw_deg, 0, 'f', 0)
                                .arg(m_adjust_mirror_x > 0 ? "+" : "-")
                                .arg(m_adjust_mirror_y > 0 ? "+" : "-"));
  }
}

void VehiclePreviewPage::rotate_minus90() { m_adjust_yaw_deg -= 90.f; apply_adjustment("rotate -90"); }
void VehiclePreviewPage::rotate_plus90()  { m_adjust_yaw_deg += 90.f; apply_adjustment("rotate +90"); }
void VehiclePreviewPage::rotate180()     { m_adjust_yaw_deg += 180.f; apply_adjustment("flip 180"); }
void VehiclePreviewPage::mirror_x()       { m_adjust_mirror_x *= -1.f; apply_adjustment("mirror X"); }
void VehiclePreviewPage::mirror_y()       { m_adjust_mirror_y *= -1.f; apply_adjustment("mirror Y"); }

void VehiclePreviewPage::recenter_mesh()
{
  if (!m_path_edit->text().isEmpty()) update_mesh_bounds();
}

void VehiclePreviewPage::reset_mesh_transform()
{
  m_adjust_yaw_deg  = 0.f;
  m_adjust_mirror_x = 1.f;
  m_adjust_mirror_y = 1.f;
  apply_adjustment("reset");
  if (!m_path_edit->text().isEmpty()) update_mesh_bounds();
}

bool VehiclePreviewPage::eventFilter(QObject *obj, QEvent *ev)
{
  if (ev->type() == QEvent::Resize) {
    if (m_calibration_badge && m_calibration_badge->isVisible()) {
      if (auto *parent = qobject_cast<QWidget*>(m_calibration_badge->parent())) {
        const int margin = 12;
        m_calibration_badge->move(
            parent->width()  - m_calibration_badge->width()  - margin,
            parent->height() - m_calibration_badge->height() - margin);
      }
    }
  }
  return QWidget::eventFilter(obj, ev);
}

void VehiclePreviewPage::resizeEvent(QResizeEvent *ev)
{
  QWidget::resizeEvent(ev);
}

void VehiclePreviewPage::set_tool_mode(ToolMode mode)
{
  m_tool_mode = mode;
  const int idx = (mode == ToolMode::Import) ? 0 : 1;
  if (m_left_stack)  m_left_stack->setCurrentIndex(idx);
  if (m_right_stack) m_right_stack->setCurrentIndex(idx);
  if (m_apply_btn)   m_apply_btn->setVisible(mode == ToolMode::Import);
  if (m_import_mode_btn) m_import_mode_btn->setChecked(mode == ToolMode::Import);
  if (m_sensor_mode_btn) m_sensor_mode_btn->setChecked(mode == ToolMode::SensorAssembly);
}

void VehiclePreviewPage::apply_to_spec()
{
  if (m_info_box) {
    m_info_box->appendPlainText(QString("[apply→spec] yaw=%1°  mirror=(%2,%3) - re-import requested")
                                .arg(m_adjust_yaw_deg, 0, 'f', 0)
                                .arg(m_adjust_mirror_x > 0 ? "+" : "-")
                                .arg(m_adjust_mirror_y > 0 ? "+" : "-"));
  }
  emit apply_requested(m_adjust_yaw_deg, m_adjust_mirror_x, m_adjust_mirror_y);
}

void VehiclePreviewPage::capture_window()
{

  QWidget *target = window() ? window() : this;
  QPixmap pix = target->grab();
  if (pix.isNull()) {
    if (m_info_box) m_info_box->appendPlainText("[capture] grab returned null pixmap.");
    return;
  }
  const QString cwdShots = QDir::currentPath() + "/.cs_import/calib_shots";
  const QString fallback = "/tmp/qt3d_shots";
  QString out_dir = cwdShots;
  if (!QDir().mkpath(out_dir)) out_dir = fallback;
  if (!QDir().mkpath(out_dir)) {
    if (m_info_box) m_info_box->appendPlainText("[capture] could not create output dir.");
    return;
  }
  const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
  const QString stem = QFileInfo(m_path_edit->text()).completeBaseName();
  const QString outPath = QString("%1/%2_%3.png").arg(out_dir, stem.isEmpty() ? "calib" : stem, stamp);
  if (!pix.save(outPath, "PNG")) {
    if (m_info_box) m_info_box->appendPlainText("[capture] save failed: " + outPath);
    return;
  }
  QGuiApplication::clipboard()->setText(outPath);
  if (m_info_box) m_info_box->appendPlainText("[capture] saved → " + outPath + "  (path copied to clipboard)");
  if (m_status_label) m_status_label->setText("Captured → " + outPath);
}

}

#endif
