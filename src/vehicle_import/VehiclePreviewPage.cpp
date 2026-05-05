// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "VehiclePreviewPage.h"

#ifndef CARLA_STUDIO_WITH_QT3D
namespace carla_studio::vehicle_import {
VehiclePreviewPage::VehiclePreviewPage(QWidget *parent) : QWidget(parent) {}
void VehiclePreviewPage::loadMesh(const QString&) {}
void VehiclePreviewPage::setWheelPositionsCm(const QVector3D&, const QVector3D&,
                                             const QVector3D&, const QVector3D&) {}
void VehiclePreviewPage::resetCamera() {}
void VehiclePreviewPage::onBrowse() {}
void VehiclePreviewPage::onLoadClicked() {}
void VehiclePreviewPage::onSceneStatusChanged(int) {}
void VehiclePreviewPage::buildScene() {}
void VehiclePreviewPage::buildGridFloor(Qt3DCore::QEntity*, float, float) {}
void VehiclePreviewPage::buildAxisGizmo(Qt3DCore::QEntity*, float) {}
void VehiclePreviewPage::buildReferenceOutlines(Qt3DCore::QEntity*) {}
void VehiclePreviewPage::buildWheelMarkers(Qt3DCore::QEntity*) {}
void VehiclePreviewPage::updateMeshBounds() {}
void VehiclePreviewPage::viewTop() {}
void VehiclePreviewPage::viewSide() {}
void VehiclePreviewPage::viewFront() {}
void VehiclePreviewPage::view3D() {}
void VehiclePreviewPage::fitCamera() {}
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

#include "MeshGeometry.h"

#include <QEvent>
#include <QFileDialog>
#include <QResizeEvent>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
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
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QPointLight>
#include <Qt3DRender/QDirectionalLight>
#include <Qt3DRender/QSceneLoader>
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
  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(8, 8, 8, 8);
  outer->setSpacing(6);

  auto *topRow = new QHBoxLayout();
  topRow->addWidget(new QLabel("Mesh:"));
  mPathEdit = new QLineEdit();
  mPathEdit->setPlaceholderText("/path/to/vehicle.obj or .fbx");
  topRow->addWidget(mPathEdit, 1);
  mBrowseBtn = new QPushButton("Browse…");
  mLoadBtn   = new QPushButton("Load");
  mResetCamBtn = new QPushButton("Reset View");
  topRow->addWidget(mBrowseBtn);
  topRow->addWidget(mLoadBtn);
  topRow->addWidget(mResetCamBtn);
  outer->addLayout(topRow);

  auto *toolbar = new QHBoxLayout();
  toolbar->setSpacing(4);
  auto addBtn = [&](const QString &label, const QString &tip,
                    void (VehiclePreviewPage::*slot)()) {
    auto *b = new QPushButton(label, this);
    b->setToolTip(tip);
    b->setMaximumWidth(80);
    connect(b, &QPushButton::clicked, this, slot);
    toolbar->addWidget(b);
  };
  toolbar->addWidget(new QLabel("View:"));
  addBtn("Top",   "Top-down ortho view (XY plane)",   &VehiclePreviewPage::viewTop);
  addBtn("Side",  "Side ortho view (XZ plane)",      &VehiclePreviewPage::viewSide);
  addBtn("Front", "Front ortho view (YZ plane)",     &VehiclePreviewPage::viewFront);
  addBtn("3D",    "Perspective camera, orbit-able",  &VehiclePreviewPage::view3D);
  addBtn("Fit",   "Re-frame camera around mesh",     &VehiclePreviewPage::fitCamera);
  toolbar->addSpacing(12);
  toolbar->addWidget(new QLabel("Adjust (preview):"));
  addBtn("Rot -90",  "Rotate -90° around vertical axis", &VehiclePreviewPage::rotateMinus90);
  addBtn("Rot +90",  "Rotate +90° around vertical axis", &VehiclePreviewPage::rotatePlus90);
  addBtn("Flip 180", "Rotate 180° around vertical axis", &VehiclePreviewPage::rotate180);
  addBtn("Mirror X", "Mirror across the X axis",         &VehiclePreviewPage::mirrorX);
  addBtn("Mirror Y", "Mirror across the Y axis",         &VehiclePreviewPage::mirrorY);
  addBtn("Recenter", "Recenter mesh on origin",          &VehiclePreviewPage::recenterMesh);
  addBtn("Reset",    "Clear all preview adjustments",    &VehiclePreviewPage::resetMeshTransform);
  toolbar->addSpacing(12);
  {
    auto *applyBtn = new QPushButton("Apply & Re-import", this);
    applyBtn->setToolTip("Send current yaw/mirror adjustments back to the importer "
                         "and re-run import so the cooked vehicle matches the preview.");
    applyBtn->setStyleSheet(
        "QPushButton { padding: 4px 10px; font-weight: 600; "
        "background-color: #FFE082; color: #6A4F00; border: 1px solid #FFD54F; "
        "border-radius: 3px; }");
    connect(applyBtn, &QPushButton::clicked, this, &VehiclePreviewPage::applyToSpec);
    toolbar->addWidget(applyBtn);
  }
  toolbar->addStretch();
  outer->addLayout(toolbar);

  buildScene();

  auto *container = QWidget::createWindowContainer(mView, this);
  container->setMinimumSize(640, 420);
  container->setFocusPolicy(Qt::StrongFocus);
  outer->addWidget(container, 1);

  mCalibrationBadge = new QLabel(container);
  mCalibrationBadge->setStyleSheet(
      "QLabel { background-color: rgba(20, 20, 20, 200); color: #DDDDDD; "
      "padding: 6px 12px; border-radius: 4px; font-weight: 700; "
      "font-size: 13px; }");
  mCalibrationBadge->setText("");
  mCalibrationBadge->hide();
  container->installEventFilter(this);   // catches resize to reposition

  mStatusLabel = new QLabel("Idle.");
  mStatusLabel->setStyleSheet("color: #555;");
  outer->addWidget(mStatusLabel);

  mInfoBox = new QPlainTextEdit();
  mInfoBox->setReadOnly(true);
  mInfoBox->setMaximumHeight(120);
  mInfoBox->setStyleSheet("font-family: monospace; font-size: 11px;");
  outer->addWidget(mInfoBox);

  connect(mBrowseBtn,   &QPushButton::clicked, this, &VehiclePreviewPage::onBrowse);
  connect(mLoadBtn,     &QPushButton::clicked, this, &VehiclePreviewPage::onLoadClicked);
  connect(mResetCamBtn, &QPushButton::clicked, this, &VehiclePreviewPage::resetCamera);
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

  mMeshEntity    = new Qt3DCore::QEntity(mRoot);
  mMeshTransform = new Qt3DCore::QTransform();
  mMeshEntity->addComponent(mMeshTransform);
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
}

void VehiclePreviewPage::buildReferenceOutlines(Qt3DCore::QEntity *root)
{
  const float zLift = 0.6f;
  makeStencilRect(root,  30.f,  85.f, zLift, 1.5f, QColor(170, 220, 255));
  makeStencilRect(root,  92.f, 230.f, zLift, 1.5f, QColor(120, 220, 130));
  makeStencilRect(root, 125.f, 250.f, zLift, 1.5f, QColor( 80, 200, 220));
  makeStencilRect(root, 125.f, 300.f, zLift, 1.5f, QColor(240, 170, 100));
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

void VehiclePreviewPage::loadMesh(const QString &path)
{
  mPathEdit->setText(path);
  mStatusLabel->setText("Loading: " + path);
  mInfoBox->clear();

  MeshGeometry g = loadMeshGeometry(path);
  if (!g.valid) {
    mStatusLabel->setText("Load FAILED.");
    mInfoBox->setPlainText("loadMeshGeometry: returned invalid mesh for " + path);
    return;
  }
  if (!installMeshGeometry(mMeshEntity, mMeshTransform, g)) {
    mStatusLabel->setText("Load FAILED (no renderable geometry).");
    return;
  }
  mStatusLabel->setText("Loaded.");
  updateMeshBounds();
}

void VehiclePreviewPage::setWheelPositionsCm(const QVector3D &fl,
                                             const QVector3D &fr,
                                             const QVector3D &rl,
                                             const QVector3D &rr)
{
  const QVector3D pts[4] = { fl, fr, rl, rr };
  for (int i = 0; i < 4; ++i) {
    if (mWheelXforms[i]) mWheelXforms[i]->setTranslation(pts[i]);
  }
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
    case S::Ready:   mStatusLabel->setText("Loaded."); updateMeshBounds(); break;
    case S::Error:   mStatusLabel->setText("Load FAILED."); break;
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
  if (mWheelXforms[0]) mWheelXforms[0]->setTranslation(wheelAt(-1.f, +1.f));
  if (mWheelXforms[1]) mWheelXforms[1]->setTranslation(wheelAt(+1.f, +1.f));
  if (mWheelXforms[2]) mWheelXforms[2]->setTranslation(wheelAt(-1.f, -1.f));
  if (mWheelXforms[3]) mWheelXforms[3]->setTranslation(wheelAt(+1.f, -1.f));

  if (mCalibrationBadge) {
    QString text;
    QString bg;
    const bool fwdYdominant = (lenY >= lenX) && (lenY >= lenZ);
    const bool zIsShortest  = (lenZ < lenX) && (lenZ < lenY);
    if (fwdYdominant && zIsShortest) {
      text = QStringLiteral("✓  No calibration required");
      bg   = "rgba(20, 110, 50, 220)";
    } else if (lenX > lenY && zIsShortest) {
      text = QStringLiteral("✗  Forward axis is X — try Rot +90 or Rot -90");
      bg   = "rgba(170, 30, 30, 220)";
    } else if (!zIsShortest) {
      text = QStringLiteral("✗  Up-axis looks wrong (Z not shortest) — try Rot +90");
      bg   = "rgba(170, 30, 30, 220)";
    } else {
      text = QStringLiteral("⚠  Geometry unusual — verify orientation manually");
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
  if (ev->type() == QEvent::Resize && mCalibrationBadge && mCalibrationBadge->isVisible()) {
    if (auto *parent = qobject_cast<QWidget*>(mCalibrationBadge->parent())) {
      const int margin = 12;
      mCalibrationBadge->move(
          parent->width()  - mCalibrationBadge->width()  - margin,
          parent->height() - mCalibrationBadge->height() - margin);
    }
  }
  return QWidget::eventFilter(obj, ev);
}

void VehiclePreviewPage::applyToSpec()
{
  if (mInfoBox) {
    mInfoBox->appendPlainText(QString("[apply→spec] yaw=%1°  mirror=(%2,%3) — re-import requested")
                                .arg(mAdjustYawDeg, 0, 'f', 0)
                                .arg(mAdjustMirrorX > 0 ? "+" : "-")
                                .arg(mAdjustMirrorY > 0 ? "+" : "-"));
  }
  emit applyRequested(mAdjustYawDeg, mAdjustMirrorX, mAdjustMirrorY);
}

}  // namespace carla_studio::vehicle_import

#endif  // CARLA_STUDIO_WITH_QT3D
