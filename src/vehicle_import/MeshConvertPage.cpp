// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_import/MeshConvertPage.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QStandardPaths>
#include <QTemporaryFile>
#include <QTextStream>
#include <QTime>
#include <QVBoxLayout>

namespace carla_studio::vehicle_import {

namespace {

QString stamp() { return QTime::currentTime().toString("hh:mm:ss"); }

QString resolveBlenderBinary() {
  return QStandardPaths::findExecutable("blender");
}

QString conversionScript() {
  return QStringLiteral(R"PY(
import bpy, sys, os
src = sys.argv[sys.argv.index('--') + 1]
dst = sys.argv[sys.argv.index('--') + 2]
ext = os.path.splitext(src)[1].lower()
bpy.ops.wm.read_homefile(use_empty=True)
if ext == '.blend':
    bpy.ops.wm.open_mainfile(filepath=src)
elif ext in ('.fbx',):
    bpy.ops.import_scene.fbx(filepath=src)
elif ext in ('.gltf', '.glb'):
    bpy.ops.import_scene.gltf(filepath=src)
elif ext in ('.dae',):
    bpy.ops.wm.collada_import(filepath=src)
else:
    print(f'unsupported input extension: {ext}', file=sys.stderr); sys.exit(2)
total_v = total_f = 0
for o in bpy.data.objects:
    if o.type == 'MESH':
        total_v += len(o.data.vertices)
        total_f += len(o.data.polygons)
        print(f'mesh: {o.name}  v={len(o.data.vertices)}  f={len(o.data.polygons)}', file=sys.stderr)
print(f'TOTAL  v={total_v}  f={total_f}', file=sys.stderr)
for o in bpy.data.objects:
    if o.type == 'ARMATURE':
        for b in o.data.bones:
            head = o.matrix_world @ b.head_local
            print(f'bone: {b.name}  head_world=({head.x:.3f}, {head.y:.3f}, {head.z:.3f})', file=sys.stderr)
bpy.ops.wm.obj_export(filepath=dst, forward_axis='Y', up_axis='Z',
                      apply_modifiers=True, export_uv=False, export_normals=False,
                      export_materials=False, export_triangulated_mesh=True)
print(f'wrote {dst}', file=sys.stderr)
)PY");
}

}  // namespace

MeshConvertPage::MeshConvertPage(HandoffToImport handoff, QWidget *parent)
    : QWidget(parent), mHandoff(std::move(handoff)) {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(12, 12, 12, 12);
  root->setSpacing(6);

  auto *inputRow = new QHBoxLayout();
  auto *inLbl = new QLabel("Source:");
  inLbl->setFixedWidth(60);
  mInputEdit = new QLineEdit();
  mInputEdit->setPlaceholderText("path to .fbx / .blend / .gltf / .glb / .dae");
  mInputEdit->setReadOnly(true);
  mBrowseBtn = new QPushButton("Browse…");
  inputRow->addWidget(inLbl);
  inputRow->addWidget(mInputEdit, 1);
  inputRow->addWidget(mBrowseBtn);
  root->addLayout(inputRow);

  mDetectedLabel = new QLabel();
  mDetectedLabel->setStyleSheet("color: #555;");
  mDetectedLabel->setVisible(false);
  root->addWidget(mDetectedLabel);

  auto *outRow = new QHBoxLayout();
  auto *outLbl = new QLabel("Output:");
  outLbl->setFixedWidth(60);
  mOutputEdit = new QLineEdit();
  mOutputEdit->setPlaceholderText("auto: <input>.obj in same folder");
  outRow->addWidget(outLbl);
  outRow->addWidget(mOutputEdit, 1);
  root->addLayout(outRow);

  auto *btnRow = new QHBoxLayout();
  mBlenderLabel = new QLabel();
  btnRow->addWidget(mBlenderLabel, 1);
  mCancelBtn = new QPushButton("Cancel");
  mCancelBtn->setEnabled(false);
  mConvertBtn = new QPushButton("Convert to OBJ");
  mConvertBtn->setEnabled(false);
  mHandoffBtn = new QPushButton("Use in Vehicle Import");
  mHandoffBtn->setEnabled(false);
  mHandoffBtn->setToolTip("Switch to the From 3D Model tab with the converted OBJ pre-loaded.");
  btnRow->addWidget(mCancelBtn);
  btnRow->addWidget(mConvertBtn);
  btnRow->addWidget(mHandoffBtn);
  root->addLayout(btnRow);

  mLog = new QPlainTextEdit();
  mLog->setReadOnly(true);
  mLog->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  mLog->setMinimumHeight(160);
  mLog->setPlaceholderText("Conversion log will appear here…");
  root->addWidget(mLog, 1);

  setBlenderStatus();

  connect(mBrowseBtn,  &QPushButton::clicked, this, &MeshConvertPage::onBrowseInput);
  connect(mConvertBtn, &QPushButton::clicked, this, &MeshConvertPage::onConvert);
  connect(mCancelBtn,  &QPushButton::clicked, this, &MeshConvertPage::onCancel);
  connect(mHandoffBtn, &QPushButton::clicked, this, [this]() {
    if (mHandoff && !mLastOutputPath.isEmpty()) mHandoff(mLastOutputPath);
  });
}

void MeshConvertPage::setBlenderStatus() {
  const QString bin = resolveBlenderBinary();
  if (bin.isEmpty()) {
    mBlenderLabel->setText("blender CLI not found on PATH — install it to enable conversion");
    mBlenderLabel->setStyleSheet("color: #B00020;");
    mConvertBtn->setEnabled(false);
  } else {
    mBlenderLabel->setText(QString("blender: %1").arg(bin));
    mBlenderLabel->setStyleSheet("color: #1B5E20;");
  }
}

void MeshConvertPage::onBrowseInput() {
  const QString seed = mInputEdit->text().trimmed();
  const QString p = QFileDialog::getOpenFileName(
      this, "Select source mesh",
      seed.isEmpty() ? QStandardPaths::writableLocation(QStandardPaths::HomeLocation) : QFileInfo(seed).absolutePath(),
      "All meshes (*.fbx *.blend *.gltf *.glb *.dae);;FBX (*.fbx);;Blender (*.blend);;glTF (*.gltf *.glb);;Collada (*.dae)");
  if (p.isEmpty()) return;
  mInputEdit->setText(p);
  const QString ext = QFileInfo(p).suffix().toLower();
  mDetectedLabel->setText(QString("Detected: %1").arg(ext.toUpper()));
  mDetectedLabel->setVisible(true);
  if (mOutputEdit->text().trimmed().isEmpty()) {
    mOutputEdit->setText(QFileInfo(p).absolutePath() + "/" + QFileInfo(p).completeBaseName() + ".obj");
  }
  if (!resolveBlenderBinary().isEmpty()) mConvertBtn->setEnabled(true);
}

void MeshConvertPage::onConvert() {
  const QString src = mInputEdit->text().trimmed();
  QString dst = mOutputEdit->text().trimmed();
  if (src.isEmpty()) { appendLog("Error: pick a source mesh first."); return; }
  if (dst.isEmpty())
    dst = QFileInfo(src).absolutePath() + "/" + QFileInfo(src).completeBaseName() + ".obj";
  const QString blender = resolveBlenderBinary();
  if (blender.isEmpty()) { appendLog("Error: blender CLI not on PATH."); return; }

  QTemporaryFile *script = new QTemporaryFile(this);
  script->setFileTemplate(QDir::tempPath() + "/carla_studio_convert_XXXXXX.py");
  if (!script->open()) { appendLog("Error: could not write temp script."); return; }
  QTextStream ts(script);
  ts << conversionScript();
  ts.flush();
  const QString scriptPath = script->fileName();
  script->close();

  if (mProc) { mProc->deleteLater(); mProc = nullptr; }
  mProc = new QProcess(this);
  mProc->setProcessChannelMode(QProcess::MergedChannels);
  connect(mProc, &QProcess::readyReadStandardOutput, this, [this]() {
    const QString out = QString::fromLocal8Bit(mProc->readAllStandardOutput());
    for (const QString &line : out.split('\n', Qt::SkipEmptyParts)) appendLog(line);
  });
  connect(mProc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, [this](int code, QProcess::ExitStatus) { onProcessFinished(code); });
  mLastOutputPath = dst;
  mConvertBtn->setEnabled(false);
  mCancelBtn->setEnabled(true);
  mHandoffBtn->setEnabled(false);
  appendLog(QString("[%1]  Running: blender --background --python %2 -- '%3' '%4'")
              .arg(stamp(), scriptPath, src, dst));
  mProc->start(blender, QStringList() << "--background" << "--python" << scriptPath
                                      << "--" << src << dst);
}

void MeshConvertPage::onCancel() {
  if (mProc && mProc->state() != QProcess::NotRunning) {
    mProc->kill();
    mProc->waitForFinished(2000);
    appendLog("Cancelled.");
  }
  mConvertBtn->setEnabled(!mInputEdit->text().trimmed().isEmpty()
                          && !resolveBlenderBinary().isEmpty());
  mCancelBtn->setEnabled(false);
}

void MeshConvertPage::onProcessFinished(int exitCode) {
  mCancelBtn->setEnabled(false);
  mConvertBtn->setEnabled(true);
  if (exitCode == 0 && QFileInfo(mLastOutputPath).isFile()) {
    appendLog(QString("[%1]  Conversion succeeded → %2 (%3 bytes)")
                .arg(stamp(), mLastOutputPath)
                .arg(QFileInfo(mLastOutputPath).size()));
    mHandoffBtn->setEnabled(true);
    return;
  }
  const QString full = mLog ? mLog->toPlainText() : QString();
  if (full.contains("not a blend file") || full.contains("Failed to read blend file")) {
    appendLog(QString("[%1]  Hint: this .blend was likely authored in a newer Blender than the "
                      "local one. Install matching Blender (or use the .fbx export of the same "
                      "scene if available — .fbx is version-agnostic).").arg(stamp()));
  } else {
    appendLog(QString("[%1]  Conversion failed (exit %2). See log above for Blender errors.")
                .arg(stamp()).arg(exitCode));
  }
}

void MeshConvertPage::appendLog(const QString &line) {
  if (mLog) mLog->appendPlainText(line);
}

}  // namespace carla_studio::vehicle_import
