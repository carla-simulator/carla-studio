// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/mesh_convert_page.h"

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

}

MeshConvertPage::MeshConvertPage(HandoffToImport handoff, QWidget *parent)
    : QWidget(parent), m_handoff(std::move(handoff)) {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(12, 12, 12, 12);
  root->setSpacing(6);

  auto *inputRow = new QHBoxLayout();
  auto *inLbl = new QLabel("Source:");
  inLbl->setFixedWidth(60);
  m_input_edit = new QLineEdit();
  m_input_edit->setPlaceholderText("path to .fbx / .blend / .gltf / .glb / .dae");
  m_input_edit->setReadOnly(true);
  m_browse_btn = new QPushButton("Browse…");
  inputRow->addWidget(inLbl);
  inputRow->addWidget(m_input_edit, 1);
  inputRow->addWidget(m_browse_btn);
  root->addLayout(inputRow);

  m_detected_label = new QLabel();
  m_detected_label->setStyleSheet("color: #555;");
  m_detected_label->setVisible(false);
  root->addWidget(m_detected_label);

  auto *outRow = new QHBoxLayout();
  auto *outLbl = new QLabel("Output:");
  outLbl->setFixedWidth(60);
  m_output_edit = new QLineEdit();
  m_output_edit->setPlaceholderText("auto: <input>.obj in same folder");
  outRow->addWidget(outLbl);
  outRow->addWidget(m_output_edit, 1);
  root->addLayout(outRow);

  auto *btn_row = new QHBoxLayout();
  m_blender_label = new QLabel();
  btn_row->addWidget(m_blender_label, 1);
  m_cancel_btn = new QPushButton("Cancel");
  m_cancel_btn->setEnabled(false);
  m_convert_btn = new QPushButton("Convert to OBJ");
  m_convert_btn->setEnabled(false);
  m_handoff_btn = new QPushButton("Use in Vehicle Import");
  m_handoff_btn->setEnabled(false);
  m_handoff_btn->setToolTip("Switch to the From 3D Model tab with the converted OBJ pre-loaded.");
  btn_row->addWidget(m_cancel_btn);
  btn_row->addWidget(m_convert_btn);
  btn_row->addWidget(m_handoff_btn);
  root->addLayout(btn_row);

  m_log = new QPlainTextEdit();
  m_log->setReadOnly(true);
  m_log->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  m_log->setMinimumHeight(160);
  m_log->setPlaceholderText("Conversion log will appear here…");
  root->addWidget(m_log, 1);

  set_blender_status();

  connect(m_browse_btn,  &QPushButton::clicked, this, &MeshConvertPage::on_browse_input);
  connect(m_convert_btn, &QPushButton::clicked, this, &MeshConvertPage::on_convert);
  connect(m_cancel_btn,  &QPushButton::clicked, this, &MeshConvertPage::on_cancel);
  connect(m_handoff_btn, &QPushButton::clicked, this, [this]() {
    if (m_handoff && !m_last_output_path.isEmpty()) m_handoff(m_last_output_path);
  });
}

void MeshConvertPage::set_blender_status() {
  const QString bin = resolveBlenderBinary();
  if (bin.isEmpty()) {
    m_blender_label->setText("blender CLI not found on PATH - install it to enable conversion");
    m_blender_label->setStyleSheet("color: #B00020;");
    m_convert_btn->setEnabled(false);
  } else {
    m_blender_label->setText(QString("blender: %1").arg(bin));
    m_blender_label->setStyleSheet("color: #1B5E20;");
  }
}

void MeshConvertPage::on_browse_input() {
  const QString seed = m_input_edit->text().trimmed();
  const QString p = QFileDialog::getOpenFileName(
      this, "Select source mesh",
      seed.isEmpty() ? QStandardPaths::writableLocation(QStandardPaths::HomeLocation) : QFileInfo(seed).absolutePath(),
      "All meshes (*.fbx *.blend *.gltf *.glb *.dae);;FBX (*.fbx);;Blender (*.blend);;glTF (*.gltf *.glb);;Collada (*.dae)");
  if (p.isEmpty()) return;
  m_input_edit->setText(p);
  const QString ext = QFileInfo(p).suffix().toLower();
  m_detected_label->setText(QString("Detected: %1").arg(ext.toUpper()));
  m_detected_label->setVisible(true);
  if (m_output_edit->text().trimmed().isEmpty()) {
    m_output_edit->setText(QFileInfo(p).absolutePath() + "/" + QFileInfo(p).completeBaseName() + ".obj");
  }
  if (!resolveBlenderBinary().isEmpty()) m_convert_btn->setEnabled(true);
}

void MeshConvertPage::on_convert() {
  const QString src = m_input_edit->text().trimmed();
  QString dst = m_output_edit->text().trimmed();
  if (src.isEmpty()) { append_log("Error: pick a source mesh first."); return; }
  if (dst.isEmpty())
    dst = QFileInfo(src).absolutePath() + "/" + QFileInfo(src).completeBaseName() + ".obj";
  const QString blender = resolveBlenderBinary();
  if (blender.isEmpty()) { append_log("Error: blender CLI not on PATH."); return; }

  QTemporaryFile *script = new QTemporaryFile(this);
  script->setFileTemplate(QDir::tempPath() + "/carla_studio_convert_XXXXXX.py");
  if (!script->open()) { append_log("Error: could not write temp script."); return; }
  QTextStream ts(script);
  ts << conversionScript();
  ts.flush();
  const QString scriptPath = script->fileName();
  script->close();

  if (m_proc) { m_proc->deleteLater(); m_proc = nullptr; }
  m_proc = new QProcess(this);
  m_proc->setProcessChannelMode(QProcess::MergedChannels);
  connect(m_proc, &QProcess::readyReadStandardOutput, this, [this]() {
    const QString out = QString::fromLocal8Bit(m_proc->readAllStandardOutput());
    for (const QString &line : out.split('\n', Qt::SkipEmptyParts)) append_log(line);
  });
  connect(m_proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, [this](int code, QProcess::ExitStatus) { on_process_finished(code); });
  m_last_output_path = dst;
  m_convert_btn->setEnabled(false);
  m_cancel_btn->setEnabled(true);
  m_handoff_btn->setEnabled(false);
  append_log(QString("[%1]  Running: blender --background --python %2 -- '%3' '%4'")
              .arg(stamp(), scriptPath, src, dst));
  m_proc->start(blender, QStringList() << "--background" << "--python" << scriptPath
                                      << "--" << src << dst);
}

void MeshConvertPage::on_cancel() {
  if (m_proc && m_proc->state() != QProcess::NotRunning) {
    m_proc->kill();
    m_proc->waitForFinished(2000);
    append_log("Cancelled.");
  }
  m_convert_btn->setEnabled(!m_input_edit->text().trimmed().isEmpty()
                          && !resolveBlenderBinary().isEmpty());
  m_cancel_btn->setEnabled(false);
}

void MeshConvertPage::on_process_finished(int exit_code) {
  m_cancel_btn->setEnabled(false);
  m_convert_btn->setEnabled(true);
  if (exit_code == 0 && QFileInfo(m_last_output_path).isFile()) {
    append_log(QString("[%1]  Conversion succeeded → %2 (%3 bytes)")
                .arg(stamp(), m_last_output_path)
                .arg(QFileInfo(m_last_output_path).size()));
    m_handoff_btn->setEnabled(true);
    return;
  }
  const QString full = m_log ? m_log->toPlainText() : QString();
  if (full.contains("not a blend file") || full.contains("Failed to read blend file")) {
    append_log(QString("[%1]  Hint: this .blend was likely authored in a newer Blender than the "
                      "local one. Install matching Blender (or use the .fbx export of the same "
                      "scene if available - .fbx is version-agnostic).").arg(stamp()));
  } else {
    append_log(QString("[%1]  Conversion failed (exit %2). See log above for Blender errors.")
                .arg(stamp()).arg(exit_code));
  }
}

void MeshConvertPage::append_log(const QString &line) {
  if (m_log) m_log->appendPlainText(line);
}

}
