// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/prebuilt_package_page.h"

#include <QColor>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QStandardPaths>
#include <QTime>
#include <QVBoxLayout>

namespace carla_studio::vehicle_import {

namespace {
QString stamp() { return QTime::currentTime().toString("hh:mm:ss"); }
}

PrebuiltPackagePage::PrebuiltPackagePage(CarlaRootResolver find_carla_root,
                                         QWidget *parent)
    : QWidget(parent), m_find_carla_root(std::move(find_carla_root)) {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(12, 12, 12, 12);
  root->setSpacing(10);

  auto *banner = new QLabel(
    "Installs a pre-cooked vehicle (.uasset + .uexp) into your CARLA root (No Unreal Editor required)");
  banner->setWordWrap(true);
  banner->setStyleSheet("color: #555; font-size: 11px;");
  root->addWidget(banner);

  auto *srcGroup = new QGroupBox("Package Folder");
  auto *srcRow = new QHBoxLayout();
  m_folder_edit = new QLineEdit();
  m_folder_edit->setPlaceholderText("Folder containing .uasset and .uexp files…");
  m_folder_edit->setReadOnly(true);
  m_browse_btn = new QPushButton("Browse…");
  srcRow->addWidget(m_folder_edit, 1);
  srcRow->addWidget(m_browse_btn);
  auto *srcLayout = new QVBoxLayout(srcGroup);
  srcLayout->addLayout(srcRow);
  root->addWidget(srcGroup);

  auto *filesGroup = new QGroupBox("Files in package");
  m_file_list = new QListWidget();
  auto *filesLayout = new QVBoxLayout(filesGroup);
  filesLayout->addWidget(m_file_list);
  root->addWidget(filesGroup, 1);

  auto *dstGroup = new QGroupBox("Install destination");
  auto *dstForm = new QVBoxLayout(dstGroup);
  auto *nameRow = new QHBoxLayout();
  nameRow->addWidget(new QLabel("Vehicle folder name:"));
  m_name_edit = new QLineEdit();
  m_name_edit->setPlaceholderText("Auto-detected from BP_ filename…");
  nameRow->addWidget(m_name_edit, 1);
  dstForm->addLayout(nameRow);

  auto *dstRow = new QHBoxLayout();
  dstRow->addWidget(new QLabel("Destination:"));
  m_dst_label = new QLabel("Set CARLA_ROOT and browse to a package folder.");
  m_dst_label->setWordWrap(true);
  m_dst_label->setStyleSheet("color: gray; font-size: 11px;");
  dstRow->addWidget(m_dst_label, 1);
  dstForm->addLayout(dstRow);
  root->addWidget(dstGroup);

  auto *btn_row = new QHBoxLayout();
  btn_row->addStretch();
  m_install_btn = new QPushButton("Install Package");
  m_install_btn->setEnabled(false);
  btn_row->addWidget(m_install_btn);
  root->addLayout(btn_row);

  m_status_label = new QLabel();
  m_status_label->setStyleSheet("color: gray; font-size: 11px;");
  root->addWidget(m_status_label);

  m_log = new QPlainTextEdit();
  m_log->setReadOnly(true);
  m_log->setMaximumHeight(110);
  m_log->setPlaceholderText("Install log will appear here…");
  root->addWidget(m_log);

  connect(m_browse_btn,  &QPushButton::clicked, this, &PrebuiltPackagePage::on_browse);
  connect(m_install_btn, &QPushButton::clicked, this, &PrebuiltPackagePage::on_install);
  connect(m_name_edit,   &QLineEdit::textChanged, this,
          [this](const QString &) { refresh_destination(); });

  refresh_destination();
}

QString PrebuiltPackagePage::resolve_vehicles_dir() const {
  const QString root = m_find_carla_root ? m_find_carla_root() : QString();
  if (root.isEmpty()) return QString();
  QDir d(root);
  const QStringList shippingDirs = d.entryList(
      QStringList() << "Carla-*-Linux-Shipping" << "Carla-*-Shipping",
      QDir::Dirs | QDir::NoDotAndDotDot);
  for (const QString &sub : shippingDirs) {
    const QString p = root + "/" + sub +
                      "/CarlaUnreal/Content/Carla/Blueprints/Vehicles";
    if (QDir(p).exists()) return p;
  }
  const QString src = root + "/Unreal/CarlaUnreal/Content/Carla/Blueprints/Vehicles";
  if (QDir(src).exists()) return src;
  return QString();
}

void PrebuiltPackagePage::refresh_destination() {
  const QString name = m_name_edit->text().trimmed();
  const QString vehiclesDir = resolve_vehicles_dir();
  if (vehiclesDir.isEmpty()) {
    m_dst_label->setText("CARLA_ROOT not set or Vehicles/ directory not found.");
    m_dst_label->setStyleSheet("color: #CC4444; font-size: 11px;");
    m_install_btn->setEnabled(false);
    return;
  }
  if (name.isEmpty()) {
    m_dst_label->setText(vehiclesDir + "/<vehicle name>/");
    m_dst_label->setStyleSheet("color: gray; font-size: 11px;");
    m_install_btn->setEnabled(false);
    return;
  }
  m_dst_label->setText(vehiclesDir + "/" + name + "/");
  m_dst_label->setStyleSheet("color: #44AA66; font-size: 11px;");
  m_install_btn->setEnabled(!m_folder_edit->text().isEmpty());
}

void PrebuiltPackagePage::on_browse() {
  const QString folder = QFileDialog::getExistingDirectory(
      this, "Select vehicle package folder",
      QStandardPaths::writableLocation(QStandardPaths::HomeLocation));
  if (folder.isEmpty()) return;

  m_folder_edit->setText(folder);
  m_file_list->clear();
  m_install_btn->setEnabled(false);

  const QDir d(folder);
  const QStringList uassets = d.entryList({"*.uasset"}, QDir::Files);
  const QStringList uexps   = d.entryList({"*.uexp"},   QDir::Files);

  if (uassets.isEmpty()) {
    m_file_list->addItem("⚠  No .uasset files found in this folder.");
    m_status_label->setText("Not a valid package folder.");
    m_status_label->setStyleSheet("color: #CC4444; font-size: 11px;");
    return;
  }

  for (const QString &f : uassets) {
    auto *it = new QListWidgetItem("● " + f);
    it->setForeground(QColor("#44AAFF"));
    m_file_list->addItem(it);
  }
  for (const QString &f : uexps) {
    auto *it = new QListWidgetItem("  " + f);
    it->setForeground(QColor("#88BBDD"));
    m_file_list->addItem(it);
  }

  if (m_name_edit->text().trimmed().isEmpty()) {
    QString guess;
    for (const QString &a : uassets) {
      if (a.startsWith("BP_", Qt::CaseInsensitive)) {
        guess = QFileInfo(a).completeBaseName().mid(3);
        break;
      }
    }
    if (guess.isEmpty()) guess = QFileInfo(uassets.first()).completeBaseName();
    static const QStringList kSuffixes = { "_FLW", "_FRW", "_RLW", "_RRW", "_Wheel" };
    for (const QString &s : kSuffixes) {
      if (guess.endsWith(s, Qt::CaseInsensitive)) { guess.chop(s.size()); break; }
    }
    m_name_edit->setText(guess);
  }

  m_status_label->setText(QString("%1 file(s) found  ·  %2 .uasset  ·  %3 .uexp")
                          .arg(uassets.size() + uexps.size())
                          .arg(uassets.size()).arg(uexps.size()));
  m_status_label->setStyleSheet("color: #44AA66; font-size: 11px;");
  refresh_destination();
}

void PrebuiltPackagePage::on_install() {
  const QString src  = m_folder_edit->text().trimmed();
  const QString name = m_name_edit->text().trimmed();
  const QString vehiclesDir = resolve_vehicles_dir();
  if (src.isEmpty() || name.isEmpty() || vehiclesDir.isEmpty()) {
    m_log->appendPlainText(QString("[%1]  Cannot install: source / name / CARLA_ROOT missing.").arg(stamp()));
    return;
  }
  const QString dstFolder = vehiclesDir + "/" + name;
  QDir().mkpath(dstFolder);

  const QDir d(src);
  const QStringList files = d.entryList({"*.uasset", "*.uexp"}, QDir::Files);
  int ok = 0, fail = 0;
  for (const QString &f : files) {
    const QString s = src + "/" + f;
    const QString t = dstFolder + "/" + f;
    if (QFile::exists(t)) QFile::remove(t);
    if (QFile::copy(s, t)) {
      ++ok;
      m_log->appendPlainText("  ✓ " + f);
    } else {
      ++fail;
      m_log->appendPlainText("  ✗ " + f + " (copy failed)");
    }
  }
  if (fail == 0) {
    m_log->appendPlainText(QString("[%1]  Installed %2 file(s) → %3").arg(stamp()).arg(ok).arg(dstFolder));
    m_status_label->setText(QString("Installed %1 file(s) → %2").arg(ok).arg(dstFolder));
    m_status_label->setStyleSheet("color: #44AA66; font-size: 11px;");
  } else {
    m_log->appendPlainText(QString("[%1]  Done with errors: %2 ok, %3 failed.").arg(stamp()).arg(ok).arg(fail));
    m_status_label->setText(QString("Done with errors: %1 ok, %2 failed.").arg(ok).arg(fail));
    m_status_label->setStyleSheet("color: #CC4444; font-size: 11px;");
  }
}

}
