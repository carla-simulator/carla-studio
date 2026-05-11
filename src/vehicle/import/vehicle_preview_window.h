// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QDialog>
#include <QString>
#include <QVector3D>

namespace carla_studio::vehicle_import {

class VehiclePreviewPage;

class VehiclePreviewWindow : public QDialog {
  Q_OBJECT
 public:
  static VehiclePreviewWindow *instance();

  void show_for(const QString &mesh_path,
               const QVector3D &fl, const QVector3D &fr,
               const QVector3D &rl, const QVector3D &rr);

  void show_for_mesh(const QString &mesh_path);

  VehiclePreviewPage *page() const { return m_page; }

 private:
  explicit VehiclePreviewWindow(QWidget *parent = nullptr);
  VehiclePreviewPage *m_page = nullptr;
};

}
