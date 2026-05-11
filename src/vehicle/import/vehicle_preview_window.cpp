// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_preview_window.h"
#include "vehicle_preview_page.h"

#include <QVBoxLayout>

namespace carla_studio::vehicle_import {

VehiclePreviewWindow::VehiclePreviewWindow(QWidget *parent) : QDialog(parent)
{
  setWindowTitle("CARLA Studio - Vehicle Preview");
  setModal(false);
  setSizeGripEnabled(true);
  resize(1024, 720);

  auto *lay = new QVBoxLayout(this);
  lay->setContentsMargins(6, 6, 6, 6);
  m_page = new VehiclePreviewPage(this);
  lay->addWidget(m_page);
}

VehiclePreviewWindow *VehiclePreviewWindow::instance()
{
  static VehiclePreviewWindow *gWin = nullptr;
  if (!gWin) gWin = new VehiclePreviewWindow();
  return gWin;
}

void VehiclePreviewWindow::show_for(const QString &mesh_path,
                                   const QVector3D &fl, const QVector3D &fr,
                                   const QVector3D &rl, const QVector3D &rr)
{
  if (m_page) {
    m_page->load_mesh(mesh_path);
    m_page->set_wheel_positions_cm(fl, fr, rl, rr);
  }
  show();
  raise();
  activateWindow();
}

void VehiclePreviewWindow::show_for_mesh(const QString &mesh_path)
{
  if (m_page) m_page->load_mesh(mesh_path);
  show();
  raise();
  activateWindow();
}

}
