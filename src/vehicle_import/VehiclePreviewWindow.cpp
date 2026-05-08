// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "VehiclePreviewWindow.h"
#include "VehiclePreviewPage.h"

#include <QVBoxLayout>

namespace carla_studio::vehicle_import {

VehiclePreviewWindow::VehiclePreviewWindow(QWidget *parent) : QDialog(parent)
{
  setWindowTitle("CARLA Studio — Vehicle Preview");
  setModal(false);
  setSizeGripEnabled(true);
  resize(1024, 720);

  auto *lay = new QVBoxLayout(this);
  lay->setContentsMargins(6, 6, 6, 6);
  mPage = new VehiclePreviewPage(this);
  lay->addWidget(mPage);
}

VehiclePreviewWindow *VehiclePreviewWindow::instance()
{
  static VehiclePreviewWindow *gWin = nullptr;
  if (!gWin) gWin = new VehiclePreviewWindow();
  return gWin;
}

void VehiclePreviewWindow::showFor(const QString &meshPath,
                                   const QVector3D &fl, const QVector3D &fr,
                                   const QVector3D &rl, const QVector3D &rr)
{
  if (mPage) {
    mPage->loadMesh(meshPath);
    mPage->setWheelPositionsCm(fl, fr, rl, rr);
  }
  show();
  raise();
  activateWindow();
}

void VehiclePreviewWindow::showForMesh(const QString &meshPath)
{
  if (mPage) mPage->loadMesh(meshPath);
  show();
  raise();
  activateWindow();
}

}  // namespace carla_studio::vehicle_import
