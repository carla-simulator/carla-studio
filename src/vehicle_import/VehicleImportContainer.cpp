// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_import/VehicleImportContainer.h"

#include "vehicle_import/VehicleImportPage.h"

#include <QHBoxLayout>
#include <QTabBar>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

namespace carla_studio::vehicle_import {

VehicleImportContainer::VehicleImportContainer(EditorBinaryResolver findEditor,
                                               UprojectResolver     findUproject,
                                               CarlaRootResolver    findCarlaRoot,
                                               StartCarlaRequester  requestStartCarla,
                                               QWidget *parent)
    : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(6, 6, 6, 6);
  layout->setSpacing(0);

  mTabs = new QTabWidget(this);
  mTabs->setDocumentMode(true);
  mFromMesh = new VehicleImportPage(std::move(findEditor),
                                    std::move(findUproject),
                                    std::move(findCarlaRoot),
                                    std::move(requestStartCarla), this);
  mTabs->addTab(mFromMesh, "From 3D Model");
  mTabs->tabBar()->hide();   // single tab — no need for a tab bar
  layout->addWidget(mTabs);
}

void VehicleImportContainer::setSubTabCornerWidget(QWidget *w) {
  if (!mTabs || !w) return;
  auto *box = new QWidget(mTabs);
  auto *lay = new QHBoxLayout(box);
  lay->setContentsMargins(10, 0, 10, 6);
  lay->setSpacing(0);
  lay->addStretch();
  lay->addWidget(w, 0, Qt::AlignBottom | Qt::AlignHCenter);
  lay->addStretch();
  mTabs->setCornerWidget(box, Qt::TopRightCorner);
}

}  // namespace carla_studio::vehicle_import
