// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/vehicle_import_container.h"

#include "vehicle/import/vehicle_import_page.h"

#include <QHBoxLayout>
#include <QTabBar>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

namespace carla_studio::vehicle_import {

VehicleImportContainer::VehicleImportContainer(EditorBinaryResolver find_editor,
                                               UprojectResolver     find_uproject,
                                               CarlaRootResolver    find_carla_root,
                                               StartCarlaRequester  request_start_carla,
                                               QWidget *parent)
    : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(6, 6, 6, 6);
  layout->setSpacing(0);

  m_tabs = new QTabWidget(this);
  m_tabs->setDocumentMode(true);
  m_from_mesh = new VehicleImportPage(std::move(find_editor),
                                    std::move(find_uproject),
                                    std::move(find_carla_root),
                                    std::move(request_start_carla), this);
  m_tabs->addTab(m_from_mesh, "From 3D Model");
  m_tabs->tabBar()->hide();
  layout->addWidget(m_tabs);
}

void VehicleImportContainer::set_sub_tab_corner_widget(QWidget *w) {
  if (!m_tabs || !w) return;
  auto *box = new QWidget(m_tabs);
  auto *lay = new QHBoxLayout(box);
  lay->setContentsMargins(10, 0, 10, 6);
  lay->setSpacing(0);
  lay->addStretch();
  lay->addWidget(w, 0, Qt::AlignBottom | Qt::AlignHCenter);
  lay->addStretch();
  m_tabs->setCornerWidget(box, Qt::TopRightCorner);
}

}
