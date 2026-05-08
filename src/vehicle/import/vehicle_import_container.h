// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QString>
#include <QWidget>
#include <functional>

class QTabWidget;

namespace carla_studio::vehicle_import {

class VehicleImportPage;

class VehicleImportContainer : public QWidget {
  Q_OBJECT
 public:
  using EditorBinaryResolver = std::function<QString()>;
  using UprojectResolver     = std::function<QString()>;
  using CarlaRootResolver    = std::function<QString()>;
  using StartCarlaRequester  = std::function<void()>;

  explicit VehicleImportContainer(EditorBinaryResolver find_editor,
                                  UprojectResolver     find_uproject,
                                  CarlaRootResolver    find_carla_root,
                                  StartCarlaRequester  request_start_carla,
                                  QWidget *parent = nullptr);

  void set_sub_tab_corner_widget(QWidget *w);

 private:
  QTabWidget          *m_tabs     = nullptr;
  VehicleImportPage   *m_from_mesh = nullptr;
};

}
