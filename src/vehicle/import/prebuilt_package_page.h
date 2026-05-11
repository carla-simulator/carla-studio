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

class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QPushButton;

namespace carla_studio::vehicle_import {

class PrebuiltPackagePage : public QWidget {
  Q_OBJECT
 public:
  using CarlaRootResolver = std::function<QString()>;

  explicit PrebuiltPackagePage(CarlaRootResolver find_carla_root,
                               QWidget *parent = nullptr);

 public slots:
  void refresh_destination();

 private:
  void on_browse();
  void on_install();
  QString resolve_vehicles_dir() const;

  CarlaRootResolver m_find_carla_root;

  QLineEdit      *m_folder_edit  = nullptr;
  QPushButton    *m_browse_btn   = nullptr;
  QListWidget    *m_file_list    = nullptr;
  QLabel         *m_status_label = nullptr;
  QLineEdit      *m_name_edit    = nullptr;
  QLabel         *m_dst_label    = nullptr;
  QPushButton    *m_install_btn  = nullptr;
  QPlainTextEdit *m_log         = nullptr;
};

}
