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
#include <functional>

class QComboBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QNetworkAccessManager;
class QPlainTextEdit;
class QProcess;
class QPushButton;
class QTabWidget;

namespace carla_studio::setup_wizard {

class SetupWizardDialog : public QDialog {
  Q_OBJECT
 public:
  using SetCarlaRoot = std::function<void(const QString &)>;

  explicit SetupWizardDialog(SetCarlaRoot set_carla_root,
                             int initial_tab = 1,
                             QWidget *parent = nullptr);

 private:
  QWidget *build_binary_tab();
  QWidget *build_source_tab();
  QWidget *build_maps_tab();

  void refresh_prereqs();
  void fetch_releases();
  void browse_clone_path();
  void browse_ue_path();
  void on_clone_repo();
  void on_clone_content();
  void on_build();
  void on_set_as_root();
  void on_copy_log();
  void append_log(const QString &line);

  SetCarlaRoot m_set_carla_root;

  QTabWidget    *m_tabs            = nullptr;

  QLineEdit     *m_clone_path       = nullptr;
  QComboBox     *m_branch_combo     = nullptr;
  QLabel        *m_git_dot          = nullptr;
  QLabel        *m_git_detail       = nullptr;
  QLabel        *m_lfs_dot          = nullptr;
  QLabel        *m_lfs_detail       = nullptr;
  QLabel        *m_ninja_dot        = nullptr;
  QLabel        *m_ninja_detail     = nullptr;
  QLabel        *m_cmake_dot        = nullptr;
  QLabel        *m_cmake_detail     = nullptr;
  QLineEdit     *m_ue_path          = nullptr;
  QLabel        *m_ue_ready_label    = nullptr;
  QPushButton   *m_install_prereq_btn = nullptr;
  QPushButton   *m_re_clone_btn      = nullptr;
  QPushButton   *m_clone_content_btn = nullptr;
  QPushButton   *m_build_btn        = nullptr;
  QPushButton   *m_set_as_root_btn    = nullptr;
  QPushButton   *m_copy_log_btn      = nullptr;
  QPlainTextEdit *m_log            = nullptr;

  QListWidget          *m_maps_list        = nullptr;
  QPushButton          *m_maps_refresh_btn  = nullptr;
  QPushButton          *m_maps_install_btn  = nullptr;
  QLineEdit            *m_maps_target_root  = nullptr;
  QPlainTextEdit       *m_maps_log        = nullptr;
  QNetworkAccessManager *m_nam            = nullptr;
};

}
