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
class QPlainTextEdit;
class QProcess;
class QPushButton;

namespace carla_studio::vehicle_import {

class MeshConvertPage : public QWidget {
  Q_OBJECT
 public:
  using HandoffToImport = std::function<void(const QString &obj_path)>;

  explicit MeshConvertPage(HandoffToImport handoff, QWidget *parent = nullptr);

 private:
  void on_browse_input();
  void on_convert();
  void on_cancel();
  void on_process_finished(int exit_code);
  void append_log(const QString &line);
  void set_blender_status();

  HandoffToImport  m_handoff;
  QLineEdit       *m_input_edit       = nullptr;
  QPushButton     *m_browse_btn       = nullptr;
  QLabel          *m_detected_label   = nullptr;
  QLineEdit       *m_output_edit      = nullptr;
  QLabel          *m_blender_label    = nullptr;
  QPushButton     *m_convert_btn      = nullptr;
  QPushButton     *m_cancel_btn       = nullptr;
  QPushButton     *m_handoff_btn      = nullptr;
  QPlainTextEdit  *m_log             = nullptr;
  QProcess        *m_proc            = nullptr;
  QString          m_last_output_path;
};

}
