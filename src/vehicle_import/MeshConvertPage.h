// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
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
  using HandoffToImport = std::function<void(const QString &objPath)>;

  explicit MeshConvertPage(HandoffToImport handoff, QWidget *parent = nullptr);

 private:
  void onBrowseInput();
  void onConvert();
  void onCancel();
  void onProcessFinished(int exitCode);
  void appendLog(const QString &line);
  void setBlenderStatus();

  HandoffToImport  mHandoff;
  QLineEdit       *mInputEdit       = nullptr;
  QPushButton     *mBrowseBtn       = nullptr;
  QLabel          *mDetectedLabel   = nullptr;
  QLineEdit       *mOutputEdit      = nullptr;
  QLabel          *mBlenderLabel    = nullptr;
  QPushButton     *mConvertBtn      = nullptr;
  QPushButton     *mCancelBtn       = nullptr;
  QPushButton     *mHandoffBtn      = nullptr;
  QPlainTextEdit  *mLog             = nullptr;
  QProcess        *mProc            = nullptr;
  QString          mLastOutputPath;
};

}  // namespace carla_studio::vehicle_import
