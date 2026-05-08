// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QString>
#include <QStringList>
#include <functional>

namespace carla_studio::vehicle_import {

struct PreflightOutcome {
  enum class Result { ReadyExisting, Failed };
  Result      result = Result::Failed;
  QString     reason;
  QString     editorBinary;
  QString     uprojectPath;
  bool        sourceWritten     = false;
  bool        moduleHookPatched = false;
};

using PreflightLogSink     = std::function<void(const QString &)>;
using PreflightProgressFn  = std::function<void(int /*pct*/, const QString &/*phase*/)>;

PreflightOutcome runImporterPreflight(const QString          &uprojectPath,
                                      const QString          &enginePath,
                                      const QString          &editorBinary,
                                      const PreflightLogSink &log,
                                      const PreflightProgressFn &progress);

}  // namespace carla_studio::vehicle_import
