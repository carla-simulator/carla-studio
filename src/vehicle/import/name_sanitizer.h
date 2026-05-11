// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QString>
#include <QValidator>

namespace carla_studio::vehicle_import {

constexpr int kMaxVehicleNameLength = 60;

QString sanitize_vehicle_name(const QString &raw);

bool is_canonical_vehicle_name(const QString &name);

class VehicleNameValidator : public QValidator {
 public:
  explicit VehicleNameValidator(QObject *parent = nullptr);
  State validate(QString &input, int &pos) const override;
  void  fixup(QString &input) const override;
};

}
