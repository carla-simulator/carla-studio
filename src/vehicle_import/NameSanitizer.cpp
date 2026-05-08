// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_import/NameSanitizer.h"

#include <QChar>

namespace carla_studio::vehicle_import {

QString sanitizeVehicleName(const QString &raw) {
  QString out;
  out.reserve(raw.size());
  for (QChar c : raw) {
    const ushort u = c.unicode();
    if ((u >= 'a' && u <= 'z') || (u >= '0' && u <= '9')) {
      out.append(c);
    } else if (u >= 'A' && u <= 'Z') {
      out.append(QChar(u + ('a' - 'A')));
    } else {
      if (!out.isEmpty() && out.back() != QLatin1Char('_'))
        out.append(QLatin1Char('_'));
    }
  }
  while (out.endsWith(QLatin1Char('_'))) out.chop(1);
  while (out.startsWith(QLatin1Char('_'))) out.remove(0, 1);
  if (!out.isEmpty() && out.front().isDigit()) out.prepend(QStringLiteral("v_"));
  if (out.size() > kMaxVehicleNameLength) out.truncate(kMaxVehicleNameLength);
  while (out.endsWith(QLatin1Char('_'))) out.chop(1);
  return out;
}

bool isCanonicalVehicleName(const QString &name) {
  if (name.isEmpty() || name.size() > kMaxVehicleNameLength) return false;
  return sanitizeVehicleName(name) == name;
}

VehicleNameValidator::VehicleNameValidator(QObject *parent)
    : QValidator(parent) {}

QValidator::State VehicleNameValidator::validate(QString &input, int &) const {
  if (input.isEmpty()) return Intermediate;
  if (sanitizeVehicleName(input) == input) return Acceptable;
  return Intermediate;
}

void VehicleNameValidator::fixup(QString &input) const {
  input = sanitizeVehicleName(input);
}

}  // namespace carla_studio::vehicle_import
