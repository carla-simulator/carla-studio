// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_import/SpecApplicator.h"

namespace carla_studio::vehicle_import {

ApplyResult applySpecToEditor(const VehicleSpec &spec,
                              const QString & /*editorRpcEndpoint*/) {
  ApplyResult r;
  r.bpAssetPath = QStringLiteral("/Game/Carla/Static/Vehicles/4Wheeled/")
                + spec.name + "/BP_" + spec.name;
  r.detail = QStringLiteral(
      "SpecApplicator is the contract for the slim UE plugin. The plugin's job is "
      "deterministic: import the canonical FBX, modify the inherited PhysicsAsset "
      "(chassis Box Center.Z above wheel-bottom plane, Linear Damping = 0, wheel "
      "primitives Sphere + Kinematic + collision Disabled), build 4 Wheel BPs with "
      "Shape Radius from spec, build the Vehicle BP with size-class CoM offset and "
      "torque-curve preset, and SaveAsset the lot. No discovery, no inference, no "
      "user prompts — every value comes from `spec`. To be wired into "
      "CarlaTools/Source/CarlaTools/Private/VehicleImporter.cpp Slice 2.");
  return r;
}

ValidationReport validateImportedVehicle(const QString &bpAssetPath,
                                         const QString & /*editorRpcEndpoint*/) {
  ValidationReport v;
  v.detail = QStringLiteral(
      "Post-import validation pass — Slice 4 contract. The UE plugin should: "
      "(a) spawn `") + bpAssetPath + QStringLiteral("_C` as a transient actor in the editor world; "
      "(b) place it at z = wheel_bottom_z; "
      "(c) tick physics for one frame; "
      "(d) assert chassis Box does not overlap the ground plane; "
      "(e) assert all 4 wheel spheres touch the ground; "
      "(f) attach a collision sensor and confirm it fires on contact; "
      "(g) destroy the actor. If any check fails, abort the import with a "
      "structured diagnostic — never ship a BP that fails validation.");
  return v;
}

}  // namespace carla_studio::vehicle_import
