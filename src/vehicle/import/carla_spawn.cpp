// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/carla_spawn.h"

#include "vehicle/import/cook_step.h"

#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QSaveFile>

#ifdef CARLA_STUDIO_WITH_LIBCARLA
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Actor.h>
#include <carla/client/ActorList.h>
#include <carla/geom/Transform.h>
#include <chrono>
#include <thread>
#include <stdexcept>
#endif

namespace carla_studio::vehicle_import {

namespace {

QString configFileFor(const QString &uproject_path) {
  const QString dir = QFileInfo(uproject_path).absolutePath();
  return dir + "/Content/Carla/Config/VehicleParameters.json";
}

QString classPathFor(const QString &bp_asset_path) {
  const QString name = bp_asset_path.section('/', -1);
  return bp_asset_path + "." + name + "_C";
}

QString contentSubpathForBp(const QString &bp_asset_path) {
  QString p = bp_asset_path;
  if (p.startsWith("/Game/")) p.remove(0, QString("/Game/").size());
  const qsizetype lastSlash = p.lastIndexOf('/');
  return lastSlash > 0 ? p.left(lastSlash) : p;
}

QString shippingContentRoot(const QString &shipping_carla_root) {
  for (const QString &mid : { QStringLiteral("CarlaUnreal"), QStringLiteral("CarlaUE5") }) {
    const QString candidate = shipping_carla_root + "/" + mid + "/Content";
    if (QFileInfo(candidate).isDir()) return candidate;
  }
  return shipping_carla_root + "/CarlaUnreal/Content";
}

bool readVehiclesJson(const QString &configPath, QJsonObject &rootOut, QJsonArray &arrOut,
                      QString *detailOut) {
  QFile in(configPath);
  if (!in.open(QIODevice::ReadOnly)) {
    if (detailOut) *detailOut = QString("Cannot read %1").arg(configPath);
    return false;
  }
  const QByteArray bytes = in.readAll();
  in.close();
  QJsonParseError perr;
  QJsonDocument doc = QJsonDocument::fromJson(bytes, &perr);
  if (perr.error != QJsonParseError::NoError || !doc.isObject()) {
    if (detailOut) *detailOut = QString("Could not parse JSON: %1").arg(perr.errorString());
    return false;
  }
  rootOut = doc.object();
  arrOut  = rootOut.value("Vehicles").toArray();
  return true;
}

bool writeVehiclesJson(const QString &configPath, const QJsonObject &root, QString *detailOut) {
  QSaveFile out(configPath);
  if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    if (detailOut) *detailOut = QString("Cannot write %1").arg(configPath);
    return false;
  }
  out.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
  if (!out.commit()) {
    if (detailOut) *detailOut = QString("Failed to commit %1").arg(configPath);
    return false;
  }
  return true;
}

bool appendVehicleToConfig(const QString &configPath, const QString &classPath,
                           const QString &make, const QString &model,
                           int numWheels, QString *detailOut, bool *replacedOut) {
  QJsonObject root;
  QJsonArray vehicles;
  if (!readVehiclesJson(configPath, root, vehicles, detailOut)) return false;

  QJsonArray kept;
  bool replaced = false;
  for (const QJsonValue &v : vehicles) {
    const QJsonObject o = v.toObject();
    auto get = [&](const char *upper, const char *lower) {
      const QString u = o.value(upper).toString();
      return u.isEmpty() ? o.value(lower).toString() : u;
    };
    if (get("Model","model") == model || get("Class","class") == classPath) {
      replaced = true;
      continue;
    }
    kept.append(o);
  }

  QJsonObject entry;
  entry["Make"]              = make.toLower();
  entry["Model"]             = model;
  entry["Class"]             = classPath;
  entry["NumberOfWheels"]    = numWheels;
  entry["Generation"]        = 2;
  entry["BaseType"]          = "car";
  entry["HasDynamicDoors"]   = false;
  entry["HasLights"]         = true;
  entry["RecommendedColors"] = QJsonArray();
  entry["SupportedDrivers"]  = QJsonArray();
  kept.append(entry);
  root["Vehicles"] = kept;

  if (!writeVehiclesJson(configPath, root, detailOut)) return false;
  if (replacedOut) *replacedOut = replaced;
  if (detailOut) *detailOut = replaced ? "Existing entry replaced." : "Registration entry appended.";
  return true;
}

QStringList purgeStaleUncookedDeployments(const QString &shippingContent) {
  QStringList removed;
  const QString vehiclesRoot = shippingContent + "/Carla/Static/Vehicles/4Wheeled";
  QDir root(vehiclesRoot);
  if (!root.exists()) return removed;
  const QStringList dirs = root.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
  for (const QString &name : dirs) {
    const QString dirPath = vehiclesRoot + "/" + name;
    QDirIterator it(dirPath, { "*.uasset" }, QDir::Files);
    bool sawAny = false;
    bool allHaveExp = true;
    while (it.hasNext()) {
      const QString uasset = it.next();
      sawAny = true;
      const QString sibling = uasset.left(uasset.size() - 7) + ".uexp";
      if (!QFileInfo(sibling).isFile()) { allHaveExp = false; break; }
    }
    if (sawAny && !allHaveExp) {
      QDir(dirPath).removeRecursively();
      removed << name;
    }
  }
  if (removed.isEmpty()) return removed;

  const QString configPath = shippingContent + "/Carla/Config/VehicleParameters.json";
  if (!QFileInfo(configPath).isFile()) return removed;
  QJsonObject jroot;
  QJsonArray vehicles;
  if (!readVehiclesJson(configPath, jroot, vehicles, nullptr)) return removed;
  QJsonArray kept;
  for (const QJsonValue &v : vehicles) {
    const QString model = v.toObject().value("Model").toString();
    if (removed.contains(model, Qt::CaseInsensitive)) continue;
    kept.append(v);
  }
  jroot["Vehicles"] = kept;
  writeVehiclesJson(configPath, jroot, nullptr);
  return removed;
}

}

RegisterResult register_vehicle_in_json(const VehicleRegistration &reg) {
  RegisterResult r;
  r.config_file_path = configFileFor(reg.uproject_path);
  if (reg.uproject_path.isEmpty() || reg.bp_asset_path.isEmpty()) {
    r.detail = "Missing uproject path or imported BP path.";
    return r;
  }
  if (!QFileInfo(r.config_file_path).isFile()) {
    r.detail = QString("VehicleParameters.json not found at %1").arg(r.config_file_path);
    return r;
  }
  r.ok = appendVehicleToConfig(r.config_file_path, classPathFor(reg.bp_asset_path),
                               reg.make, reg.model, reg.number_of_wheels,
                               &r.detail, &r.already_present);
  return r;
}

namespace {

bool ensureUSDImportTemplatesDeployed(const QString &dstContent,
                                      const QString &editor_binary,
                                      const QString &uproject_path,
                                      QString *detail) {
  static constexpr const char *kTemplatesSub =
      "Carla/Blueprints/USDImportTemplates";
  static constexpr const char *kSentinel =
      "BaseUSDImportVehicle.uexp";

  const QString dest_dir = dstContent + "/" + kTemplatesSub;
  if (QFileInfo(dest_dir + "/" + kSentinel).isFile()) {
    return true;
  }
  if (editor_binary.isEmpty() || !QFileInfo(editor_binary).isExecutable()) {
    if (detail) *detail = "USDImportTemplates not deployed and no UE editor "
                          "binary available to cook them. Set CARLA_UNREAL_ENGINE_PATH.";
    return false;
  }
  CookRequest cr;
  cr.uproject_path   = uproject_path;
  cr.editor_binary   = editor_binary;
  cr.content_subpath = kTemplatesSub;
  cr.vehicle_name    = "USDImportTemplates";
  cr.timeout_ms      = 600000;
  const CookResult crr = cook_vehicle_assets(cr);
  if (!crr.ok) {
    if (detail) *detail = QString("USDImportTemplates cook failed: %1").arg(crr.detail);
    return false;
  }

  QDir().mkpath(dest_dir);
  int copied = 0;
  QDirIterator it(crr.cooked_root, { "*.uasset", "*.uexp", "*.ubulk" },
                  QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext()) {
    const QString s   = it.next();
    const QString rel = QDir(crr.cooked_root).relativeFilePath(s);
    const QString t   = dest_dir + "/" + rel;
    QDir().mkpath(QFileInfo(t).absolutePath());
    QFile::remove(t);
    if (QFile::copy(s, t)) ++copied;
  }
  if (copied == 0) {
    if (detail) *detail = QString("USDImportTemplates cook produced files but copy to %1 "
                                  "yielded nothing.").arg(dest_dir);
    return false;
  }
  if (detail) *detail = QString("Deployed %1 USDImportTemplate file(s) to %2 (one-time).")
                          .arg(copied).arg(dest_dir);
  return true;
}

}

DeployResult deploy_vehicle_to_shipping_carla(const VehicleRegistration &reg) {
  DeployResult d;
  if (reg.shipping_carla_root.isEmpty()) {
    d.detail = "Shipping CARLA root not configured (Cfg → CARLA root).";
    return d;
  }
  if (reg.uproject_path.isEmpty() || reg.bp_asset_path.isEmpty()) {
    d.detail = "Missing uproject or BP path.";
    return d;
  }
  const QString srcContent = QFileInfo(reg.uproject_path).absolutePath() + "/Content";
  const QString dstContent = shippingContentRoot(reg.shipping_carla_root);
  if (!QFileInfo(dstContent).isDir()) {
    d.detail = QString("Shipping content dir not found: %1").arg(dstContent);
    return d;
  }
  const QStringList purged = purgeStaleUncookedDeployments(dstContent);
  if (!purged.isEmpty())
    d.detail = QString("Purged %1 stale uncooked vehicle(s): %2. ")
                 .arg(purged.size()).arg(purged.join(", "));

  const QString sub = contentSubpathForBp(reg.bp_asset_path);
  const QString srcDir = srcContent + "/" + sub;
  const QString dstDir = dstContent + "/" + sub;
  d.dest_dir = dstDir;
  if (!QFileInfo(srcDir).isDir()) {
    d.detail = QString("Source asset dir not found in CARLA_src: %1 - run Import first and "
                       "make sure the editor is fresh enough to save .uasset to disk.").arg(srcDir);
    return d;
  }

  if (reg.editor_binary.isEmpty() || !QFileInfo(reg.editor_binary).isExecutable()) {
    d.detail += "Cannot deploy: no executable UE editor resolved (need CARLA_UNREAL_ENGINE_PATH "
                "or Setup Wizard configured). Cook is mandatory - shipping CARLA only loads "
                "cooked .uasset+.uexp pairs.";
    return d;
  }
  {
    QString templatesDetail;
    if (!ensureUSDImportTemplatesDeployed(dstContent, reg.editor_binary,
                                          reg.uproject_path, &templatesDetail)) {
      d.detail += templatesDetail;
      return d;
    }
    if (!templatesDetail.isEmpty()) d.detail += templatesDetail + " ";
  }
  CookRequest cr;
  cr.uproject_path   = reg.uproject_path;
  cr.editor_binary   = reg.editor_binary;
  cr.content_subpath = sub;
  cr.vehicle_name    = reg.model;
  cr.timeout_ms      = 1200000;
  const CookResult crr = cook_vehicle_assets(cr);
  if (!crr.ok) {
    d.detail += QString("Cook step failed: %1").arg(crr.detail);
    return d;
  }
  d.cooked_files = static_cast<int>(crr.cooked_files.size());
  const QString copyFromDir = crr.cooked_root;

  if (QDir(dstDir).exists()) QDir(dstDir).removeRecursively();
  QDir().mkpath(dstDir);

  QString cookedContentRoot = copyFromDir;
  {
    const QString needle = QStringLiteral("/Content/");
    const qsizetype idx = cookedContentRoot.lastIndexOf(needle);
    if (idx >= 0)
      cookedContentRoot = cookedContentRoot.left(idx + needle.size() - 1);
  }
  const QString vehicleRel = sub;
  QDirIterator it(cookedContentRoot,
                  { "*.uasset", "*.uexp", "*.ubulk", "*.umap" },
                  QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext()) {
    const QString s = it.next();
    const QString rel = QDir(cookedContentRoot).relativeFilePath(s);
    const QString t = dstContent + "/" + rel;
    const bool inVehicleDir = rel.startsWith(vehicleRel + QLatin1Char('/'))
                             || rel == vehicleRel;
    if (!inVehicleDir && QFileInfo::exists(t)) continue;
    QDir().mkpath(QFileInfo(t).absolutePath());
    if (inVehicleDir) QFile::remove(t);
    if (QFile::copy(s, t)) ++d.files_copied;
  }
  if (d.files_copied == 0) {
    d.detail = QString("No .uasset files found under %1 - Import probably did not save to disk.").arg(srcDir);
    return d;
  }
  const QString bpName = QFileInfo(reg.bp_asset_path).fileName();
  const QString bpFile = dstDir + "/" + bpName + ".uasset";
  if (!QFileInfo(bpFile).isFile()) {
    d.detail = QString("Copied %1 file(s) but the BP itself (%2) is missing on disk - re-import "
                       "is required (the previous Import session ran with an older plugin that "
                       "didn't persist the BP).").arg(d.files_copied).arg(bpName);
    return d;
  }
  d.config_file_path = dstContent + "/Carla/Config/VehicleParameters.json";
  if (!QFileInfo(d.config_file_path).isFile()) {
    d.detail = QString("Copied %1 file(s) to %2 but shipping VehicleParameters.json not found at %3.")
                 .arg(d.files_copied).arg(dstDir).arg(d.config_file_path);
    return d;
  }
  bool already = false;
  QString jsonDetail;
  if (!appendVehicleToConfig(d.config_file_path, classPathFor(reg.bp_asset_path),
                             reg.make, reg.model, reg.number_of_wheels,
                             &jsonDetail, &already)) {
    d.detail = QString("Copied %1 file(s) but JSON update failed: %2").arg(d.files_copied).arg(jsonDetail);
    return d;
  }
  d.ok = true;
  d.detail = QString("Copied %1 file(s) to shipping CARLA; JSON entry %2.")
               .arg(d.files_copied).arg(already ? "was already present" : "appended");
  return d;
}

SpawnResult spawn_in_running_carla(const QString &make, const QString &model,
                                const QString &host, int port) {
  SpawnResult sr;
#ifndef CARLA_STUDIO_WITH_LIBCARLA
  (void)make; (void)model; (void)host; (void)port;
  sr.kind   = SpawnResult::Kind::Failed;
  sr.detail = "Studio was built without LibCarla.";
  return sr;
#else
  try {
    auto client = carla::client::Client(host.toStdString(),
                                        static_cast<uint16_t>(port));
    client.SetTimeout(std::chrono::seconds(60));
    client.GetServerVersion();
    auto world  = client.GetWorld();
    auto bps    = world.GetBlueprintLibrary();
    const std::string id = QString("vehicle.%1.%2")
                              .arg(make.toLower(), model.toLower()).toStdString();
    auto bp = bps->Find(id);
    if (!bp) {
      bool reloadAttempted = false;
      bool reloadOk        = false;
      try {
        client.ReloadWorld();
        reloadAttempted = true;
        for (int tries = 0; tries < 6; ++tries) {
          try {
            world = client.GetWorld();
            bps   = world.GetBlueprintLibrary();
            bp    = bps->Find(id);
            if (bp) { reloadOk = true; break; }
          } catch (...) {
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        }
      } catch (const std::exception &e) {
        sr.kind   = SpawnResult::Kind::BlueprintNotFound;
        sr.detail = QString("Blueprint '%1' not in library; ReloadWorld threw: %2 - "
                            "restart CARLA via Studio's STOP+START so it re-reads "
                            "VehicleParameters.json.")
                      .arg(QString::fromStdString(id),
                           QString::fromUtf8(e.what()));
        return sr;
      } catch (...) {
        sr.kind   = SpawnResult::Kind::BlueprintNotFound;
        sr.detail = QString("Blueprint '%1' not in library; ReloadWorld threw a non-std "
                            "exception - restart CARLA via Studio's STOP+START.")
                      .arg(QString::fromStdString(id));
        return sr;
      }
      if (!reloadOk) {
        sr.kind   = SpawnResult::Kind::BlueprintNotFound;
        sr.detail = QString("Blueprint '%1' still not in library after ReloadWorld%2 - "
                            "restart CARLA via Studio's STOP+START.")
                      .arg(QString::fromStdString(id),
                           reloadAttempted ? " + 6 retries" : "");
        return sr;
      }
    }
    auto map        = world.GetMap();
    auto transforms = map->GetRecommendedSpawnPoints();
    if (transforms.empty()) {
      sr.kind   = SpawnResult::Kind::Failed;
      sr.detail = "Map exposes no recommended spawn points.";
      return sr;
    }

    int moved = 0;
    auto allActors = world.GetActors();
    const carla::geom::Transform parking(
        carla::geom::Location(-1000.0f, -1000.0f, -1000.0f),
        carla::geom::Rotation());
    for (size_t i = 0; i < allActors->size(); ++i) {
      auto a = allActors->at(i);
      if (!a) continue;
      const std::string tid = a->GetTypeId();
      if (tid.rfind("vehicle.", 0) == 0) {
        if (auto v = std::dynamic_pointer_cast<carla::client::Vehicle>(a)) {
          v->SetAutopilot(false);
        }
        a->SetSimulatePhysics(false);
        a->SetTransform(parking);
        ++moved;
      }
    }
    decltype(world.TrySpawnActor(*bp, transforms.front())) actor;
    int tried = 0;
    for (const auto &t : transforms) {
      ++tried;
      actor = world.TrySpawnActor(*bp, t);
      if (actor) break;
    }
    if (!actor) {
      for (const auto &t : transforms) {
        ++tried;
        carla::geom::Transform lifted(
            carla::geom::Location(t.location.x, t.location.y, t.location.z + 100.0f),
            t.rotation);
        actor = world.TrySpawnActor(*bp, lifted);
        if (actor) break;
      }
    }
    if (!actor) {
      sr.kind   = SpawnResult::Kind::Failed;
      sr.detail = QString("TrySpawnActor returned null at all %1 recommended spawn point(s) "
                          "(both ground level and +1 m clearance). Stock vehicles spawn here, "
                          "this one doesn't - the imported vehicle's inherited PhysicsAsset "
                          "(from BaseUSDImportVehicle) has a chassis Box sized for a sedan and "
                          "rejects collision at every spawn point that fits the imported mesh. "
                          "Fix: resize the chassis Box body in the PhysicsAsset to match the "
                          "cooked mesh bbox (slice 2 / SpecApplicator).")
                  .arg(tried);
      return sr;
    }
    if (auto vehicle = std::dynamic_pointer_cast<carla::client::Vehicle>(actor)) {
      vehicle->SetAutopilot(true);
      sr.kind   = SpawnResult::Kind::Spawned;
      sr.detail = QString("Parked %1 existing vehicle(s) at (-1000,-1000); spawned actor id %2 "
                          "with SAE L5 autopilot enabled - Traffic Manager driving.")
                    .arg(moved).arg(actor->GetId());
    } else {
      sr.kind   = SpawnResult::Kind::Spawned;
      sr.detail = QString("Parked %1 existing vehicle(s) at (-1000,-1000); spawned actor id %2 "
                          "(not a vehicle - autopilot not applicable).")
                    .arg(moved).arg(actor->GetId());
    }
    return sr;
  } catch (const std::exception &e) {
    sr.kind   = SpawnResult::Kind::NoSimulator;
    sr.detail = QString("CARLA RPC unreachable on %1:%2 - start CARLA via Studio's "
                        "START button. (%3)").arg(host).arg(port).arg(QString::fromUtf8(e.what()));
    return sr;
  } catch (...) {
    sr.kind   = SpawnResult::Kind::Failed;
    sr.detail = "Unknown exception during LibCarla spawn - check that the running CARLA "
                "simulator's API version matches the libcarla-client this Studio was built "
                "against.";
    return sr;
  }
#endif
}

DriveTestResult drive_test_vehicle(const QString &make, const QString &model,
                                  const QString &host, int port) {
  DriveTestResult r;
#ifndef CARLA_STUDIO_WITH_LIBCARLA
  (void)make; (void)model; (void)host; (void)port;
  r.detail = "Studio built without LibCarla.";
  return r;
#else
  try {
    auto client = carla::client::Client(host.toStdString(),
                                        static_cast<uint16_t>(port));
    client.SetTimeout(std::chrono::seconds(60));
    client.GetServerVersion();
    auto world = client.GetWorld();
    auto bps   = world.GetBlueprintLibrary();
    const std::string id = QString("vehicle.%1.%2")
                             .arg(make.toLower(), model.toLower()).toStdString();
    auto bp = bps->Find(id);
    if (!bp) {
      r.detail = QString("Blueprint 'vehicle.%1.%2' not in library - run Drive first.")
                   .arg(make.toLower(), model.toLower());
      return r;
    }
    auto map = world.GetMap();
    auto transforms = map->GetRecommendedSpawnPoints();
    if (transforms.empty()) {
      r.detail = "Map has no recommended spawn points.";
      return r;
    }
    decltype(world.TrySpawnActor(*bp, transforms.front())) actor;
    for (const auto &t : transforms) {
      actor = world.TrySpawnActor(*bp, t);
      if (actor) break;
    }
    if (!actor) {
      r.detail = "TrySpawnActor returned null - chassis collision blocks all spawn points.";
      return r;
    }
    r.spawned = true;

    auto vehicle = std::dynamic_pointer_cast<carla::client::Vehicle>(actor);
    if (!vehicle) {
      r.detail = "Spawned actor is not a Vehicle.";
      actor->Destroy();
      return r;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    const carla::geom::Location initLoc = vehicle->GetLocation();

    carla::client::Vehicle::Control ctrl;
    ctrl.throttle       = 1.0f;
    ctrl.steer          = 0.0f;
    ctrl.hand_brake     = false;
    ctrl.reverse        = false;
    ctrl.manual_gear_shift = false;
    ctrl.gear           = 0;
    for (int i = 0; i < 10; ++i) {
      vehicle->ApplyControl(ctrl);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    const carla::geom::Location finalLoc = vehicle->GetLocation();
    const float dx = finalLoc.x - initLoc.x;
    const float dy = finalLoc.y - initLoc.y;
    r.displacement_m = std::sqrt(dx * dx + dy * dy) / 100.f;
    r.drives_forward = r.displacement_m > 1.f;

    const carla::geom::BoundingBox bbox = vehicle->GetBoundingBox();
    r.bbox_extent_x = bbox.extent.x;
    r.bbox_extent_y = bbox.extent.y;
    r.bbox_extent_z = bbox.extent.z;
    constexpr float kMustangX = 220.f, kMustangY = 90.f, kMustangZ = 65.f;
    constexpr float kTol = 0.25f;
    r.bbox_is_custom = (std::abs(r.bbox_extent_x - kMustangX) / kMustangX > kTol)
                  || (std::abs(r.bbox_extent_y - kMustangY) / kMustangY > kTol)
                  || (std::abs(r.bbox_extent_z - kMustangZ) / kMustangZ > kTol);

    try {
      r.destroy_clean = vehicle->Destroy();
    } catch (...) {
      r.destroy_clean = false;
    }

    r.detail = QString("disp:%1m  bbox:%2x%3x%4cm  drives:%5  custom_bbox:%6  destroy:%7")
                 .arg(r.displacement_m, 0, 'f', 2)
                 .arg(r.bbox_extent_x * 2.f, 0, 'f', 0)
                 .arg(r.bbox_extent_y * 2.f, 0, 'f', 0)
                 .arg(r.bbox_extent_z * 2.f, 0, 'f', 0)
                 .arg(r.drives_forward ? "YES" : "NO")
                 .arg(r.bbox_is_custom  ? "YES" : "NO")
                 .arg(r.destroy_clean  ? "OK"  : "FAIL");
    return r;
  } catch (const std::exception &e) {
    r.detail = QString("RPC error: %1").arg(QString::fromUtf8(e.what()));
    return r;
  } catch (...) {
    r.detail = "Unknown exception during drive test.";
    return r;
  }
#endif
}

}
