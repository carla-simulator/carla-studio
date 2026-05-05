// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later —
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle_import/CarlaSpawn.h"

#include "vehicle_import/CookStep.h"

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

QString configFileFor(const QString &uprojectPath) {
  const QString dir = QFileInfo(uprojectPath).absolutePath();
  return dir + "/Content/Carla/Config/VehicleParameters.json";
}

QString classPathFor(const QString &bpAssetPath) {
  const QString name = bpAssetPath.section('/', -1);
  return bpAssetPath + "." + name + "_C";
}

QString contentSubpathForBp(const QString &bpAssetPath) {
  QString p = bpAssetPath;
  if (p.startsWith("/Game/")) p.remove(0, QString("/Game/").size());
  const int lastSlash = p.lastIndexOf('/');
  return lastSlash > 0 ? p.left(lastSlash) : p;
}

QString shippingContentRoot(const QString &shippingCarlaRoot) {
  for (const QString &mid : { QStringLiteral("CarlaUnreal"), QStringLiteral("CarlaUE5") }) {
    const QString candidate = shippingCarlaRoot + "/" + mid + "/Content";
    if (QFileInfo(candidate).isDir()) return candidate;
  }
  return shippingCarlaRoot + "/CarlaUnreal/Content";
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

}  // namespace

RegisterResult registerVehicleInJson(const VehicleRegistration &reg) {
  RegisterResult r;
  r.configFilePath = configFileFor(reg.uprojectPath);
  if (reg.uprojectPath.isEmpty() || reg.bpAssetPath.isEmpty()) {
    r.detail = "Missing uproject path or imported BP path.";
    return r;
  }
  if (!QFileInfo(r.configFilePath).isFile()) {
    r.detail = QString("VehicleParameters.json not found at %1").arg(r.configFilePath);
    return r;
  }
  r.ok = appendVehicleToConfig(r.configFilePath, classPathFor(reg.bpAssetPath),
                               reg.make, reg.model, reg.numberOfWheels,
                               &r.detail, &r.alreadyPresent);
  return r;
}

namespace {

bool ensureUSDImportTemplatesDeployed(const QString &dstContent,
                                      const QString &editorBinary,
                                      const QString &uprojectPath,
                                      QString *detail) {
  static constexpr const char *kTemplatesSub =
      "Carla/Blueprints/USDImportTemplates";
  static constexpr const char *kSentinel =
      "BaseUSDImportVehicle.uexp";

  const QString destDir = dstContent + "/" + kTemplatesSub;
  if (QFileInfo(destDir + "/" + kSentinel).isFile()) {
    return true;
  }
  if (editorBinary.isEmpty() || !QFileInfo(editorBinary).isExecutable()) {
    if (detail) *detail = "USDImportTemplates not deployed and no UE editor "
                          "binary available to cook them. Set CARLA_UNREAL_ENGINE_PATH.";
    return false;
  }
  CookRequest cr;
  cr.uprojectPath   = uprojectPath;
  cr.editorBinary   = editorBinary;
  cr.contentSubpath = kTemplatesSub;
  cr.vehicleName    = "USDImportTemplates";
  cr.timeoutMs      = 600000;
  const CookResult crr = cookVehicleAssets(cr);
  if (!crr.ok) {
    if (detail) *detail = QString("USDImportTemplates cook failed: %1").arg(crr.detail);
    return false;
  }

  QDir().mkpath(destDir);
  int copied = 0;
  QDirIterator it(crr.cookedRoot, { "*.uasset", "*.uexp", "*.ubulk" },
                  QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext()) {
    const QString s   = it.next();
    const QString rel = QDir(crr.cookedRoot).relativeFilePath(s);
    const QString t   = destDir + "/" + rel;
    QDir().mkpath(QFileInfo(t).absolutePath());
    QFile::remove(t);
    if (QFile::copy(s, t)) ++copied;
  }
  if (copied == 0) {
    if (detail) *detail = QString("USDImportTemplates cook produced files but copy to %1 "
                                  "yielded nothing.").arg(destDir);
    return false;
  }
  if (detail) *detail = QString("Deployed %1 USDImportTemplate file(s) to %2 (one-time).")
                          .arg(copied).arg(destDir);
  return true;
}

}  // namespace

DeployResult deployVehicleToShippingCarla(const VehicleRegistration &reg) {
  DeployResult d;
  if (reg.shippingCarlaRoot.isEmpty()) {
    d.detail = "Shipping CARLA root not configured (Cfg → CARLA root).";
    return d;
  }
  if (reg.uprojectPath.isEmpty() || reg.bpAssetPath.isEmpty()) {
    d.detail = "Missing uproject or BP path.";
    return d;
  }
  const QString srcContent = QFileInfo(reg.uprojectPath).absolutePath() + "/Content";
  const QString dstContent = shippingContentRoot(reg.shippingCarlaRoot);
  if (!QFileInfo(dstContent).isDir()) {
    d.detail = QString("Shipping content dir not found: %1").arg(dstContent);
    return d;
  }
  const QStringList purged = purgeStaleUncookedDeployments(dstContent);
  if (!purged.isEmpty())
    d.detail = QString("Purged %1 stale uncooked vehicle(s): %2. ")
                 .arg(purged.size()).arg(purged.join(", "));

  const QString sub = contentSubpathForBp(reg.bpAssetPath);
  const QString srcDir = srcContent + "/" + sub;
  const QString dstDir = dstContent + "/" + sub;
  d.destDir = dstDir;
  if (!QFileInfo(srcDir).isDir()) {
    d.detail = QString("Source asset dir not found in CARLA_src: %1 — run Import first and "
                       "make sure the editor is fresh enough to save .uasset to disk.").arg(srcDir);
    return d;
  }

  if (reg.editorBinary.isEmpty() || !QFileInfo(reg.editorBinary).isExecutable()) {
    d.detail += "Cannot deploy: no executable UE editor resolved (need CARLA_UNREAL_ENGINE_PATH "
                "or Setup Wizard configured). Cook is mandatory — shipping CARLA only loads "
                "cooked .uasset+.uexp pairs.";
    return d;
  }
  {
    QString templatesDetail;
    if (!ensureUSDImportTemplatesDeployed(dstContent, reg.editorBinary,
                                          reg.uprojectPath, &templatesDetail)) {
      d.detail += templatesDetail;
      return d;
    }
    if (!templatesDetail.isEmpty()) d.detail += templatesDetail + " ";
  }
  CookRequest cr;
  cr.uprojectPath   = reg.uprojectPath;
  cr.editorBinary   = reg.editorBinary;
  cr.contentSubpath = sub;
  cr.vehicleName    = reg.model;
  cr.timeoutMs      = 1200000;
  const CookResult crr = cookVehicleAssets(cr);
  if (!crr.ok) {
    d.detail += QString("Cook step failed: %1").arg(crr.detail);
    return d;
  }
  d.cookedFiles = crr.cookedFiles.size();
  const QString copyFromDir = crr.cookedRoot;

  if (QDir(dstDir).exists()) QDir(dstDir).removeRecursively();

  QDir().mkpath(dstDir);
  QDirIterator it(copyFromDir, { "*.uasset", "*.uexp", "*.ubulk", "*.umap" }, QDir::Files,
                  QDirIterator::Subdirectories);
  while (it.hasNext()) {
    const QString s = it.next();
    const QString rel = QDir(copyFromDir).relativeFilePath(s);
    const QString t = dstDir + "/" + rel;
    QDir().mkpath(QFileInfo(t).absolutePath());
    QFile::remove(t);
    if (QFile::copy(s, t)) ++d.filesCopied;
  }
  if (d.filesCopied == 0) {
    d.detail = QString("No .uasset files found under %1 — Import probably did not save to disk.").arg(srcDir);
    return d;
  }
  const QString bpName = QFileInfo(reg.bpAssetPath).fileName();
  const QString bpFile = dstDir + "/" + bpName + ".uasset";
  if (!QFileInfo(bpFile).isFile()) {
    d.detail = QString("Copied %1 file(s) but the BP itself (%2) is missing on disk — re-import "
                       "is required (the previous Import session ran with an older plugin that "
                       "didn't persist the BP).").arg(d.filesCopied).arg(bpName);
    return d;
  }
  d.configFilePath = dstContent + "/Carla/Config/VehicleParameters.json";
  if (!QFileInfo(d.configFilePath).isFile()) {
    d.detail = QString("Copied %1 file(s) to %2 but shipping VehicleParameters.json not found at %3.")
                 .arg(d.filesCopied).arg(dstDir).arg(d.configFilePath);
    return d;
  }
  bool already = false;
  QString jsonDetail;
  if (!appendVehicleToConfig(d.configFilePath, classPathFor(reg.bpAssetPath),
                             reg.make, reg.model, reg.numberOfWheels,
                             &jsonDetail, &already)) {
    d.detail = QString("Copied %1 file(s) but JSON update failed: %2").arg(d.filesCopied).arg(jsonDetail);
    return d;
  }
  d.ok = true;
  d.detail = QString("Copied %1 file(s) to shipping CARLA; JSON entry %2.")
               .arg(d.filesCopied).arg(already ? "was already present" : "appended");
  return d;
}

SpawnResult spawnInRunningCarla(const QString &make, const QString &model,
                                const QString &host, int port) {
  SpawnResult sr;
#ifndef CARLA_STUDIO_WITH_LIBCARLA
  (void)host; (void)port;
  sr.kind   = SpawnResult::Kind::Failed;
  sr.detail = "Studio was built without LibCarla.";
  return sr;
#else
  try {
    auto client = carla::client::Client(host.toStdString(), port);
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
        sr.detail = QString("Blueprint '%1' not in library; ReloadWorld threw: %2 — "
                            "restart CARLA via Studio's STOP+START so it re-reads "
                            "VehicleParameters.json.")
                      .arg(QString::fromStdString(id),
                           QString::fromUtf8(e.what()));
        return sr;
      } catch (...) {
        sr.kind   = SpawnResult::Kind::BlueprintNotFound;
        sr.detail = QString("Blueprint '%1' not in library; ReloadWorld threw a non-std "
                            "exception — restart CARLA via Studio's STOP+START.")
                      .arg(QString::fromStdString(id));
        return sr;
      }
      if (!reloadOk) {
        sr.kind   = SpawnResult::Kind::BlueprintNotFound;
        sr.detail = QString("Blueprint '%1' still not in library after ReloadWorld%2 — "
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
                          "this one doesn't — the imported vehicle's inherited PhysicsAsset "
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
                          "with SAE L5 autopilot enabled — Traffic Manager driving.")
                    .arg(moved).arg(actor->GetId());
    } else {
      sr.kind   = SpawnResult::Kind::Spawned;
      sr.detail = QString("Parked %1 existing vehicle(s) at (-1000,-1000); spawned actor id %2 "
                          "(not a vehicle — autopilot not applicable).")
                    .arg(moved).arg(actor->GetId());
    }
    return sr;
  } catch (const std::exception &e) {
    sr.kind   = SpawnResult::Kind::NoSimulator;
    sr.detail = QString("CARLA RPC unreachable on %1:%2 — start CARLA via Studio's "
                        "START button. (%3)").arg(host).arg(port).arg(QString::fromUtf8(e.what()));
    return sr;
  } catch (...) {
    sr.kind   = SpawnResult::Kind::Failed;
    sr.detail = "Unknown exception during LibCarla spawn — check that the running CARLA "
                "simulator's API version matches the libcarla-client this Studio was built "
                "against.";
    return sr;
  }
#endif
}

}  // namespace carla_studio::vehicle_import
