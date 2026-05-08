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

#include <cstdint>
#include <optional>
#include <vector>

namespace carla_studio::utils {

namespace detail {

QString run_cmd(const QString &cmd, const QStringList &args, int timeout_ms = 3000);
QString read_file(const QString &path);
QString detect_os_pretty();
void detect_cpu(QString *model, int *logical, int *physical);
void detect_ram(std::int64_t *total, std::int64_t *free);

}

struct GpuInfo {
  QString name;
  std::int64_t vram_bytes = 0;
  QString cuda_capability;
  QString driver_version;
  bool is_nvidia = false;
};

struct VolumeInfo {
  QString mount_point;
  QString label;
  QString file_system;
  std::int64_t total_bytes = 0;
  std::int64_t free_bytes = 0;
  bool is_read_only = false;
  bool is_external = false;
};

class SystemProbe {
public:
  SystemProbe();

  const QString &os_pretty() const { return os_pretty_; }
  const QString &kernel() const { return kernel_; }

  const QString &cpu_model() const { return cpu_model_; }
  int cpu_logical_cores() const { return cpu_logical_cores_; }
  int cpu_physical_cores() const { return cpu_physical_cores_; }

  std::int64_t ram_total_bytes() const { return ram_total_bytes_; }
  std::int64_t ram_free_bytes() const { return ram_free_bytes_; }

  const std::vector<GpuInfo> &gpus() const { return gpus_; }
  const GpuInfo *primary_gpu() const;
  std::int64_t best_vram_bytes() const;

  const QString &cuda_toolkit_version() const { return cuda_toolkit_version_; }
  const QString &nvidia_driver_version() const { return nvidia_driver_version_; }

  const std::vector<VolumeInfo> &volumes() const { return volumes_; }
  std::int64_t free_bytes_at(const QString &path) const;

private:
  QString os_pretty_;
  QString kernel_;
  QString cpu_model_;
  int     cpu_logical_cores_  = 0;
  int     cpu_physical_cores_ = 0;
  std::int64_t ram_total_bytes_ = 0;
  std::int64_t ram_free_bytes_  = 0;
  std::vector<GpuInfo>    gpus_;
  QString cuda_toolkit_version_;
  QString nvidia_driver_version_;
  std::vector<VolumeInfo> volumes_;
};

enum class Tool {
  CarlaServer,
  NuRec,
  AlpamayoR1,
  AlpaSimRenderer,
  Lyra2Generation,
  Lyra2Playback,
  AssetHarvester,
  AutowareBuild,
  TeraSim,
};

enum class Eligibility { Eligible, SoftWarn, HardGrey };

struct FitResult {
  Eligibility verdict = Eligibility::Eligible;
  QStringList reasons;
  QString     suggestion;
  QString     minimum_spec_pretty;
};

FitResult evaluate(Tool tool, const SystemProbe &probe);

QString human_bytes(std::int64_t bytes);

}
