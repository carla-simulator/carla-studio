// Copyright (C) 2026 Abdul, Hashim.
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// carla-studio-cli_actuate — SAE autonomy level setter
//
// Usage:
//   carla-studio actuate sae-l5              Full automation
//   carla-studio actuate sae-l4              High automation
//   carla-studio actuate sae-l3              Conditional automation
//   carla-studio actuate sae-l2              Partial automation
//   carla-studio actuate sae-l1 [--keyboard] Driver assistance (+ keyboard ego)
//   carla-studio actuate sae-l0              No automation (manual)
//
// Persists the level to QSettings so the GUI reflects the selection on next open.

#include <QCoreApplication>
#include <QSettings>
#include <QString>
#include <QStringList>
#include <QTextStream>

namespace {

QTextStream &out() { static QTextStream s(stdout); return s; }
QTextStream &err() { static QTextStream s(stderr); return s; }

static void log(const QString &l) { out() << "[actuate] " << l << "\n"; out().flush(); }

static const struct { const char *id; int level; const char *label; } kLevels[] = {
    { "sae-l0", 0, "L0 No Automation"       },
    { "sae-l1", 1, "L1 Driver Assistance"   },
    { "sae-l2", 2, "L2 Partial Automation"  },
    { "sae-l3", 3, "L3 Conditional Automation" },
    { "sae-l4", 4, "L4 High Automation"     },
    { "sae-l5", 5, "L5 Full Automation"     },
};

}

int carla_cli_actuate_main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    QCoreApplication::setOrganizationName("CARLA Simulator");
    QCoreApplication::setApplicationName("CARLA Studio");

    const QStringList args = app.arguments();
    const QString level_id = args.value(1).toLower();

    bool keyboard = args.contains("--keyboard");

    if (level_id.isEmpty()) {
        err() << "Usage: carla-studio actuate <sae-level> [--keyboard]\n"
              << "Levels: sae-l0  sae-l1  sae-l2  sae-l3  sae-l4  sae-l5\n";
        return 2;
    }

    int chosen_level = -1;
    const char *chosen_label = nullptr;
    for (const auto &entry : kLevels) {
        if (level_id == entry.id) {
            chosen_level = entry.level;
            chosen_label = entry.label;
            break;
        }
    }

    if (chosen_level < 0) {
        err() << "Unknown SAE level: " << level_id << "\n"
              << "Valid: sae-l0 sae-l1 sae-l2 sae-l3 sae-l4 sae-l5\n";
        return 2;
    }

    QSettings settings;
    settings.setValue("actuate/sae_level", chosen_level);
    log(QString("SAE level set to %1").arg(chosen_label));

    if (keyboard) {
        settings.setValue("actuate/player_EGO", "Keyboard");
        log("EGO control set to Keyboard.");
    }

    if (chosen_level == 1) {
        // L1: persist the l1 feature choice (acc by default; --lane-keep overrides)
        const bool lane_keep = args.contains("--lane-keep");
        settings.setValue("actuate/sae_l1_feature", lane_keep ? "lane_keep" : "acc");
        log(QString("L1 feature: %1").arg(lane_keep ? "Lane Keeping" : "Adaptive Cruise Control"));
    }

    settings.sync();
    log("Settings persisted. GUI will reflect this on next launch.");
    return 0;
}
