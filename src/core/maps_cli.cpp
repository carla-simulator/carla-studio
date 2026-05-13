// Copyright (C) 2026 Abdul, Hashim.
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// carla-studio-cli_maps — load-additional-maps provider installer
//
// Usage:
//   carla-studio load-additional-maps mcity [--directory <carla-root>]
//   carla-studio load-additional-maps apollo [--directory <carla-root>]

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>

namespace {

QTextStream &out() { static QTextStream s(stdout); return s; }
QTextStream &err() { static QTextStream s(stderr); return s; }

static bool g_progress = false;

static void log(const QString &l)    { out() << "[maps] " << l << "\n"; out().flush(); }
static void logErr(const QString &l) { err() << "[maps] ERROR: " << l << "\n"; err().flush(); }

struct Provider {
    QString id;
    QString label;
    QString git_url;
    QString target_subdir;
    QString load_hint;
};

static const QList<Provider> kProviders = {
    {
        "mcity",
        "Mcity Digital Twin",
        "https://github.com/mcity/mcity-digital-twin.git",
        "CommunityMaps/mcity",
        "python PythonAPI/examples/load_mcity_digital_twin.py"
    },
    {
        "apollo",
        "Apollo HD Maps (Town01-05)",
        "https://github.com/MaisJamal/Carla_apollo_maps.git",
        "CommunityMaps/apollo",
        "Copy map folders from CommunityMaps/apollo into <CARLA_ROOT>/CommunityMaps/"
    },
};

static QString findCarlaRoot(const QString &hint) {
    if (!hint.isEmpty() && QFileInfo(hint).isDir()) return hint;
    const QString env = QString::fromLocal8Bit(qgetenv("CARLA_ROOT")).trimmed();
    if (!env.isEmpty() && QFileInfo(env).isDir()) return env;
    return {};
}

static bool run_interactive(const QString &cmd) {
    log("$ " + cmd);
    QProcess p;
    p.setProcessChannelMode(QProcess::ForwardedChannels);
    p.start("/bin/bash", QStringList() << "-c" << cmd);
    p.waitForFinished(1800000);
    return p.exitCode() == 0;
}

}

int carla_cli_maps_main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();

    QString provider_id;
    QString directory;

    for (int i = 1; i < args.size(); ++i) {
        const QString a = args[i];
        if (a == "--progress") { g_progress = true; }
        else if ((a == "--directory" || a == "-d") && i + 1 < args.size())
            directory = args[++i];
        else if (!a.startsWith('-'))
            provider_id = a;
    }

    if (provider_id.isEmpty()) {
        out() << "Usage: carla-studio load-additional-maps <provider> [--directory <carla-root>] [--progress]\n"
              << "\nAvailable providers:\n";
        for (const auto &p : kProviders)
            out() << "  " << p.id << "  — " << p.label << "\n";
        return 2;
    }

    const Provider *chosen = nullptr;
    for (const auto &p : kProviders) {
        if (p.id == provider_id) { chosen = &p; break; }
    }
    if (!chosen) {
        logErr("Unknown provider: " + provider_id);
        out() << "Available: ";
        for (const auto &p : kProviders) out() << p.id << " ";
        out() << "\n";
        return 2;
    }

    const QString carla_root = findCarlaRoot(directory);
    if (carla_root.isEmpty()) {
        logErr("CARLA root not found. Set CARLA_ROOT or use --directory <path>.");
        return 1;
    }

    const QString target = carla_root + "/" + chosen->target_subdir;
    log(QString("Installing '%1' into %2").arg(chosen->label).arg(target));

    if (QFileInfo(target + "/.git").exists()) {
        log("Already cloned — pulling latest...");
        if (!run_interactive(QString("cd '%1' && git pull %2 2>&1").arg(target, g_progress ? "--progress" : "-q"))) {
            logErr("git pull failed.");
            return 1;
        }
    } else {
        QDir().mkpath(target);
        const QString gitProgress = g_progress ? "--progress" : "-q";
        if (!run_interactive(
                QString("git clone --depth 1 %1 '%2' '%3' 2>&1")
                .arg(gitProgress, chosen->git_url, target))) {
            logErr("git clone failed.");
            return 1;
        }
    }

    log("Installed. Load hint:");
    log("  " + chosen->load_hint);
    return 0;
}
