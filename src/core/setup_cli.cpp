// Copyright (C) 2026 Abdul, Hashim.
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// carla-studio-cli_setup  — headless setup + build runner
//
// Usage:
//   carla-studio setup --directory <dir> --release <ver>
//       Fetch the GitHub release matching <ver> and download + extract the
//       Linux tarball (mirrors GUI: Cfg → Install / Update CARLA…).
//
//   carla-studio build --directory <dir> --branch <branch> [--engine <ue-path>]
//       Clone CARLA <branch> into <dir> and build with UE at <ue-path>.
//       Set CARLA_LONG_TESTS=1 to run in test mode (env-gated, takes ~2h).
//
// In every mode the tool probes for and auto-installs missing dependencies.

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QRegularExpression>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QTextStream>

#include "setup/prereq_probe.h"

namespace {

QTextStream &out() { static QTextStream s(stdout); return s; }
QTextStream &err() { static QTextStream s(stderr); return s; }

static bool g_progress = false;

static void log(const QString &line) { out() << "[setup] " << line << "\n"; out().flush(); }
static void logErr(const QString &line) { err() << "[setup] ERROR: " << line << "\n"; err().flush(); }

static QString run_cmd(const QString &cmd, int timeout_ms = 300000) {
    QProcess p;
    p.start("/bin/bash", QStringList() << "-c" << cmd);
    p.waitForFinished(timeout_ms);
    return QString::fromLocal8Bit(p.readAllStandardOutput() + p.readAllStandardError()).trimmed();
}

static bool run_interactive(const QString &cmd) {
    log("Running: " + cmd);
    QProcess p;
    p.setProcessChannelMode(QProcess::ForwardedChannels);
    p.start("/bin/bash", QStringList() << "-c" << cmd);
    p.waitForFinished(7200000);
    return p.exitCode() == 0;
}

static bool ensure_prereqs() {
    using namespace carla_studio::setup_wizard;
    struct Tool { QString name; QString pkg; };
    const QList<Tool> tools = {
        {"git",     "git"},
        {"git-lfs", "git-lfs"},
        {"cmake",   "cmake"},
        {"ninja",   "ninja-build"},
        {"python3", "python3"},
        {"pip3",    "python3-pip"},
        {"wget",    "wget"},
        {"tar",     "tar"},
        {"curl",    "curl"},
    };
    QStringList missing;
    for (const auto &t : tools) {
        const QString r = run_cmd(QString("command -v %1 2>/dev/null").arg(t.name), 3000);
        if (r.isEmpty()) missing << t.pkg;
    }
    if (missing.isEmpty()) { log("All prerequisites satisfied."); return true; }
    log("Missing tools: " + missing.join(", ") + " — installing...");
    const QString pkgs = missing.join(' ');
    const bool ok = run_interactive(QString("sudo apt-get install -y %1 2>&1").arg(pkgs));
    if (!ok) {
        logErr("apt-get install failed. Run as root or install manually: " + pkgs);
        return false;
    }
    return true;
}

// ── GitHub release resolution — mirrors fetchCarlaReleases() in carla_studio.cpp ──

struct ReleaseAsset {
    QString name;
    QString downloadUrl;
    qint64  size = 0;
};
struct ReleaseInfo {
    QString tag;
    QList<ReleaseAsset> assets;
};

static QList<ReleaseInfo> fetch_releases_from_github() {
    QStringList curlArgs;
    curlArgs << "-fsSL"
             << "-H" << "Accept: application/vnd.github+json"
             << "-H" << "X-GitHub-Api-Version: 2022-11-28";
    const QString token = QString::fromLocal8Bit(qgetenv("GITHUB_TOKEN")).trimmed();
    if (!token.isEmpty())
        curlArgs << "-H" << QStringLiteral("Authorization: Bearer %1").arg(token);
    curlArgs << "https://api.github.com/repos/carla-simulator/carla/releases?per_page=60";

    QProcess curl;
    curl.start("curl", curlArgs);
    if (!curl.waitForFinished(20000)) {
        curl.kill(); curl.waitForFinished(1500);
        logErr("Release fetch timed out.");
        return {};
    }
    if (curl.exitCode() != 0) {
        logErr("curl failed: " + QString::fromLocal8Bit(curl.readAllStandardError()));
        return {};
    }

    QJsonParseError je;
    const auto doc = QJsonDocument::fromJson(curl.readAllStandardOutput(), &je);
    if (je.error != QJsonParseError::NoError || !doc.isArray()) {
        logErr("Release JSON parse failed: " + je.errorString());
        return {};
    }

    // Regex for markdown links: [name](url) — mirrors GUI fetchCarlaReleases().
    static const QRegularExpression kMdLink(
        QStringLiteral("\\[([^\\]]+)\\]\\(([^)\\s]+)\\)"));

    QList<ReleaseInfo> result;
    for (const QJsonValue &v : doc.array()) {
        if (!v.isObject()) continue;
        const auto o = v.toObject();
        ReleaseInfo r;
        r.tag = o.value("tag_name").toString();
        if (r.tag.isEmpty()) continue;

        QSet<QString> seen;

        // 1. Formal release assets (browser_download_url).
        for (const QJsonValue &av : o.value("assets").toArray()) {
            if (!av.isObject()) continue;
            const auto ao = av.toObject();
            ReleaseAsset a;
            a.name        = ao.value("name").toString();
            a.downloadUrl = ao.value("browser_download_url").toString();
            a.size        = qint64(ao.value("size").toDouble());
            if (!a.downloadUrl.isEmpty()) {
                seen.insert(a.name.toLower());
                r.assets.append(a);
            }
        }

        // 2. Markdown links in release body — CARLA stores its CDN links here.
        //    Mirrors the body-link parsing in fetchCarlaReleases() / carla_studio.cpp.
        const QString body = o.value("body").toString();
        auto it = kMdLink.globalMatch(body);
        while (it.hasNext()) {
            auto m = it.next();
            const QString name  = m.captured(1).trimmed();
            const QString url   = m.captured(2).trimmed();
            const QString lower = name.toLower();
            const bool archiveLike =
                lower.endsWith(".tar.gz") || lower.endsWith(".tgz") ||
                lower.endsWith(".zip")    || lower.endsWith(".tar.xz");
            if (!archiveLike || seen.contains(lower)) continue;
            seen.insert(lower);
            ReleaseAsset a;
            a.name        = name;
            a.downloadUrl = url;
            r.assets.append(a);
        }

        result.append(r);
    }
    return result;
}

// Mirrors pickAssetForMode(InstallSdk) from carla_studio.cpp:
// find asset starting with "carla_" (not "additionalmaps") ending with .tar.gz
static QString pick_linux_tarball(const ReleaseInfo &r) {
    for (const auto &a : r.assets) {
        const QString lower = a.name.toLower();
        if (lower.startsWith("carla_") && !lower.startsWith("additionalmaps")
            && lower.endsWith(".tar.gz"))
            return a.downloadUrl;
    }
    // fallback: any .tar.gz asset that isn't an additional-maps pack
    for (const auto &a : r.assets) {
        const QString lower = a.name.toLower();
        if (!lower.startsWith("additionalmaps") && lower.endsWith(".tar.gz"))
            return a.downloadUrl;
    }
    return {};
}

// Resolve version string to an actual download URL.
// Tries the tag as-is, with leading 'v', without leading 'v'.
// Returns empty string and logs a diagnostic if resolution fails.
static QString resolve_download_url(const QString &version,
                                    const QList<ReleaseInfo> &releases,
                                    QString &matchedTag) {
    const QStringList candidates = {
        version,
        "v" + version,
        version.startsWith('v') ? version.mid(1) : QString(),
    };
    for (const auto &r : releases) {
        for (const QString &c : candidates) {
            if (!c.isEmpty() && r.tag == c) {
                const QString url = pick_linux_tarball(r);
                if (!url.isEmpty()) { matchedTag = r.tag; return url; }
                // Tag found but no Linux tarball asset — log available assets.
                out() << "[setup] Release tag '" << r.tag
                      << "' found but no Linux .tar.gz asset. Assets in this release:\n";
                for (const auto &a : r.assets)
                    out() << "    " << a.name << "  " << a.downloadUrl << "\n";
                out().flush();
                return {};
            }
        }
    }
    return {};
}

static int setup_binary(const QString &directory, const QString &release) {
    log(QString("Setup: binary package%1 → %2")
        .arg(release.isEmpty() ? "" : " v" + release).arg(directory));

    if (!ensure_prereqs()) return 1;

    QDir().mkpath(directory);
    if (!QFileInfo(directory).isDir()) {
        logErr("Cannot create directory: " + directory);
        return 1;
    }
    if (release.isEmpty()) {
        logErr("No --release specified. Use: carla-studio setup --directory <dir> --release <version>");
        return 2;
    }

    log("Fetching release index from GitHub…");
    const QList<ReleaseInfo> releases = fetch_releases_from_github();
    if (releases.isEmpty()) {
        logErr("Could not fetch releases from GitHub. Check network / GITHUB_TOKEN.");
        return 1;
    }

    QString matchedTag;
    const QString url = resolve_download_url(release, releases, matchedTag);
    if (url.isEmpty()) {
        // Check if the tag exists at all or if asset resolution failed.
        const QStringList candidates = {release, "v" + release,
            release.startsWith('v') ? release.mid(1) : QString()};
        bool tagFound = false;
        for (const auto &r : releases)
            for (const QString &c : candidates)
                if (!c.isEmpty() && r.tag == c) { tagFound = true; break; }
        if (!tagFound) {
            logErr(QString("Tag '%1' not found in releases. Available:").arg(release));
            for (const auto &r : releases)
                out() << "  " << r.tag << "\n";
            out().flush();
        }
        // Asset-not-found diagnostics already printed by resolve_download_url().
        return 1;
    }

    log(QString("Matched release tag: %1").arg(matchedTag));
    log(QString("Download URL: %1").arg(url));

    const QString tarball = directory + "/CARLA_" + release + ".tar.gz";
    const QString wgetProgress = g_progress
        ? "--progress=bar:force:noscroll"
        : "-q --show-progress";
    const bool dl_ok = run_interactive(
        QString("wget -c %1 -O '%2' '%3' 2>&1").arg(wgetProgress, tarball, url));
    if (!dl_ok || !QFileInfo(tarball).exists()) {
        logErr("Download failed: " + url);
        return 1;
    }

    log("Extracting " + tarball + " → " + directory);
    const QString tarVerbose = g_progress ? "v" : "";
    const bool ex_ok = run_interactive(
        QString("tar -x%1zf '%2' -C '%3' 2>&1").arg(tarVerbose, tarball, directory));
    if (!ex_ok) { logErr("Extraction failed."); return 1; }

    log("Done. CARLA_ROOT=" + directory);
    return 0;
}

static int build_source(const QString &directory, const QString &branch,
                        const QString &engine_path) {
    log(QString("Build: branch=%1  engine=%2  dest=%3")
        .arg(branch)
        .arg(engine_path.isEmpty() ? "(auto)" : engine_path)
        .arg(directory));

    if (!ensure_prereqs()) return 1;

    if (!QFileInfo(directory + "/.git").exists()) {
        QDir().mkpath(directory);
        log("Cloning carla-simulator/carla @ " + branch + " into " + directory);
        const QString gitProgress = g_progress ? "--progress" : "-q";
        const QString clone_cmd = QString(
            "git clone --depth 1 %1 -b '%2' "
            "https://github.com/carla-simulator/carla.git '%3' 2>&1")
            .arg(gitProgress, branch, directory);
        if (!run_interactive(clone_cmd)) { logErr("git clone failed."); return 1; }
        log("Initialising submodules...");
        const bool content_ok = run_interactive(
            QString("cd '%1' && git submodule update --init --recursive 2>&1").arg(directory));
        if (!content_ok) { logErr("Submodule checkout failed."); return 1; }
    } else {
        log("Repo already present — pulling latest.");
        run_interactive(QString("cd '%1' && git pull 2>&1").arg(directory));
    }

    QStringList env_extra;
    if (!engine_path.isEmpty()) {
        env_extra << QString("UNREAL_ENGINE_PATH='%1'").arg(engine_path);
        env_extra << QString("CARLA_UNREAL_ENGINE_PATH='%1'").arg(engine_path);
        env_extra << QString("UE5_ROOT='%1'").arg(engine_path);
        env_extra << QString("UE4_ROOT='%1'").arg(engine_path);
    }
    const QString env_prefix = env_extra.isEmpty() ? "" : env_extra.join(' ') + " ";

    log("Running CarlaSetup.sh --skip-prerequisites --interactive (first build takes ~2h)...");
    const bool make_ok = run_interactive(
        QString("cd '%1' && %2bash CarlaSetup.sh --skip-prerequisites --interactive 2>&1").arg(directory, env_prefix));
    if (!make_ok) { logErr("CarlaSetup.sh failed. Check output above."); return 1; }

    log("Build complete. CARLA_ROOT=" + directory);
    return 0;
}

}

int carla_cli_setup_main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();

    QString directory;
    QString release;
    QString branch;
    QString engine_path;
    bool build_only = false;

    for (int i = 1; i < args.size(); ++i) {
        if (args[i] == "--progress") { g_progress = true; continue; }
        const QString a = args[i];
        if ((a == "--directory" || a == "-d") && i + 1 < args.size())
            directory = args[++i];
        else if ((a == "--release" || a == "-r") && i + 1 < args.size())
            release = args[++i];
        else if ((a == "--branch" || a == "-b") && i + 1 < args.size())
            branch = args[++i];
        else if ((a == "--engine" || a == "-e") && i + 1 < args.size())
            engine_path = args[++i];
        else if (a == "--build-only")
            build_only = true;
    }

    if (directory.isEmpty()) {
        err() << "Usage:\n"
              << "  carla-studio setup --directory <dir> --release <ver> [--progress]\n"
              << "  carla-studio build --directory <dir> --branch <branch> [--engine <path>] [--progress]\n";
        return 2;
    }

    if (!branch.isEmpty() || build_only) {
        if (branch.isEmpty()) branch = "ue5-dev";
        return build_source(directory, branch, engine_path);
    }
    return setup_binary(directory, release);
}
