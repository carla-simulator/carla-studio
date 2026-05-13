// Copyright (C) 2026 Abdul, Hashim.
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// carla-studio-cli_healthcheck — connectivity and environment health checker
//
// Usage: carla-studio healthcheck [--host <host>] [--port <rpc-port>]
//
// Exit codes:
//   0  all checks passed
//   1  one or more checks failed (details on stdout)

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>

#include "utils/resource_fit.h"
#include "utils/platform_proc.h"

namespace {

QTextStream &out() { static QTextStream s(stdout); return s; }

struct CheckResult {
    QString name;
    bool    ok   = false;
    QString detail;
};

static QString run(const QString &cmd, int timeout_ms = 5000) {
    QProcess p;
    p.start("/bin/bash", QStringList() << "-c" << cmd);
    p.waitForFinished(timeout_ms);
    return QString::fromLocal8Bit(
        p.readAllStandardOutput() + p.readAllStandardError()).trimmed();
}

static CheckResult check_carla_rpc(const QString &host, int port) {
    CheckResult r;
    r.name = QString("CARLA RPC %1:%2").arg(host).arg(port);
    const QString probe = run(
        QString("timeout 3 bash -c 'echo >>/dev/tcp/%1/%2' 2>&1").arg(host).arg(port), 5000);
    r.ok     = probe.isEmpty();
    r.detail = r.ok ? "reachable" : "not reachable (start carla-studio start first)";
    return r;
}

static CheckResult check_python_carla() {
    CheckResult r;
    r.name = "Python carla package";
    const QString v = run("python3 -c 'import carla; print(carla.__version__)' 2>&1", 8000);
    r.ok     = !v.isEmpty() && !v.contains("No module named") && !v.contains("Error");
    r.detail = r.ok ? ("version " + v) : ("not found — " + v.left(120));
    return r;
}

static CheckResult check_gpu() {
    CheckResult r;
    r.name = "GPU (nvidia-smi)";
#if defined(__APPLE__)
    r.ok = true; r.detail = "macOS — nvidia-smi not applicable";
#else
    const QString v = run("nvidia-smi --query-gpu=name --format=csv,noheader 2>&1", 4000);
    r.ok     = !v.isEmpty() && !v.contains("command not found") && !v.contains("error");
    r.detail = r.ok ? v.left(80) : "no NVIDIA GPU detected (or nvidia-smi missing)";
#endif
    return r;
}

static CheckResult check_ram() {
    CheckResult r;
    r.name = "RAM";
    const qint64 mib = carla_studio_proc::system_total_ram_mib();
    r.ok     = mib > 8192;
    r.detail = QString("%1 GiB total (%2)").arg(mib / 1024).arg(
        r.ok ? "OK" : "< 8 GiB — may be insufficient for CARLA");
    return r;
}

static CheckResult check_disk() {
    CheckResult r;
    r.name = "Disk (CARLA_ROOT)";
    const QString carla_root = QString::fromLocal8Bit(qgetenv("CARLA_ROOT")).trimmed();
    if (carla_root.isEmpty()) {
        r.ok = false;
        r.detail = "CARLA_ROOT not set";
        return r;
    }
    const QString df = run(QString("df -BG --output=avail '%1' 2>/dev/null | tail -1").arg(carla_root), 3000);
    bool ok_conv = false;
    const int avail_gb = QString(df).remove('G').trimmed().toInt(&ok_conv);
    r.ok     = ok_conv && avail_gb > 10;
    r.detail = ok_conv ? QString("%1 GiB free at %2").arg(avail_gb).arg(carla_root)
                       : "cannot determine free space at " + carla_root;
    return r;
}

static CheckResult check_carla_binary() {
    CheckResult r;
    r.name = "CARLA binary";
    const QString root = QString::fromLocal8Bit(qgetenv("CARLA_ROOT")).trimmed();
    if (root.isEmpty()) { r.ok = false; r.detail = "CARLA_ROOT not set"; return r; }
    static const QStringList kBinaries = {
        "CarlaUnreal-Linux-Shipping", "CarlaUnreal.sh",
        "CarlaUE5.sh", "CarlaUE4.sh", "LinuxNoEditor/CarlaUE4.sh",
    };
    for (const QString &rel : kBinaries) {
        const QString p = root + "/" + rel;
        if (QFileInfo::exists(p)) {
            r.ok = true;
            r.detail = p;
            return r;
        }
    }
    // Some releases (e.g. 0.10.0) extract into a single versioned subdirectory.
    for (const QFileInfo &fi : QDir(root).entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot)) {
        for (const QString &rel : kBinaries) {
            const QString p = fi.filePath() + "/" + rel;
            if (QFileInfo::exists(p)) {
                r.ok = true;
                r.detail = p;
                return r;
            }
        }
    }
    r.ok = false;
    r.detail = "No shipping binary found in " + root;
    return r;
}

}

int carla_cli_healthcheck_main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();

    QString host = "localhost";
    int port = 2000;
    for (int i = 1; i < args.size(); ++i) {
        const QString a = args[i];
        if ((a == "--host" || a == "-H") && i + 1 < args.size())
            host = args[++i];
        else if ((a == "--port" || a == "-p") && i + 1 < args.size())
            port = args[++i].toInt();
    }

    out() << "[healthcheck] CARLA Studio environment check\n";
    out() << "─────────────────────────────────────────────\n";

    const QList<CheckResult> checks = {
        check_carla_binary(),
        check_carla_rpc(host, port),
        check_python_carla(),
        check_gpu(),
        check_ram(),
        check_disk(),
    };

    bool all_ok = true;
    for (const auto &c : checks) {
        const QString mark = c.ok ? "✓" : "✗";
        out() << QString("  %1  %2  %3\n")
                   .arg(mark)
                   .arg(c.name, -30)
                   .arg(c.detail);
        if (!c.ok) all_ok = false;
    }

    out() << "─────────────────────────────────────────────\n"
          << "[healthcheck] " << (all_ok ? "PASS" : "FAIL") << "\n";
    out().flush();
    return all_ok ? 0 : 1;
}
