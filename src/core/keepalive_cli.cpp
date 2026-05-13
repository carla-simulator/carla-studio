// Copyright (C) 2026 Abdul, Hashim.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include <QCoreApplication>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QThread>

namespace {
QTextStream &out() { static QTextStream s(stdout); return s; }
static void log(const QString &line) { out() << "[keepalive] " << line << "\n"; out().flush(); }

static void gsettingsSet(const QString &schema, const QString &key, const QString &value) {
    QProcess::execute("gsettings", {"set", schema, key, value});
}
static void gsettingsReset(const QString &schema, const QString &key) {
    QProcess::execute("gsettings", {"reset", schema, key});
}
static QString gsettingsGet(const QString &schema, const QString &key) {
    QProcess p;
    p.start("gsettings", {"get", schema, key});
    p.waitForFinished(3000);
    return QString::fromLocal8Bit(p.readAllStandardOutput()).trimmed();
}
}

int carla_cli_keepalive_main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();

    QString action = "run";
    int interval = 60;
    for (int i = 1; i < args.size(); ++i) {
        const QString &a = args[i];
        if (a == "disable" || a == "enable" || a == "status" || a == "run")
            action = a;
        else if ((a == "--interval" || a == "-i") && i + 1 < args.size())
            interval = args[++i].toInt();
    }

    if (action == "disable") {
        gsettingsSet("org.gnome.desktop.session", "idle-delay", "0");
        gsettingsSet("org.gnome.desktop.screensaver", "idle-activation-enabled", "false");
        QProcess::execute("/bin/sh", {"-c",
            "sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target 2>/dev/null || true"});
        QProcess::execute("xset", {"s", "off", "-dpms"});
        log("Sleep prevention applied");
        return 0;
    }

    if (action == "enable") {
        gsettingsReset("org.gnome.desktop.session", "idle-delay");
        gsettingsReset("org.gnome.desktop.screensaver", "idle-activation-enabled");
        QProcess::execute("/bin/sh", {"-c",
            "sudo systemctl unmask sleep.target suspend.target hibernate.target hybrid-sleep.target 2>/dev/null || true"});
        QProcess::execute("xset", {"s", "on", "+dpms"});
        log("Sleep enabled");
        return 0;
    }

    if (action == "status") {
        log("GNOME idle-delay: " + gsettingsGet("org.gnome.desktop.session", "idle-delay"));
        log("Screensaver enabled: " + gsettingsGet("org.gnome.desktop.screensaver", "idle-activation-enabled"));
        QProcess dpms;
        dpms.start("xset", {"q"});
        dpms.waitForFinished(3000);
        const QString q = QString::fromLocal8Bit(dpms.readAllStandardOutput());
        for (const QString &line : q.split('\n'))
            if (line.contains("DPMS")) log("  " + line.trimmed());
        return 0;
    }

    log(QString("Keep-alive running (interval %1s). Press Ctrl+C to stop.").arg(interval));
    QProcess::startDetached("caffeine", {"-a"});
    while (true) {
        QProcess::execute("xdotool", {"mousemove_relative", "1", "1"});
        QThread::sleep(static_cast<unsigned long>(interval) / 2);
        QProcess::execute("xdotool", {"mousemove_relative", "--", "-1", "-1"});
        QThread::sleep(static_cast<unsigned long>(interval) / 2);
    }
    return 0;
}
