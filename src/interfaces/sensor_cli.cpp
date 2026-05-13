// Copyright (C) 2026 Abdul, Hashim.
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// carla-studio-cli_sensor — sensor CLI commands
//
// Usage:
//   carla-studio sensor fisheye --configure
//       Print fisheye sensor JSON config to stdout.
//
//   carla-studio sensor fisheye --viewport
//       Launch the interactive calibration viewer (needs DISPLAY).
//
//   carla-studio sensor fisheye --configure --model <model> --fov <degrees>
//       Custom fisheye config (model: fisheye-equidistant|fisheye-equisolid, fov: 0-360).

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>

namespace {

QTextStream &out() { static QTextStream s(stdout); return s; }
QTextStream &err() { static QTextStream s(stderr); return s; }

static void log(const QString &l)    { out() << "[sensor] " << l << "\n"; out().flush(); }
static void logErr(const QString &l) { err() << "[sensor] ERROR: " << l << "\n"; err().flush(); }

// Matches the synthesizeLensCsv() presets in carla_studio.cpp
static QString fisheye_config_json(const QString &model, double fov) {
    QJsonObject cfg;
    cfg["sensor_type"]   = "sensor.camera.rgb";
    cfg["image_size_x"]  = 800;
    cfg["image_size_y"]  = 450;
    cfg["fov"]           = fov;
    cfg["lens_model"]    = model;

    QJsonObject lens;
    if (model == "fisheye-equidistant") {
        lens["type"]          = "fisheye-equidistant";
        lens["max_fov_deg"]   = fov;
        lens["cx"]            = 0.0;
        lens["cy"]            = 0.0;
        lens["xi"]            = 1.0;
    } else {
        // equisolid defaults
        lens["type"]          = "fisheye-equisolid";
        lens["max_fov_deg"]   = fov;
        lens["cx"]            = 0.0;
        lens["cy"]            = 0.0;
    }
    cfg["lens"]          = lens;
    cfg["mount_preset"]  = "Roof Center";
    cfg["px"]            = 0.0;
    cfg["py"]            = 0.0;
    cfg["pz"]            = 1.8;
    cfg["rx"]            = 0.0;
    cfg["ry"]            = 0.0;
    cfg["rz"]            = 0.0;

    return QString::fromUtf8(
        QJsonDocument(cfg).toJson(QJsonDocument::Indented));
}

}

int carla_cli_sensor_main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();

    const QString sensor = args.value(1);
    if (sensor != "fisheye") {
        err() << "Usage: carla-studio sensor <type> --configure|--viewport\n"
              << "Supported types: fisheye\n";
        return 2;
    }

    bool do_configure = false;
    bool do_viewport  = false;
    QString model     = "fisheye-equidistant";
    double  fov       = 180.0;

    for (int i = 2; i < args.size(); ++i) {
        const QString a = args[i];
        if (a == "--configure") do_configure = true;
        else if (a == "--viewport") do_viewport = true;
        else if (a == "--model" && i + 1 < args.size()) model = args[++i];
        else if (a == "--fov"   && i + 1 < args.size()) fov   = args[++i].toDouble();
    }

    if (!do_configure && !do_viewport) {
        err() << "Usage: carla-studio sensor fisheye --configure [--model <m>] [--fov <deg>]\n"
              << "       carla-studio sensor fisheye --viewport\n";
        return 2;
    }

    if (do_configure) {
        out() << fisheye_config_json(model, fov) << "\n";
    }

    if (do_viewport) {
        if (qEnvironmentVariable("DISPLAY").isEmpty() &&
            qEnvironmentVariable("WAYLAND_DISPLAY").isEmpty()) {
            logErr("No display available (DISPLAY / WAYLAND_DISPLAY not set). "
                   "Run in a graphical session.");
            return 1;
        }
        const QString preview_bin = QCoreApplication::applicationDirPath()
                                  + "/carla-studio-vehicle-preview";
        if (!QFileInfo(preview_bin).isExecutable()) {
            logErr("carla-studio-vehicle-preview not found next to this binary.");
            return 1;
        }
        // Launch the interactive calibration viewport (no mesh → calibration grid only)
        log("Launching fisheye calibration viewport...");
        QStringList pv_args;
        pv_args << "--interactive";
        QProcess preview;
        preview.setProcessChannelMode(QProcess::ForwardedChannels);
        preview.start(preview_bin, pv_args);
        if (!preview.waitForStarted(5000)) {
            logErr("Failed to start preview binary.");
            return 1;
        }
        preview.waitForFinished(-1);
    }

    return 0;
}
