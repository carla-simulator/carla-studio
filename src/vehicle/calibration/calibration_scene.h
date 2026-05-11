// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <QJsonObject>
#include <QList>
#include <QString>
#include <QVector3D>
#include <QWidget>

class QListWidget;
class QListWidgetItem;
class QStackedWidget;

namespace Qt3DCore   { class QEntity; class QTransform; }
namespace Qt3DExtras { class Qt3DWindow; }
namespace Qt3DRender { class QPickEvent; }

namespace carla_studio::calibration {

enum class TargetType {
  Checkerboard = 0,
  AprilTag,
  ColorChecker,
  ConeSquare,
  ConeLine,
  PylonWall,
};

struct PlacedTarget {
  TargetType type = TargetType::Checkerboard;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw_deg = 0.0;
  int    cols = 8;
  int    rows = 6;
  double size_m = 1.2;
  int    april_id = 0;

  QJsonObject to_json() const;
  static PlacedTarget from_json(const QJsonObject &);
};

QString label_for(TargetType t);

QList<PlacedTarget> load_persisted(const QString &scenario_id,
                                  const QString &sensor_name);
void                save_persisted(const QString &scenario_id,
                                  const QString &sensor_name,
                                  const QList<PlacedTarget> &targets);
QStringList         list_persisted_sensors(const QString &scenario_id);

class CalibrationScene : public QWidget {
  Q_OBJECT
 public:
  explicit CalibrationScene(QWidget *parent = nullptr);
  ~CalibrationScene() override;

  void set_hero_pose(double xMeters, double yMeters, double yaw_deg);
  void set_sensor_pose(double xMeters, double yMeters, double zMeters,
                     double yaw_deg);
  void set_targets(const QList<PlacedTarget> &);
  QList<PlacedTarget> targets() const { return m_targets; }
  int  selected_index() const { return m_selected; }

 signals:
  void targets_changed(const QList<PlacedTarget> &);
  void selection_changed(int index);

 public slots:
  void set_next_drop_type(TargetType t) { m_next_drop_type = t; }
  void add_target_at_center(TargetType t);
  void add_target_at(TargetType t, double xWorld, double yWorld);
  void remove_at(int index);
  void clear_all();
  void update_target_pose(int index, double x, double y, double z, double yaw_deg);

 private:
  void rebuild_target_entities();
  void on_ground_clicked(Qt3DRender::QPickEvent *);
  void on_target_clicked(int index, Qt3DRender::QPickEvent *);

  Qt3DExtras::Qt3DWindow *m_view         = nullptr;
  Qt3DCore::QEntity      *m_root         = nullptr;
  Qt3DCore::QEntity      *m_ground       = nullptr;
  Qt3DCore::QEntity      *m_hero         = nullptr;
  Qt3DCore::QTransform   *m_hero_xform    = nullptr;
  Qt3DCore::QEntity      *m_sensor_marker = nullptr;
  Qt3DCore::QTransform   *m_sensor_xform  = nullptr;
  Qt3DCore::QEntity      *m_targets_root  = nullptr;

  QList<PlacedTarget>     m_targets;
  int                     m_selected = -1;
  TargetType              m_next_drop_type = TargetType::Checkerboard;
};

}
