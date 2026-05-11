// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "setup/setup_wizard_dialog.h"

#include "setup/prereq_probe.h"

#include <QApplication>
#include <QClipboard>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QStandardPaths>
#include <QTabWidget>
#include <QTime>
#include <QUrl>
#include <QVBoxLayout>

namespace carla_studio::setup_wizard {

namespace {

QString stamp() { return QTime::currentTime().toString("hh:mm:ss"); }

QString defaultClonePath() {
  const QString envSrc  = QString::fromLocal8Bit(qgetenv("CARLA_SRC_ROOT")).trimmed();
  if (!envSrc.isEmpty()) return envSrc;
  const QString envRoot = QString::fromLocal8Bit(qgetenv("CARLA_ROOT")).trimmed();
  if (!envRoot.isEmpty()) return envRoot;
  return QDir::homePath() + "/carla/CARLA_src";
}

QString defaultUePath() {
  const QString env = QString::fromLocal8Bit(qgetenv("CARLA_UNREAL_ENGINE_PATH")).trimmed();
  if (!env.isEmpty()) return env;
  return QDir::homePath() + "/carla/Unreal5";
}

constexpr const char *kCommunityMaps[][2] = {
  { "Mcity Digital Twin",          "https://mcity.umich.edu/" },
  { "Apollo HD Maps (Town01-05)",  "https://github.com/ApolloAuto/apollo" },
};

void styleDot(QLabel *dot, bool ok) {
  dot->setText(ok ? "✓" : "·");
  dot->setStyleSheet(ok ? "color:#33AA55; font-weight:600;"
                        : "color:#888;");
}

}

SetupWizardDialog::SetupWizardDialog(SetCarlaRoot set_carla_root,
                                     int initial_tab,
                                     QWidget *parent)
    : QDialog(parent),
      m_set_carla_root(std::move(set_carla_root)) {
  setWindowTitle("CARLA · Setup Wizard - CARLA Studio");
  resize(720, 640);

  m_nam = new QNetworkAccessManager(this);

  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(10, 10, 10, 10);
  root->setSpacing(8);

  m_tabs = new QTabWidget(this);
  m_tabs->addTab(build_binary_tab(), "Binary Package");
  m_tabs->addTab(build_source_tab(), "Build from Source");
  m_tabs->addTab(build_maps_tab(),   "Maps");
  root->addWidget(m_tabs, 1);

  m_tabs->setCurrentIndex(std::clamp(initial_tab, 0, 2));

  connect(m_tabs, &QTabWidget::currentChanged, this, [this](int idx) {
    if (idx == 2 && m_maps_list && m_maps_list->count() <= 2)
      fetch_releases();
  });

  refresh_prereqs();
  if (std::clamp(initial_tab, 0, 2) == 2)
    fetch_releases();
}

QWidget *SetupWizardDialog::build_binary_tab() {
  auto *w = new QWidget();
  auto *l = new QVBoxLayout(w);
  l->setContentsMargins(10, 10, 10, 10);
  l->setSpacing(8);
  auto *intro = new QLabel(
    "Download a pre-built CARLA release tarball and extract it to a target "
    "directory. Studio sets <code>CARLA_ROOT</code> automatically.");
  intro->setWordWrap(true);
  l->addWidget(intro);

  auto *useExisting = new QLabel(
    "<small>For now, use <b>Tools → Install / Update CARLA…</b> to manage "
    "binary releases. The full release picker will land here in a follow-up.</small>");
  useExisting->setWordWrap(true);
  l->addWidget(useExisting);
  l->addStretch();
  return w;
}

QWidget *SetupWizardDialog::build_source_tab() {
  auto *w = new QWidget();
  auto *l = new QVBoxLayout(w);
  l->setContentsMargins(10, 10, 10, 10);
  l->setSpacing(8);

  auto *cloneRow = new QHBoxLayout();
  cloneRow->addWidget(new QLabel("Clone to:"));
  m_clone_path = new QLineEdit(defaultClonePath());
  cloneRow->addWidget(m_clone_path, 1);
  auto *browseClone = new QPushButton("Browse…");
  cloneRow->addWidget(browseClone);
  l->addLayout(cloneRow);
  connect(browseClone, &QPushButton::clicked, this, &SetupWizardDialog::browse_clone_path);

  auto *introBox = new QLabel(
    "<b>Build CARLA from Source</b> - clones the CARLA GitHub repo and "
    "compiles the Unreal Engine plugin. Required for Vehicle Model Import.");
  introBox->setWordWrap(true);
  introBox->setStyleSheet(
    "background:#FFF8E1; color:#5C4400; padding:6px; border:1px solid #E5C070;");
  l->addWidget(introBox);

  auto *fullBuild = new QLabel(
    "<small><b>Full build requires:</b> CARLA's fork of UE5 (clone + build, "
    "~3 h, ~225 GB), CARLA source clone, and CARLA content clone (Bitbucket). "
    "Steps 1-3 below guide you through each stage. "
    "<a href='https://carla-ue5.readthedocs.io/en/latest/build_linux_ue5/'>UE5 build docs ↗</a></small>");
  fullBuild->setOpenExternalLinks(true);
  fullBuild->setWordWrap(true);
  l->addWidget(fullBuild);

  auto *branchRow = new QHBoxLayout();
  branchRow->addWidget(new QLabel("Branch:"));
  m_branch_combo = new QComboBox();
  m_branch_combo->addItems({"ue5-dev", "master", "dev"});
  branchRow->addWidget(m_branch_combo);
  branchRow->addStretch();
  l->addLayout(branchRow);

  auto *prereqGroup = new QGroupBox("Prerequisites");
  auto *prereqLayout = new QVBoxLayout(prereqGroup);
  auto *prereqHeader = new QHBoxLayout();
  prereqHeader->addStretch();
  m_install_prereq_btn = new QPushButton("Install Prerequisites…");
  prereqHeader->addWidget(m_install_prereq_btn);
  prereqLayout->addLayout(prereqHeader);

  auto addRow = [&](QLabel *&dot, QLabel *&detail, const QString &name) {
    auto *row = new QHBoxLayout();
    dot = new QLabel("·");
    dot->setFixedWidth(16);
    auto *toolLabel = new QLabel(name);
    toolLabel->setStyleSheet("font-family: monospace;");
    detail = new QLabel("checking…");
    detail->setStyleSheet("color:#666;");
    row->addWidget(dot);
    row->addWidget(toolLabel);
    row->addWidget(detail, 1);
    prereqLayout->addLayout(row);
  };
  addRow(m_git_dot,   m_git_detail,   "git - version control");
  addRow(m_lfs_dot,   m_lfs_detail,   "git-lfs - large-file content submodule");
  addRow(m_ninja_dot, m_ninja_detail, "ninja - build executor");
  addRow(m_cmake_dot, m_cmake_detail, "cmake");
  l->addWidget(prereqGroup);

  auto *ueRow = new QHBoxLayout();
  ueRow->addWidget(new QLabel("UE5 path:"));
  m_ue_path = new QLineEdit(defaultUePath());
  ueRow->addWidget(m_ue_path, 1);
  auto *browseUe = new QPushButton("Browse…");
  ueRow->addWidget(browseUe);
  m_ue_ready_label = new QLabel("·");
  m_ue_ready_label->setFixedWidth(56);
  ueRow->addWidget(m_ue_ready_label);
  l->addLayout(ueRow);
  connect(browseUe, &QPushButton::clicked, this, &SetupWizardDialog::browse_ue_path);
  connect(m_ue_path, &QLineEdit::textChanged, this, [this](const QString &) { refresh_prereqs(); });

  m_log = new QPlainTextEdit();
  m_log->setReadOnly(true);
  m_log->setMinimumHeight(100);
  m_log->setPlaceholderText("Build log will appear here…");
  l->addWidget(m_log, 1);

  auto *btn_row = new QHBoxLayout();
  m_re_clone_btn      = new QPushButton("Re-clone");
  m_clone_content_btn = new QPushButton("Clone Content");
  m_build_btn        = new QPushButton("Build  (cmake + ninja)");
  m_set_as_root_btn    = new QPushButton("Set as CARLA_ROOT");
  m_copy_log_btn      = new QPushButton("Copy Log");
  auto *closeBtn   = new QPushButton("Close");
  btn_row->addWidget(m_re_clone_btn);
  btn_row->addWidget(m_clone_content_btn);
  btn_row->addWidget(m_build_btn);
  btn_row->addWidget(m_set_as_root_btn);
  btn_row->addStretch();
  btn_row->addWidget(m_copy_log_btn);
  btn_row->addWidget(closeBtn);
  l->addLayout(btn_row);

  connect(m_re_clone_btn,      &QPushButton::clicked, this, &SetupWizardDialog::on_clone_repo);
  connect(m_clone_content_btn, &QPushButton::clicked, this, &SetupWizardDialog::on_clone_content);
  connect(m_build_btn,        &QPushButton::clicked, this, &SetupWizardDialog::on_build);
  connect(m_set_as_root_btn,    &QPushButton::clicked, this, &SetupWizardDialog::on_set_as_root);
  connect(m_copy_log_btn,      &QPushButton::clicked, this, &SetupWizardDialog::on_copy_log);
  connect(closeBtn,         &QPushButton::clicked, this, &QDialog::close);
  connect(m_install_prereq_btn, &QPushButton::clicked, this, [this]() {
    append_log("Run: sudo apt update && sudo apt install -y git git-lfs ninja-build cmake build-essential");
    QProcess::execute("/bin/bash", QStringList() << "-lc"
      << "x-terminal-emulator -e 'sudo apt update && sudo apt install -y git git-lfs ninja-build cmake build-essential' "
         "2>/dev/null || gnome-terminal -- bash -c 'sudo apt update && sudo apt install -y git git-lfs ninja-build cmake build-essential; read'");
  });

  return w;
}

QWidget *SetupWizardDialog::build_maps_tab() {
  auto *w = new QWidget();
  auto *l = new QVBoxLayout(w);
  l->setContentsMargins(10, 10, 10, 10);
  l->setSpacing(8);

  auto *targetRow = new QHBoxLayout();
  targetRow->addWidget(new QLabel("Target root:"));
  m_maps_target_root = new QLineEdit(defaultClonePath());
  targetRow->addWidget(m_maps_target_root, 1);
  l->addLayout(targetRow);

  auto *intro = new QLabel(
    "<b>Maps</b> Official additional map packs and community-contributed maps - select one to install.");
  intro->setStyleSheet("color:#444;");
  l->addWidget(intro);

  m_maps_list = new QListWidget();
  auto *headerOfficial = new QListWidgetItem("- Official Additional Maps (loading…) -");
  headerOfficial->setFlags(Qt::NoItemFlags);
  headerOfficial->setForeground(Qt::gray);
  m_maps_list->addItem(headerOfficial);
  auto *headerComm = new QListWidgetItem("- Community Maps -");
  headerComm->setFlags(Qt::NoItemFlags);
  headerComm->setForeground(Qt::gray);
  m_maps_list->addItem(headerComm);
  for (const auto &m : kCommunityMaps) {
    auto *it = new QListWidgetItem(QString::fromLatin1(m[0]));
    it->setData(Qt::UserRole, QString::fromLatin1(m[1]));
    m_maps_list->addItem(it);
  }
  l->addWidget(m_maps_list, 1);

  auto *btn_row = new QHBoxLayout();
  m_maps_refresh_btn = new QPushButton("Refresh list");
  m_maps_install_btn = new QPushButton("Install Selected Map");
  auto *closeBtn  = new QPushButton("Close");
  btn_row->addWidget(m_maps_refresh_btn);
  btn_row->addStretch();
  btn_row->addWidget(m_maps_install_btn);
  btn_row->addWidget(closeBtn);
  l->addLayout(btn_row);

  m_maps_log = new QPlainTextEdit();
  m_maps_log->setReadOnly(true);
  m_maps_log->setMaximumHeight(80);
  m_maps_log->setPlainText("Using cached release list.");
  l->addWidget(m_maps_log);

  connect(m_maps_refresh_btn, &QPushButton::clicked, this, &SetupWizardDialog::fetch_releases);
  connect(m_maps_install_btn, &QPushButton::clicked, this, [this]() {
    auto *it = m_maps_list->currentItem();
    if (!it || !(it->flags() & Qt::ItemIsSelectable)) return;
    const QString tag = it->data(Qt::UserRole).toString();
    m_maps_log->appendPlainText(QString("[%1]  Install %2 → %3 (download flow not yet wired).")
                                .arg(stamp(), tag, m_maps_target_root->text().trimmed()));
  });
  connect(closeBtn, &QPushButton::clicked, this, &QDialog::close);

  return w;
}

void SetupWizardDialog::fetch_releases() {
  if (!m_nam || !m_maps_list || !m_maps_log) return;
  m_maps_refresh_btn->setEnabled(false);
  m_maps_log->appendPlainText(QString("[%1]  Fetching release list from GitHub…").arg(stamp()));

  QNetworkRequest req(QUrl("https://api.github.com/repos/carla-simulator/carla/releases?per_page=50"));
  req.setRawHeader("Accept", "application/vnd.github+json");
  req.setRawHeader("X-GitHub-Api-Version", "2022-11-28");

  auto *reply = m_nam->get(req);
  connect(reply, &QNetworkReply::finished, this, [this, reply]() {
    reply->deleteLater();
    m_maps_refresh_btn->setEnabled(true);

    if (reply->error() != QNetworkReply::NoError) {
      m_maps_log->appendPlainText(
        QString("[%1]  Network error: %2").arg(stamp(), reply->errorString()));
      return;
    }

    const QJsonDocument doc = QJsonDocument::fromJson(reply->readAll());
    if (!doc.isArray()) {
      m_maps_log->appendPlainText(QString("[%1]  Unexpected response format.").arg(stamp()));
      return;
    }

    auto *header = m_maps_list->item(0);
    if (header) header->setText("- Official Additional Maps -");

    int insertPos = 1;
    for (const QJsonValue &v : doc.array()) {
      const QString tag = v.toObject().value("tag_name").toString();
      if (tag.isEmpty()) continue;
      auto *existing = m_maps_list->item(insertPos);
      if (existing && existing->data(Qt::UserRole).toString() == tag) {
        ++insertPos;
        continue;
      }
      auto *it = new QListWidgetItem(QString("%1  AdditionalMaps").arg(tag));
      it->setData(Qt::UserRole, tag);
      m_maps_list->insertItem(insertPos++, it);
    }

    m_maps_log->appendPlainText(
      QString("[%1]  Loaded %2 release(s).").arg(stamp()).arg(insertPos - 1));
  });
}

void SetupWizardDialog::refresh_prereqs() {
  const auto g = probe_git();
  styleDot(m_git_dot, g.ok); m_git_detail->setText(g.detail);
  const auto lfs = probe_git_lfs();
  styleDot(m_lfs_dot, lfs.ok); m_lfs_detail->setText(lfs.detail);
  const auto n = probe_ninja();
  styleDot(m_ninja_dot, n.ok); m_ninja_detail->setText(n.detail);
  const auto c = probe_cmake();
  styleDot(m_cmake_dot, c.ok); m_cmake_detail->setText(c.detail);

  const bool ueOk = path_has_unreal_editor(m_ue_path->text().trimmed());
  m_ue_ready_label->setText(ueOk ? "✓ ready" : "✗ missing");
  m_ue_ready_label->setStyleSheet(ueOk ? "color:#33AA55;" : "color:#CC4444;");
}

void SetupWizardDialog::browse_clone_path() {
  const QString d = QFileDialog::getExistingDirectory(
      this, "Choose clone directory", m_clone_path->text());
  if (!d.isEmpty()) m_clone_path->setText(d);
}

void SetupWizardDialog::browse_ue_path() {
  const QString d = QFileDialog::getExistingDirectory(
      this, "Choose UE5 source directory", m_ue_path->text());
  if (!d.isEmpty()) m_ue_path->setText(d);
}

void SetupWizardDialog::append_log(const QString &line) {
  if (m_log) m_log->appendPlainText(QString("[%1]  %2").arg(stamp(), line));
}

void SetupWizardDialog::on_clone_repo() {
  const QString dest   = m_clone_path->text().trimmed();
  const QString branch = m_branch_combo->currentText();
  if (dest.isEmpty()) { append_log("Clone: target path is empty."); return; }
  append_log(QString("Cloning carla-simulator/carla (%1) into %2 …").arg(branch, dest));
  const QString cmd = QString("git clone --depth 1 -b %1 https://github.com/carla-simulator/carla.git %2 2>&1")
                        .arg(branch, dest);
  QProcess::startDetached("/bin/bash", QStringList() << "-lc" << cmd);
}

void SetupWizardDialog::on_clone_content() {
  const QString dest = m_clone_path->text().trimmed();
  if (dest.isEmpty()) { append_log("Content clone: target path is empty."); return; }
  const QString contentDest = dest + "/Unreal/CarlaUnreal/Content/Carla";
  append_log(QString("Cloning content into %1 …").arg(contentDest));
  const QString cmd = QString("git clone --depth 1 https://bitbucket.org/carla-simulator/carla-content.git %1 2>&1")
                        .arg(contentDest);
  QProcess::startDetached("/bin/bash", QStringList() << "-lc" << cmd);
}

void SetupWizardDialog::on_build() {
  const QString dest = m_clone_path->text().trimmed();
  if (dest.isEmpty()) { append_log("Build: target path is empty."); return; }
  const QString ue_path = m_ue_path->text().trimmed();
  append_log("Configuring + building (cmake + ninja) - see terminal for live output.");
  const QString cmd = QString(
      "cd %1 && CARLA_UNREAL_ENGINE_PATH=%2 cmake -G Ninja -S . -B Build/Development "
      "--toolchain=$PWD/CMake/Toolchain.cmake "
      "-DCMAKE_BUILD_TYPE=Development -DBUILD_CARLA_UNREAL=ON && "
      "cmake --build Build/Development --target carla-unreal 2>&1")
        .arg(dest, ue_path);
  QProcess::startDetached("/bin/bash", QStringList() << "-lc"
      << QString("x-terminal-emulator -e 'bash -c \"%1; read\"' || gnome-terminal -- bash -c '%1; read'")
           .arg(cmd));
}

void SetupWizardDialog::on_set_as_root() {
  const QString dest = m_clone_path->text().trimmed();
  if (dest.isEmpty()) { append_log("Set as CARLA_ROOT: target path is empty."); return; }
  if (m_set_carla_root) m_set_carla_root(dest);
  append_log(QString("CARLA_ROOT set to: %1").arg(dest));
  qputenv("CARLA_ROOT", dest.toLocal8Bit());
}

void SetupWizardDialog::on_copy_log() {
  if (m_log) QApplication::clipboard()->setText(m_log->toPlainText());
}

}
