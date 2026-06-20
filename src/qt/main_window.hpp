#pragma once

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QProcess>
#include <QPushButton>
#include <QTextEdit>

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow() override = default;

private slots:
  void onImportYaml();
  void onBrowseData();
  void onStartIdentification();
  void onProcessFinished(int exitCode, QProcess::ExitStatus status);

private:
  void setupUi();
  void generateConfigFiles(const QString &workDir);
  QString identifyPath() const;

  // ---- 基本参数 ----
  QLineEdit *robot_name_edit_;
  QLineEdit *dof_edit_;
  QLineEdit *kinematic_prefix_edit_;

  // ---- YAML 手动编辑区 ----
  QTextEdit *yaml_editor_;

  // ---- 数据文件 ----
  QLineEdit *data_file_edit_;
  QPushButton *browse_data_btn_;
  QComboBox *algo_combo_;

  // ---- 操作 ----
  QPushButton *import_yaml_btn_;
  QPushButton *start_btn_;
  QLabel *status_label_;

  QProcess *process_;
};
