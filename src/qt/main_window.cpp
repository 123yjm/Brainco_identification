#include "main_window.hpp"

#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QVBoxLayout>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

// ---------------------------------------------------------------------------
// 默认 YAML 模板（手动填写时预填）
// ---------------------------------------------------------------------------
static const char *kDefaultYamlTemplate = R"(robot_name: "my_robot"
dof: 7
kinematic_prefix: 0

bodies:
  - name: "link1"
    mass: 1.0
    com: [0.0, 0.0, 0.0]
    Ixx: 0.01
    Iyy: 0.01
    Izz: 0.01
    joint_axis: [0.0, 0.0, 1.0]
    has_joint: true
)";

// ---------------------------------------------------------------------------
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), process_(new QProcess(this)) {
  setWindowTitle("机械臂动力学参数辨识工具");
  setupUi();

  connect(process_,
          QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &MainWindow::onProcessFinished);
}

// ---------------------------------------------------------------------------
void MainWindow::setupUi() {
  auto *central = new QWidget(this);
  setCentralWidget(central);
  auto *main_layout = new QVBoxLayout(central);

  // ---- 基本参数 ----
  auto *basic_group = new QGroupBox("基本参数", this);
  auto *basic_form = new QFormLayout(basic_group);

  robot_name_edit_ = new QLineEdit("serial_arm", this);
  dof_edit_ = new QLineEdit("7", this);
  kinematic_prefix_edit_ = new QLineEdit("2", this);

  basic_form->addRow("机器人名称:", robot_name_edit_);
  basic_form->addRow("DOF:", dof_edit_);
  basic_form->addRow("Kinematic Prefix:", kinematic_prefix_edit_);

  main_layout->addWidget(basic_group);

  // ---- YAML 编辑区 ----
  auto *yaml_group = new QGroupBox("运动学参数 (YAML)", this);
  auto *yaml_layout = new QVBoxLayout(yaml_group);

  import_yaml_btn_ = new QPushButton("导入 YAML 文件...", this);
  connect(import_yaml_btn_, &QPushButton::clicked,
          this, &MainWindow::onImportYaml);

  yaml_editor_ = new QTextEdit(this);
  yaml_editor_->setFont(QFont("Monospace", 10));
  yaml_editor_->setPlaceholderText("在此粘贴或编辑 YAML 运动学参数...");
  yaml_editor_->setPlainText(kDefaultYamlTemplate);

  yaml_layout->addWidget(import_yaml_btn_);
  yaml_layout->addWidget(yaml_editor_);

  main_layout->addWidget(yaml_group);

  // ---- 数据文件 + 算法 ----
  auto *data_group = new QGroupBox("辨识设置", this);
  auto *data_form = new QFormLayout(data_group);

  auto *data_row = new QHBoxLayout();
  data_file_edit_ = new QLineEdit(this);
  data_file_edit_->setPlaceholderText("data/revoarm_filtered_data_condnum_56.12_0618.csv");
  browse_data_btn_ = new QPushButton("浏览...", this);
  connect(browse_data_btn_, &QPushButton::clicked,
          this, &MainWindow::onBrowseData);
  data_row->addWidget(data_file_edit_);
  data_row->addWidget(browse_data_btn_);

  algo_combo_ = new QComboBox(this);
  algo_combo_->addItems({"OLS", "WLS", "IRLS", "TLS", "EKF", "NLS_FRICTION"});

  data_form->addRow("数据 CSV:", data_row);
  data_form->addRow("算法:", algo_combo_);

  main_layout->addWidget(data_group);

  // ---- 操作按钮 ----
  start_btn_ = new QPushButton("开始辨识", this);
  start_btn_->setMinimumHeight(40);
  start_btn_->setStyleSheet("font-size: 16px; font-weight: bold;");
  connect(start_btn_, &QPushButton::clicked,
          this, &MainWindow::onStartIdentification);

  status_label_ = new QLabel("就绪", this);
  status_label_->setAlignment(Qt::AlignCenter);

  main_layout->addWidget(start_btn_);
  main_layout->addWidget(status_label_);
  main_layout->addStretch();
}

// ---------------------------------------------------------------------------
QString MainWindow::identifyPath() const {
  // 优先使用环境变量，否则假设与 GUI 可执行文件同目录
  const char *env = std::getenv("IDENTIFY_PATH");
  if (env) return QString::fromUtf8(env);
  return "./identify";
}

// ---------------------------------------------------------------------------
void MainWindow::onImportYaml() {
  QString path = QFileDialog::getOpenFileName(
      this, "导入运动学参数 YAML", QString(),
      "YAML 文件 (*.yaml *.yml);;所有文件 (*)");
  if (path.isEmpty()) return;

  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QMessageBox::warning(this, "错误", "无法打开文件: " + path);
    return;
  }
  QString content = QString::fromUtf8(file.readAll());
  yaml_editor_->setPlainText(content);

  // 尝试从 YAML 中提取基本参数并填入
  try {
    YAML::Node root = YAML::Load(content.toStdString());
    if (root["robot_name"])
      robot_name_edit_->setText(
          QString::fromStdString(root["robot_name"].as<std::string>()));
    if (root["dof"])
      dof_edit_->setText(
          QString::number(root["dof"].as<int>()));
    if (root["kinematic_prefix"])
      kinematic_prefix_edit_->setText(
          QString::number(root["kinematic_prefix"].as<int>()));
  } catch (const YAML::Exception &) {
    // 解析失败不阻止，用户可手动修正
  }

  status_label_->setText("已导入: " + path);
}

// ---------------------------------------------------------------------------
void MainWindow::onBrowseData() {
  QString path = QFileDialog::getOpenFileName(
      this, "选择数据 CSV", QString(),
      "CSV 文件 (*.csv);;所有文件 (*)");
  if (!path.isEmpty())
    data_file_edit_->setText(path);
}

// ---------------------------------------------------------------------------
void MainWindow::generateConfigFiles(const QString &workDir) {
  // 1. 写入 kinematic_params.yaml
  QString kin_yaml = yaml_editor_->toPlainText();
  QString kin_path = workDir + "/kinematic_params.yaml";
  {
    std::ofstream out(kin_path.toStdString());
    out << kin_yaml.toStdString();
  }

  // 2. 写入 identification.yaml
  QString id_path = workDir + "/identification.yaml";
  QString data_file = data_file_edit_->text();
  QString robot_name = robot_name_edit_->text();

  {
    std::ofstream out(id_path.toStdString());
    out << "robot: \"" << robot_name.toStdString() << "\"\n";
    out << "kinematic_params: \"" << kin_path.toStdString() << "\"\n";
    out << "algorithm: 0\n";
    out << "regularization: 1\n";
    out << "data_file: \"" << data_file.toStdString() << "\"\n";
    out << "output_file: \"" << (workDir + "/result.yaml").toStdString() << "\"\n";
  }
}

// ---------------------------------------------------------------------------
void MainWindow::onStartIdentification() {
  // 校验
  if (yaml_editor_->toPlainText().trimmed().isEmpty()) {
    QMessageBox::warning(this, "错误", "请输入或导入运动学参数");
    return;
  }
  if (data_file_edit_->text().trimmed().isEmpty()) {
    QMessageBox::warning(this, "错误", "请选择数据 CSV 文件");
    return;
  }

  start_btn_->setEnabled(false);
  status_label_->setText("正在辨识...");

  // 创建工作目录
  QString workDir = "./res";
  QDir().mkpath(workDir);

  generateConfigFiles(workDir);

  // 使用 --algo 命令行参数直接指定算法，数据文件也直接指定
  QString id_config = workDir + "/identification.yaml";
  QString data_file = data_file_edit_->text();
  QString algo = algo_combo_->currentText();

  QStringList args;
  args << "--config" << id_config
       << "--data" << data_file
       << "--algo" << algo;

  process_->start(identifyPath(), args);
}

// ---------------------------------------------------------------------------
void MainWindow::onProcessFinished(int exitCode, QProcess::ExitStatus status) {
  start_btn_->setEnabled(true);

  QString output = QString::fromUtf8(process_->readAllStandardOutput());
  QString err_output = QString::fromUtf8(process_->readAllStandardError());

  if (status == QProcess::NormalExit && exitCode == 0) {
    // 将结果复制到 res/ 目录下
    QString resultPath = "./res/identification.yaml";
    QFile::remove(resultPath);
    QFile::copy("./res/result.yaml", resultPath);

    // 提取 RMSE 用于显示
    QString display = "辨识成功！";
    for (const auto &line : output.split('\n')) {
      if (line.contains("RMSE:")) {
        display += "  " + line.trimmed();
        break;
      }
    }
    status_label_->setText(display);
  } else {
    status_label_->setText("辨识失败 (exit=" + QString::number(exitCode) + ")");
    QString detail = output + "\n" + err_output;
    if (!detail.trimmed().isEmpty()) {
      QMessageBox::warning(this, "辨识输出", detail);
    }
  }
}
