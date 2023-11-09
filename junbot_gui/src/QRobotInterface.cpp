#include "QRobotInterface.h"
#include "ui_robotinterface.h"

RobotInterface::RobotInterface(AppModel *model, QWidget *parent)
  :QMainWindow(parent)
  ,ui(new Ui::RobotInterface)
{
  ui->setupUi(this);

  this->setWindowTitle("VMCBot");

  slot_state.resize(3);
  slot_target.resize(3);

  for(int i = 0; i < 3; i++)
  {
    slot_target[i] = QDeliveryTarget("+");
  }

  slot_state.append(false); // slot 1
  slot_state.append(false); // slot 2
  slot_state.append(false); // slot 3

  m_model = model;

  // read configuration file
  readSettings();
  initUis();

  // Setting signal and slot
  connections();

  // disconnect button
  connect(ui->disconnect_button, &QPushButton::clicked, [=]() {
      RobotInterface::slot_dis_connect();
  });

  ui->stackedWidget->setCurrentIndex(0);

  QGroupBox *m_groupBox = new QGroupBox();
  m_groupBox = ui->groupBox;
  m_groupBox->setTitle("Delivery Target");
  int m = m_model->m_targets.size();

  m_targetButton.resize(m);

  // QPushButton *buttons[m];

  QGridLayout *layout = new QGridLayout();
  int n = m;
  while(n > 0){
    for(int i = m - n; i < m - n + 4 & i < m; i++){
      m_targetButton[i] = new QPushButton(m_model->m_targets[i].name());
      m_targetButton[i]->setFixedHeight(60);
      layout->addWidget(m_targetButton[i], (m-n)/4, i-(m-n));
    }
    n = n - 4;
  }

  m_groupBox->setLayout(layout);

for(int i = 0; i < m_targetButton.size(); i++){
  connect(m_targetButton[i], &QPushButton::clicked, [=]{
      CONSOLE << "add target " << m_model->m_targets[i].name();
      updateTargetSlot(m_model->m_targets[i]);
  });
}

  connect(ui->run_btn, &QPushButton::clicked, this, &RobotInterface::slotRun);

  connect(ui->remove_btn, &QPushButton::clicked, this, &RobotInterface::slotRemoveTarget);

  connect(ui->slotTarget_1, &QPushButton::clicked, this, [=](){
        CONSOLE << "slot 3";
        m_targetSelected = 1;
  });

  connect(ui->slotTarget_2, &QPushButton::clicked, this, [=](){
        CONSOLE << "slot 2";
        m_targetSelected = 2;
  });

  connect(ui->slotTarget_3, &QPushButton::clicked, this, [=](){
        CONSOLE << "slot 1";
        m_targetSelected = 3;
  });

  connect(ui->addTarget_btn, &QPushButton::clicked, this, [=]()
  {
    if(m_model->getCurrentUserType() == "admin")
        {
            ui->settingTarget_btn->setVisible(true);
            ui->stackedWidget->setCurrentIndex(1);
            emit updateControllingStatus(0);
            emit updateMissionStatus(1);

        }
        else 
        {
            ui->settingTarget_btn->setVisible(false);
            ui->stackedWidget->setCurrentIndex(1);
            emit updateControllingStatus(0);
            emit updateMissionStatus(1);

        }
  });

  connect(ui->settingTarget_btn, &QPushButton::clicked, [=]() {
    RobotInterface::slot_settingTarget();
  });

  connect(ui->cancelWidget_btn, &QPushButton::clicked, this, [this] {
    ui->stackedWidget->setCurrentIndex(0);
    emit updateControllingStatus(1);
    emit updateMissionStatus(0);
    m_model->m_rosNode.cancel_goal();

  });

  emit updateControllingStatus(1);
  emit updateMissionStatus(0);

  CONSOLE << "Robot Interface inited";
}

RobotInterface::~RobotInterface()
{
  delete ui;
}

// TODO: Update enable and disable in Mission UI
void RobotInterface::slotRun()
{ 
  std::vector<QRobotPose> goals;
  std::vector<int> goals_Id;
  for(int i = 0; i < slot_target.size(); i++){
    
    if(slot_target[i].name() == "+") continue;

    QRobotPose goal = {slot_target[i].x_axis().toDouble(),
      slot_target[i].y_axis().toDouble(),
      slot_target[i].z_axis().toDouble(),
      1
      };
    goals.push_back(goal);

    // TODO: add id to goals
    goals_Id.push_back(i + 1);
  }

  CONSOLE << goals.size();

  bool check = false;

  check = m_model->m_rosNode.set_multi_goal("Frame", goals, goals_Id);
}

void RobotInterface::runNextTarget()
{
  m_model->m_rosNode.sendNextTarget();
}

void RobotInterface::slotRemoveTarget()
{
  if(m_targetSelected == NULL)
  {
    return;
  }
  removeTarget(m_targetSelected);
}

void RobotInterface::removeTarget(int i)
{
  CONSOLE << "Remove " << i;
  if(i == 1 && (ui->slotTarget_1->text() != "+"))
  {
    slot_target[3 - i] = QDeliveryTarget("+"); 
    updateTagerSlotUI();
  }
  else if(i == 2 && (ui->slotTarget_2->text() != "+"))
  {
    slot_target[3 - i] = QDeliveryTarget("+");
    updateTagerSlotUI();
  }
  else if(i == 3 && (ui->slotTarget_3->text() != "+"))
  {
    CONSOLE << "Bug?";
    slot_target[3 - i] = QDeliveryTarget("+");
    CONSOLE << slot_target[i - 1].name();
    updateTagerSlotUI();
  }

  m_targetSelected = NULL;
}

void RobotInterface::updateTagerSlotUI()
{
    CONSOLE << "Bug?";
    QVector<QDeliveryTarget> tmp_slot_target;

    // tmp_slot_target.resize(3);

    for(int i = 0; i < 3; i++)
    {
      if(slot_target[i].name() != "+")
      {
        tmp_slot_target.append(slot_target[i]);
        CONSOLE << i << slot_target[i].name();
      }
    }

    slot_target.clear();
    slot_target.resize(3);

    int cnt = 0;

    CONSOLE << tmp_slot_target.size();

    for(int i = 0; i < tmp_slot_target.size(); i++)
    {
      if(tmp_slot_target[i].name() != "+")
      {
        slot_target[cnt] = tmp_slot_target[i];
        cnt++;
      }
    }

    CONSOLE << cnt;

    for(int i = cnt; i < 3; i++)
    {
      slot_target[i] = QDeliveryTarget("+");
    }

    ui->slotTarget_3->setText(slot_target[0].name());
    ui->slotTarget_2->setText(slot_target[1].name());
    ui->slotTarget_1->setText(slot_target[2].name());
}

void RobotInterface::updateTargetSlot(QDeliveryTarget target)
{
  for(int i = 0; i < 3; i++)
  {
      if(slot_target[i].name() == "+")
      {
        slot_target[i] = target;
        ui->slotTarget_3->setText(slot_target[0].name());
        ui->slotTarget_2->setText(slot_target[1].name());
        ui->slotTarget_1->setText(slot_target[2].name());
        return;
      }
  }
}

void RobotInterface::readSettings()
{
  QSettings settings("junbot_gui", "settings");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  m_masterUrl =
      settings.value("connect/master_url", QString("http://192.168.1.2:11311/"))
          .toString();
  m_hostUrl =
      settings.value("connect/host_url", QString("192.168.1.3")).toString();
  m_useEnviorment =
      settings.value("connect/use_enviorment", bool(false)).toBool();
  m_autoConnect = settings.value("connect/auto_connect", bool(false)).toBool();
  m_turnLightThre =
      settings.value("connect/lineEdit_turnLightThre", double(0.1)).toDouble();
}

void RobotInterface::writeSettings()
{
  QSettings windows_setting("junbot_gui", "windows");
  windows_setting.clear();
  windows_setting.setValue("WindowGeometry/x", this->x());
  windows_setting.setValue("WindowGeometry/y", this->y());
  windows_setting.setValue("WindowGeometry/width", this->width());
  windows_setting.setValue("WindowGeometry/height", this->height());
}

void RobotInterface::closeEvent(QCloseEvent *event)
{
  writeSettings();
  QMainWindow::closeEvent(event);
}

void RobotInterface::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

void RobotInterface::initUis()
{
  // Time dynamic display
  m_timerCurrentTime = new QTimer;
  m_timerCurrentTime->setInterval(100);
  m_timerCurrentTime->start();


  m_roboItem = new QRobotItem();
}

bool RobotInterface::connectMaster(QString master_ip, QString ros_ip)
{
  CONSOLE << "Connect ?";

  if (!m_model->connectMaster(master_ip, ros_ip)) {
      CONSOLE << "Connect fail";
      return false;
  } 
  else 
  {
      // readSettings();
      CONSOLE << "ROS Master Connected";
      return true;
  }
}

void RobotInterface::slot_batteryState(sensor_msgs::BatteryState msg)
{
  // ui->label_power->setText(QString::number(msg.voltage).mid(0, 5) + "V");
  double percentage = msg.percentage;
  // ui->progressBar->setValue(percentage > 100 ? 100 : percentage);

  if (percentage <= 20) {
    ui->progressBar->setStyleSheet(
          "QProgressBar::chunk {background-color: red;width: 20px;} QProgressBar "
          "{border: 2px solid grey;border-radius: 5px;text-align: center;}");
  }
  else {
    ui->progressBar->setStyleSheet(
          "QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: "
          "center;}");
  }
}

void RobotInterface::slot_batteryPercentage(float msg)
{
  CONSOLE << msg;
  ui->progressBar->setValue(msg);
}

void RobotInterface::slot_batteryVoltage(float msg)
{
  QString message = QString::number(msg, 'f', 2);
  ui->label_power->setText(message + "V");
}

void RobotInterface::slot_rosShutdown()
{
  slot_updateRobotStatus(AppEnums::QRobotStatus::None);
}

void RobotInterface::slot_cmd_control()
{
  QPushButton *btn = qobject_cast<QPushButton *>(sender());
  char key = btn->text().toStdString()[0];
  QString button_key = btn->objectName();

  float liner = ui->horizontalSlider_linear->value() * 0.005;
  QString liner_text = QString::number(liner);
  ui->liner_label->setText(liner_text);
  float turn = ui->horizontalSlider_turn->value() * 0.005;
  QString turner_text = QString::number(turn);
  ui->turner_label->setText(turner_text);
  bool is_all = false;

  CONSOLE << "Key: " << key;
  CONSOLE << "Name: " << button_key;

  if(button_key == "forward"){
    // forward
    m_model->m_rosNode.move_base(is_all ? 'I' : 'i', liner, turn);
    CONSOLE << "my forward: " << liner;
  } else if(button_key == "back"){
    // backward
    m_model->m_rosNode.move_base(is_all ? '<' : ',', liner, turn);
    CONSOLE << "my backward: " << liner;
  } else if (button_key == "r_left") {
    // rotate left
    m_model->m_rosNode.move_base(is_all ? 'J' : 'j', liner, turn);
    CONSOLE << "left_value: ";
  } else if (button_key == "r_right") {
    // rotate right
    m_model->m_rosNode.move_base(is_all ? 'L' : 'l', liner, turn);
    CONSOLE << "right_value: ";
  } else {
    // stop
    m_model->m_rosNode.move_base(is_all ? 'K' : 'k', liner, turn);
  }
}

void RobotInterface::slot_dis_connect()
{

  ros::shutdown();
  slot_rosShutdown();
  emit signalDisconnect();
  this->close();
}

void RobotInterface::slot_updateRobotStatus(AppEnums::QRobotStatus status)
{
  switch (status) {
  case AppEnums::QRobotStatus::None: {
    this->ui->robot_status->setStyleSheet("border-image: url(:/image/data/images/status/status_none.png);");
  } 
  break;
  case AppEnums::QRobotStatus::Ready: {
    this->ui->robot_status->setStyleSheet("border-image: url(:/image/data/images/status/status_normal.png);");
  }
  break;
  case AppEnums::QRobotStatus::NotReady: {
    this->ui->robot_status->setStyleSheet("border-image: url(:/image/data/images/status/status_error.png);");
    QMessageBox::information(this, "Notification", "Not ready to run", QMessageBox::Ok);
  }
  break;
  default:
  break;
  }
}

void RobotInterface::slot_updateRobotMissonStatus(AppEnums::QMissionStatus status)
{
switch (status) {
  case AppEnums::QMissionStatus::Idle: {
   ui->mission_status->setStyleSheet("border-image: url(:/image/data/images/robot_color.png);");
  } 
  break;   
  case AppEnums::QMissionStatus::Running: {
    ui->mission_status->setStyleSheet("border-image: url(:/image/data/images/robot_blue.png);");
  }
  break;
  case AppEnums::QMissionStatus::Paused: {
    ui->mission_status->setStyleSheet("border-image: url(:/image/data/images/robot_yellow.png);");
  }
  break;
  case AppEnums::QMissionStatus::Stopped: {
    ui->mission_status->setStyleSheet("border-image: url(:/image/data/images/robot_red.png);");
  }
  break;
  default:
  break;
  }
}

void RobotInterface::slot_obstacle(AppEnums::QObstacle status)
{
  switch (status){
    case AppEnums::QObstacle::Human: {
      QMessageBox::information(this, "Notification", "There is Human ahead", QMessageBox::Ok);
    }
    break;
    case AppEnums::QObstacle::Stuff: {
      QMessageBox::information(this, "Notification", "There is Stuff ahead", QMessageBox::Ok);
    }
    break;
    default:
    break;
  }
}

void RobotInterface::connections()
{
  connect(&m_model->m_rosNode, &QNode::rosShutdown, this, &RobotInterface::slot_rosShutdown);
  connect(&m_model->m_rosNode, &QNode::Master_shutdown, this, &RobotInterface::slot_rosShutdown);

  connect(m_timerCurrentTime, &QTimer::timeout, [=]() {
    ui->label_time->setText(
          QDateTime::currentDateTime().toString("  hh:mm:ss  "));
  });

  connect(&m_model->m_rosNode, &QNode::updateGoalReached, this, [=](){
    QMessageBox::information(NULL, "Notification",
                                 "Robot has arrived the target!",
                                 QMessageBox::Ok);

    emit acceptedTarget();  
  });

  connect(this, &RobotInterface::acceptedTarget, this, [=](){
    runNextTarget();
  });

  // Robot status
  // connect(this, &RobotInterface::signalKeyPressed, m_model, &AppModel::keyRecieved);
  // connect(m_model, &AppModel::signalRobotStatusChanged, this, &RobotInterface::slot_updateRobotStatus);
  
  // connect(m_model, &AppModel::signalRobotStateUpdate, this, &RobotInterface::slot_updateRobotStatus);

  // Robot Mission status
  // connect(this, &RobotInterface::signalKeyPressed, m_model, &AppModel::keyMissonRecieved);

  connect(m_model, &AppModel::signalRobotMissionStatusChanged, this, &RobotInterface::slot_updateRobotMissonStatus);

  // Obstacle status
  // connect(&m_model->m_rosNode, &QNode::obstacleUpdate, m_model, &AppModel::checkObstacle);
  // connect(m_model, &AppModel::obstacleUpdateUi, this, &RobotInterface::slot_obstacle);

  // Battery percentage
  // connect(m_model, &AppModel::signalBatteryPercentage, this, &RobotInterface::slot_batteryPercentage);

  // Sensor State
  connect(&m_model->m_rosNode, &QNode::updateSensorStatus, m_model, &AppModel::sensorStatus);

  // Controlling State
  connect(this, &RobotInterface::updateControllingStatus, m_model, &AppModel::controllingStatus);

  // Having Mission State
  connect(this, &RobotInterface::updateMissionStatus, m_model, &AppModel::havingMissionStatus);

  //Battery Voltage
  // connect(&m_model->m_rosNode, &QNode::updateBatteryVoltage, this, &RobotInterface::slot_batteryVoltage);

  connect(m_model, &AppModel::signalNeedCharge, this, [=](){
    QMessageBox::information(this, "Notification", "Battery Low. Need Charge.", QMessageBox::Ok);
  }); 

  // Bind the speed control buttons
  connect(ui->back, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
  connect(ui->r_left, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
  connect(ui->stop, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
  connect(ui->forward, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
  connect(ui->r_right, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));

}

void RobotInterface::slot_settingTarget()
{
  m_addNewTarget = new QAddNewTarget(m_model);
  m_addNewTarget->show();
}
