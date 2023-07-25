#include "QRobotInterface.h"
#include "ui_robotinterface.h"

RobotInterface::RobotInterface(AppModel *model, QWidget *parent)
  :QMainWindow(parent)
  ,ui(new Ui::RobotInterface)
{
  ui->setupUi(this);

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

  // http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals\
  // https://automaticaddison.com/how-to-send-goals-to-the-ros-navigation-stack-using-c/
  connect(ui->run_btn, &QPushButton::clicked, this, [=]()
  {
    for(int i = 0; i < slot_target.size(); i++){
      if(slot_target[i].name() != "+")
      {
        m_model->m_rosNode.set_goal(
          slot_target[i].name(),
          slot_target[i].x_axis().toDouble(),
          slot_target[i].y_axis().toDouble(),
          slot_target[i].z_axis().toDouble(),
          0
        );
        // waiting for goal success

        CONSOLE << slot_target[i].name();
      }
    }
  });

  connect(ui->remove_btn, &QPushButton::clicked, this, &RobotInterface::slotRemoveTarget);

  connect(ui->slotTarget_1, &QPushButton::clicked, this, [=](){
        CONSOLE << "slot 1";
        m_targetSelected = 1;
  });

  connect(ui->slotTarget_2, &QPushButton::clicked, this, [=](){
        CONSOLE << "slot 2";
        m_targetSelected = 2;
  });

  connect(ui->slotTarget_3, &QPushButton::clicked, this, [=](){
        CONSOLE << "slot 3";
        m_targetSelected = 3;
  });

  connect(ui->addTarget_btn, &QPushButton::clicked, this, [=]()
  {
    ui->stackedWidget->setCurrentIndex(1);
  });

  connect(ui->settingTarget_btn, &QPushButton::clicked, [=]() {
    RobotInterface::slot_settingTarget();
  });

  connect(ui->cancelWidget_btn, &QPushButton::clicked, this, [this] {
    ui->stackedWidget->setCurrentIndex(0);
  });

}

RobotInterface::~RobotInterface()
{
  delete ui;
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
      readSettings();
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
  // switch (status) {
  // case AppEnums::QRobotStatus::None: {
  //   this->ui->slot_robotStatus->setStyleSheet("background-color: red");
  // } 
  // break;
  // case AppEnums::QRobotStatus::Normal: {
  //   this->ui->slot_robotStatus->setStyleSheet("background-color: green");
  // }
  // break;
  // case AppEnums::QRobotStatus::Error: {
  //   this->ui->slot_robotStatus->setStyleSheet("background-color: blue");
  // }
  // break;
  // case AppEnums::QRobotStatus::Warning: {
  //   this->ui->slot_robotStatus->setStyleSheet("background-color: yellow");
  // }
  // break;
  // case AppEnums::QRobotStatus::NotReady: {
  //   this->ui->slot_robotStatus->setStyleSheet("background-color: gray");
  // }
  // break;
  // default:
  // break;
  // }
}

void RobotInterface::slot_updateRobotMissonStatus(AppEnums::QMissionStatus status)
{
// switch (status) {
//   case AppEnums::QMissionStatus::Idle: {
//    ui->robotMissonStatus->setStyleSheet("background-color: red");
//   } 
//   break;   
//   case AppEnums::QMissionStatus::Running: {
//     ui->robotMissonStatus->setStyleSheet("background-color: green");
//   }
//   break;
//   case AppEnums::QMissionStatus::Paused: {
//     ui->robotMissonStatus->setStyleSheet("background-color: blue");
//   }
//   break;
//   case AppEnums::QMissionStatus::Stopped: {
//     ui->robotMissonStatus->setStyleSheet("background-color: yellow");
//   }
//   break;
//   default:
//   break;
//   }
}

void RobotInterface::connections()
{
  connect(&m_model->m_rosNode, &QNode::rosShutdown, this, &RobotInterface::slot_rosShutdown);
  connect(&m_model->m_rosNode, &QNode::Master_shutdown, this, &RobotInterface::slot_rosShutdown);

  connect(m_timerCurrentTime, &QTimer::timeout, [=]() {
    ui->label_time->setText(
          QDateTime::currentDateTime().toString("  hh:mm:ss  "));
  });

  // Robot status
  connect(this, &RobotInterface::signalKeyPressed, m_model, &AppModel::keyRecieved);
  connect(m_model, &AppModel::signalRobotStatusChanged, this, &RobotInterface::slot_updateRobotStatus);

  // Robot Mission status
  connect(this, &RobotInterface::signalKeyPressed, m_model, &AppModel::keyMissonRecieved);
  connect(m_model, &AppModel::signalRobotMissionStatusChanged, this, &RobotInterface::slot_updateRobotMissonStatus);

  // Robot battery
  connect(&m_model->m_rosNode, &QNode::batteryState, this, &RobotInterface::slot_batteryState);

  //Battery percentage
  connect(&m_model->m_rosNode, &QNode::updateBatteryPercentage, this, &RobotInterface::slot_batteryPercentage);

  //Battery Voltage
  connect(&m_model->m_rosNode, &QNode::updateBatteryVoltage, this, &RobotInterface::slot_batteryVoltage);

  // Bind the speed control buttons
  connect(ui->back, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
  connect(ui->r_left, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
  connect(ui->stop, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
  connect(ui->forward, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
  connect(ui->r_right, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));

}

void RobotInterface::display_rviz()
{
  QSettings settings("junbot_gui", "Displays");
  bool Grid_enable = settings.value("Grid/enable", bool(true)).toBool();
  double Grid_count = settings.value("Grid/count", double(20)).toDouble();

  bool Map_enable = settings.value("Map/enable", bool(true)).toBool();
  QString Map_topic = settings.value("Map/topic", QString("/map")).toString();
  double Map_alpha = settings.value("Map/alpha", double(0.7)).toDouble();
  QString Map_scheme = settings.value("Map/scheme", QString("map")).toString();
  bool Laser_enable = settings.value("Laser/enable", bool(true)).toBool();
  QString Laser_topic =
      settings.value("Laser/topic", QString("/scan")).toString();
  bool Polygon_enable = settings.value("Polygon/enable", bool(true)).toBool();
  QString Polygon_topic =
      settings
      .value("Polygon/topic", QString("/move_base/local_costmap/footprint"))
      .toString();

  bool RobotModel_enable =
      settings.value("RobotModel/enable", bool(true)).toBool();
  bool Navigation_enable =
      settings.value("Navigation/enable", bool(true)).toBool();
  QString GlobalMap_topic =
      settings
      .value("Navigation/GlobalMap/topic",
             QString("/move_base/global_costmap/costmap"))
      .toString();
  QString GlobalMap_paln = settings
      .value("Navigation/GlobalPlan/topic",
             QString("/move_base/NavfnROS/plan"))
      .toString();
  QString LocalMap_topic =
      settings
      .value("Navigation/LocalMap/topic",
             QString("/move_base/local_costmap/costmap"))
      .toString();
  QString LocalMap_plan =
      settings
      .value("Navigation/LocalPlan/topic",
             QString("/move_base/DWAPlannerROS/local_plan"))
      .toString();
}

void RobotInterface::keyPressEvent(QKeyEvent *event)
{
    qDebug() << event->key();

    if(event->key() == Qt::Key_A)
    {
        emit signalKeyPressed(1);
    }
        if(event->key() == Qt::Key_S)
    {
        emit signalKeyPressed(2);
    }
        if(event->key() == Qt::Key_D)
    {
        emit signalKeyPressed(3);
    }
        if(event->key() == Qt::Key_F)
    {
        emit signalKeyPressed(4);
    }
        if(event->key() == Qt::Key_G)
    {
        emit signalKeyPressed(5);
    }

        if(event->key() == Qt::Key_Z)
    {
        emit signalKeyPressed(6);
    }
        if(event->key() == Qt::Key_X)
    {
        emit signalKeyPressed(7);
    }
        if(event->key() == Qt::Key_C)
    {
        emit signalKeyPressed(8);
    }
        if(event->key() == Qt::Key_V)
    {
        emit signalKeyPressed(9);
    }
}

void RobotInterface::slot_settingTarget()
{
  m_addNewTarget = new QAddNewTarget(m_model);
  m_addNewTarget->show();
}
