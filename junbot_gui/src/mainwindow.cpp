#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QGraphicsScene>
#include <QCheckBox>
#include <QMessageBox>
#include <QVector>

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
  : QMainWindow(parent)
  , ui(new Ui::MainWindow)
  , m_qnode(argc, argv)
{
  ui->setupUi(this);

  // read configuration file
  readSettings();
  initUis();
  setWindowIcon(QIcon(":/image/data/images/robot.png"));
  setWindowFlags(Qt::CustomizeWindowHint);  // remove title bar

  ui->view_logging->setModel(m_qnode.loggingModel());

  // Setting signal and slot
  connections();

}

MainWindow::~MainWindow()
{
  if (m_qgraphicsScene) {
    delete m_qgraphicsScene;
  }
  delete ui;
}

void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

void MainWindow::initUis()
{
  // Time dynamic display
  m_timerCurrentTime = new QTimer;
  m_timerCurrentTime->setInterval(100);
  m_timerCurrentTime->start();
  // ui.centralwidget->hide();

  // view scene loading
  m_qgraphicsScene = new QGraphicsScene();

  m_qgraphicsScene->clear();

  m_roboItem = new QRobotItem();

  m_qgraphicsScene->addItem(m_roboItem);

  ui->mapViz->setScene(m_qgraphicsScene);
  QImage Image;
  Image.load(":/image/data/images/car/background.jpg");
  QPixmap pixmap(QPixmap::fromImage(Image));
  ui->label_carback->setMinimumSize(ui->label_carback->size());
  pixmap.scaled(ui->label_carback->size(), Qt::KeepAspectRatio);

  // ui->label->setScaledContents(true);
  ui->label_carback->setAlignment(Qt::AlignCenter);
  ui->label_carback->setPixmap(pixmap);
  ui->horizontalLayout_4->setSpacing(0);
  ui->horizontalLayout_4->setMargin(0);

  // Icon setting
  ui->pushButton_status->setIcon(QIcon(":/image/data/images/status/status_none.png"));
  ui->settings_btn->setIcon(QIcon(":/image/data/images/toolbar_settings.png"));
  ui->min_btn->setIcon(QIcon(":/image/data/images/min.png"));
  ui->max_btn->setIcon(QIcon(":/image/data/images/max.png"));
  ui->close_btn->setIcon(QIcon(":/image/data/images/close.png"));
  ui->btn_map->setIcon(QIcon(":/image/data/images/toolbar_map.png"));
  ui->btn_control->setIcon(QIcon(":/image/data/images/control.png"));
  ui->btn_status->setIcon(QIcon(":/image/data/images/status.png"));
  ui->btn_other->setIcon(QIcon(":/image/data/images/toolbar_other.png"));

  // Rviz Widget (QGraphicView)
  ui->widget_rviz->hide();

  // dashboard
  if (m_showMode == AppEnums::QDisplayMode::Robot) {
    ui->stackedWidget_left->hide();
    ui->btn_status->hide();
    ui->btn_control->hide();
    ui->settings_btn->hide();
    this->showFullScreen();
  } else {
    QSettings windows_setting("junbot_gui", "windows");
    int x = windows_setting.value("WindowGeometry/x").toInt();
    int y = windows_setting.value("WindowGeometry/y").toInt();
    int width = windows_setting.value("WindowGeometry/width").toInt();
    int height = windows_setting.value("WindowGeometry/height").toInt();
    QDesktopWidget *desktopWidget = QApplication::desktop();
    QRect clientRect = desktopWidget->availableGeometry();
    QRect targRect0 = QRect(clientRect.width() / 4, clientRect.height() / 4,
                            clientRect.width() / 2, clientRect.height() / 2);
    QRect targRect = QRect(x, y, width, height);
    if (width == 0 || height == 0 || x < 0 || x > clientRect.width() || y < 0 ||
        y > clientRect
        .height())  // If the window position was abnormal when the software was closed last time, it will be displayed in the center of the display this time.
    {
      targRect = targRect0;
    }
    this->setGeometry(targRect);  // Set the size of the main window
  }
}

// Video topic init
void MainWindow::initVideos()
{
  QSettings video_topic_setting("junbot_gui", "settings");
  QStringList names = video_topic_setting.value("video/names").toStringList();
  QStringList topics = video_topic_setting.value("video/topics").toStringList();
  if (topics.size() == 4) {
    if (topics[0] != "")
    {
      m_qnode.Sub_Image(topics[0], 0);
    }
    if (topics[1] != "")
    {
      m_qnode.Sub_Image(topics[1], 1);
    }
    if (topics[2] != "")
    {
      m_qnode.Sub_Image(topics[2], 2);
    }
    if (topics[3] != "")
    {
      m_qnode.Sub_Image(topics[3], 3);
    }
  }

  connect(&m_qnode, &QNode::Show_image, this,
          &MainWindow::slot_show_image);
}

// Read and display topic list
void MainWindow::initTopicList()
{
  ui->topic_listWidget->clear();
  ui->topic_listWidget->addItem(QString("%1   (%2)").arg("Name", "Type"));
  QMap<QString, QString> topic_list = m_qnode.get_topic_list();
  for (QMap<QString, QString>::iterator iter = topic_list.begin();
       iter != topic_list.end(); iter++) {
    ui->topic_listWidget->addItem(
          QString("%1   (%2)").arg(iter.key(), iter.value()));
  }
}

// Other setting
void MainWindow::initOthers()
{
  m_timerChart = new QTimer;
  m_timerPubImageMap = new QTimer;
  m_timerPubImageMap->setInterval(100);
  m_timerChart->setInterval(100);

  connect(m_timerPubImageMap, SIGNAL(timeout()), this,
          SLOT(slot_pubImageMapTimeOut()));

  m_timerPubImageMap->start();
  m_timerChart->start();
}

bool MainWindow::connectMaster(QString master_ip, QString ros_ip, bool use_envirment)
{

  if (use_envirment) {
    if (!m_qnode.init()) {
      return false;
    } else {
      initVideos();
      initTopicList();
      initOthers();
    }
  }
  else {
    if (!m_qnode.init(master_ip.toStdString(), ros_ip.toStdString())) {
      return false;
    } else {
      initVideos();
      initTopicList();
      initOthers();
    }
  }
  readSettings();
  return true;
}


void MainWindow::slot_batteryState(sensor_msgs::BatteryState msg)
{
  ui->label_power->setText(QString::number(msg.voltage).mid(0, 5) + "V");
  double percentage = msg.percentage;
  ui->progressBar->setValue(percentage > 100 ? 100 : percentage);

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

void MainWindow::slot_rosShutdown()
{
  slot_updateRobotStatus(AppEnums::QRobotStatus::None);
}

void MainWindow::refreshTopicList()
{
  initTopicList();
}

void MainWindow::Slider_raw_valueChanged(int value)
{
  ui->label_raw->setText(QString::number(value));
}

void MainWindow::Slider_linear_valueChanged(int value)
{
  ui->label_linear->setText(QString::number(value));
}

// Speed control related button processing slot function
void MainWindow::slot_cmd_control()
{
  QPushButton *btn = qobject_cast<QPushButton *>(sender());
  char key = btn->text().toStdString()[0];
  QString button_key = btn->objectName();

  float liner = ui->horizontalSlider_linear->value() * 0.01;
  float turn = ui->horizontalSlider_raw->value() * 0.01;
  bool is_all = false;

  CONSOLE << "Key: " << key;
  CONSOLE << "Name: " << button_key;

  if(button_key == "forward"){
    // forward
    m_qnode.move_base(is_all ? 'I' : 'i', liner, turn);
  } else if(button_key == "back"){
    // backward
    m_qnode.move_base(is_all ? '<' : ',', liner, turn);
  } else if (button_key == "r_left") {
    // rotate left
    m_qnode.move_base(is_all ? 'J' : 'j', liner, turn);
  } else if (button_key == "r_right") {
    // rotate right
    m_qnode.move_base(is_all ? 'L' : 'l', liner, turn);
  } else {
    // stop
    m_qnode.move_base(is_all ? 'K' : 'k', liner, turn);
  }
}

void MainWindow::slot_set_2D_Goal()
{
  emit signalSet2DGoal();
}

void MainWindow::slot_set_2D_Pos()
{
  emit signalSet2DPose();
}

void MainWindow::slot_set_select()
{
  // pending
}

void MainWindow::slot_move_camera_btn()
{
  emit signalSetMoveCamera();
}

void MainWindow::slot_setting_frame()
{
  // pending
}

void MainWindow::slot_set_mutil_goal_btn()
{
  // pending
}

void MainWindow::slot_return_point()
{
  // pending
}

void MainWindow::slot_position_change(QString frame, double x, double y, double z, double w)
{
  // pending
}


void MainWindow::slot_show_image(int frame_id, QImage image)
{
  switch (frame_id) {
  case 2:
    ui->label_video2->setPixmap(QPixmap::fromImage(image).scaled(
                                  ui->label_video2->width(), ui->label_video2->height()));
    break;
  case 3:
    ui->label_video3->setPixmap(QPixmap::fromImage(image).scaled(
                                  ui->label_video3->width(), ui->label_video3->height()));
    break;
  }
}

void MainWindow::slot_dis_connect()
{
  ros::shutdown();
  slot_rosShutdown();
  emit signalDisconnect();
  this->close();
}

void MainWindow::slot_hide_table_widget()
{
  if (ui->stackedWidget_left->isHidden()) {
    ui->stackedWidget_left->show();
  } else {
    ui->stackedWidget_left->hide();
  }
}

void MainWindow::slot_pubImageMapTimeOut()
{
  QImage image(600, 600, QImage::Format_RGB888);
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing);
  m_qgraphicsScene->render(&painter);
  m_qnode.pub_imageMap(image);
}

void MainWindow::slot_updateCursorPos(QPointF pos)
{
  QPointF mapPos = m_qnode.transScenePoint2Word(pos);
  ui->label_pos_map->setText("x: " + QString::number(mapPos.x()).mid(0, 4) +
                             "  y: " + QString::number(mapPos.y()).mid(0, 4));
  ui->label_pos_scene->setText("x: " + QString::number(pos.x()).mid(0, 4) +
                               "  y: " + QString::number(pos.y()).mid(0, 4));
}

void MainWindow::slot_changeMapType(int index)
{
  switch (index) {
  case 0:
    ui->widget_rviz->hide();
    ui->mapViz->show();
    break;
  case 1:
    ui->mapViz->hide();
    ui->widget_rviz->show();
    break;
  }
}

void MainWindow::slot_updateRobotStatus(AppEnums::QRobotStatus status)
{
  switch (status) {
  case AppEnums::QRobotStatus::None: {
    QTimer::singleShot(100, [this]() {
      ui->pushButton_status->setIcon(
            QIcon(":/image/data/images/status/status_none.png"));
      m_roboItem->setRobotColor(AppEnums::QRobotColor::Blue);
    });
  } break;
  case AppEnums::QRobotStatus::Normal: {
    QTimer::singleShot(200, [this]() {
      ui->pushButton_status->setIcon(
            QIcon(":/image/data/images/status/status_normal.png"));
      m_roboItem->setRobotColor(AppEnums::QRobotColor::Blue);
    });
  } break;
  case AppEnums::QRobotStatus::Error: {
    QTimer::singleShot(300, [this]() {
      ui->pushButton_status->setIcon(
            QIcon(":/image/data/images/status/status_error.png"));
      m_roboItem->setRobotColor(AppEnums::QRobotColor::Red);
    });
  } break;
  case AppEnums::QRobotStatus::Warning: {
    QTimer::singleShot(400, [this]() {
      ui->pushButton_status->setIcon(
            QIcon(":/image/data/images/status/status_warn.png"));
      m_roboItem->setRobotColor(AppEnums::QRobotColor::Yellow);
    });
  } break;
  }
}

void MainWindow::connections()
{
  connect(&m_qnode, &QNode::loggingUpdated, this, &MainWindow::updateLoggingView);
  connect(&m_qnode, &QNode::rosShutdown, this, &MainWindow::slot_rosShutdown);
  connect(&m_qnode, &QNode::Master_shutdown, this, &MainWindow::slot_rosShutdown);

  // Main display screen 

  // Control screen
  connect(ui->btn_control, &QPushButton::clicked, [=]() {
    ui->stackedWidget_left->setCurrentIndex(1);
    setCurrentMenu(ui->btn_control);
  });

  // Status screen
  connect(ui->btn_status, &QPushButton::clicked, [=]() {
    ui->stackedWidget_left->setCurrentIndex(0);
    setCurrentMenu(ui->btn_status);
  });

  // Map screen
  connect(ui->btn_map, &QPushButton::clicked, [=]() {
    ui->stackedWidget_main->setCurrentIndex(0);
    setCurrentMenu(ui->btn_map);
  });

  // Other screen
  connect(ui->btn_other, &QPushButton::clicked, [=]() {
    ui->stackedWidget_main->setCurrentIndex(1);
    setCurrentMenu(ui->btn_other);
  });

  connect(ui->pushButton_status, &QPushButton::clicked, [=]() {
    ui->btn_other->click();
  });

  connect(m_timerCurrentTime, &QTimer::timeout, [=]() {
    ui->label_time->setText(
          QDateTime::currentDateTime().toString("  hh:mm:ss  "));
  });

  // Robot status
  connect(&m_qnode, &QNode::updateRobotStatus, this, &MainWindow::slot_updateRobotStatus);

  // Robot battery
  connect(&m_qnode, &QNode::batteryState, this, &MainWindow::slot_batteryState);

  // Function to bind slider
  connect(ui->horizontalSlider_raw, SIGNAL(valueChanged(int)), this,
          SLOT(Slider_raw_valueChanged(int)));
  connect(ui->horizontalSlider_linear, SIGNAL(valueChanged(int)), this,
          SLOT(Slider_linear_valueChanged(int)));

  // Set interface
  connect(ui->settings_btn, SIGNAL(clicked()), this, SLOT(slot_setting_frame()));

  // Bind the speed control buttons
  connect(ui->back, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
  connect(ui->r_left, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
  connect(ui->stop, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
  connect(ui->forward, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
  connect(ui->r_right, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));


  connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(slot_dis_connect()));
  connect(ui->set_pos_btn, SIGNAL(clicked()), this, SLOT(slot_set_2D_Pos()));
  connect(ui->set_goal_btn, SIGNAL(clicked()), this, SLOT(slot_set_2D_Goal()));

  // return flight
  connect(ui->return_btn, SIGNAL(clicked()), this, SLOT(slot_return_point()));

  // refresh thread list
  connect(ui->refresh_topic_btn, SIGNAL(clicked()), this,
          SLOT(refreshTopicList()));

  connect(ui->close_btn, SIGNAL(clicked()), this, SLOT(slot_closeWindows()));
  connect(ui->min_btn, SIGNAL(clicked()), this, SLOT(slot_minWindows()));
  connect(ui->max_btn, SIGNAL(clicked()), this, SLOT(slot_maxWindows()));

  // Map and Path display 
  connect(&m_qnode, SIGNAL(updateMap(QImage)), m_roboItem,
          SLOT(paintMaps(QImage)));
  connect(&m_qnode, SIGNAL(plannerPath(QPolygonF)), m_roboItem,
          SLOT(paintPlannerPath(QPolygonF)));
  connect(&m_qnode, SIGNAL(updateRoboPose(QRobotPose)), m_roboItem,
          SLOT(paintRoboPos(QRobotPose)));
  connect(&m_qnode, SIGNAL(updateLaserScan(QPolygonF)), m_roboItem,
          SLOT(paintLaserScan(QPolygonF)));
  connect(m_roboItem, SIGNAL(cursorPos(QPointF)), this,
          SLOT(slot_updateCursorPos(QPointF)));

  // map
  connect(m_roboItem, SIGNAL(signalPub2DPos(QRobotPose)), &m_qnode,
          SLOT(slot_pub2DPos(QRobotPose)));
  connect(m_roboItem, SIGNAL(signalPub2DGoal(QRobotPose)), &m_qnode,
          SLOT(slot_pub2DGoal(QRobotPose)));
  connect(this, SIGNAL(signalSet2DPose()), m_roboItem, SLOT(slot_set2DPos()));
  connect(this, SIGNAL(signalSet2DGoal()), m_roboItem, SLOT(slot_set2DGoal()));
  connect(this, SIGNAL(signalSetMoveCamera()), m_roboItem,
          SLOT(slot_setMoveCamera()));
  //    connect(ui.stackedWidget_2,SIGNAL())
}

void MainWindow::display_rviz()
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

// Setting menu bar => change display? 
void MainWindow::setCurrentMenu(QPushButton *cur_btn)
{
  for (int i = 0; i < ui->horizontalLayout_menu->layout()->count(); i++) {
    QPushButton *btn = qobject_cast<QPushButton *>(
          ui->horizontalLayout_menu->itemAt(i)->widget());
    if (btn == cur_btn) {
      cur_btn->setStyleSheet(
            " QPushButton{ background-color:rgb(67, 154, 246); border:none;  "
            "padding:0px 0px 0px 0px; margin:0px 0px 0px 0px;}");
    } else {
      btn->setStyleSheet(
            "QPushButton:hover{background-color:rgb(186, 189, 182); "
            "border-bottom:2px solid rgb(67, 154, 246);}"
            "QPushButton:checked{ background-color:cyan;border-bottom:2px solid "
            "white }"
            "QPushButton:pressed{background-color:rgb(67, 154, 246)}"
            " QPushButton{ background-color:rgb(238, 238, 236); border:none;  "
            "padding:0px 0px 0px 0px; margin:0px 0px 0px 0px; }");
    }
  }
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() { ui->view_logging->scrollToBottom(); }

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  // QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program
  // 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about
  // description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::readSettings() {
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
  if (settings.value("main/show_mode", "control").toString() == "control") {
    m_showMode = AppEnums::QDisplayMode::Control;
  } else {
    m_showMode = AppEnums::QDisplayMode::Robot;
  }
}

void MainWindow::writeSettings() {
  QSettings windows_setting("junbot_gui", "windows");
  windows_setting.clear();
  windows_setting.setValue("WindowGeometry/x", this->x());
  windows_setting.setValue("WindowGeometry/y", this->y());
  windows_setting.setValue("WindowGeometry/width", this->width());
  windows_setting.setValue("WindowGeometry/height", this->height());
}
void MainWindow::mousePressEvent(QMouseEvent *event) {
  m_lastPos = event->globalPos();
  isPressedWidget = true;
}

void MainWindow::mouseMoveEvent(QMouseEvent *event) {
  if (isPressedWidget) {
    this->move(this->x() + (event->globalX() - m_lastPos.x()),
               this->y() + (event->globalY() - m_lastPos.y()));
    m_lastPos = event->globalPos();
  }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event) {
  m_lastPos = event->globalPos();
  isPressedWidget = false;
}

void MainWindow::slot_closeWindows() { this->close(); }
void MainWindow::slot_minWindows() { this->showMinimized(); }
void MainWindow::slot_maxWindows() {
  if (this->isFullScreen()) {
    this->showNormal();
  } else {
    this->showFullScreen();
  }
}
void MainWindow::closeEvent(QCloseEvent *event) {
  writeSettings();
  QMainWindow::closeEvent(event);
}
