#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QGraphicsScene>
#include <QCheckBox>
#include <QMessageBox>
#include <QVector>

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
        : QMainWindow(parent), ui(new Ui::MainWindow)
        {
    ui->setupUi(this);

    // read configuration file
    readSettings();
    initUis();
    setWindowIcon(QIcon(":/image/data/images/robot.png"));
    setWindowFlags(Qt::CustomizeWindowHint);  // remove title bar

    m_model = new AppModel(argc, argv, this);

    ui->view_logging->setModel(m_model->m_rosNode.loggingModel());

    // Setting signal and slot
    connections();

}

MainWindow::~MainWindow() {
    if (m_qgraphicsScene) {
        delete m_qgraphicsScene;
    }
    delete ui;
}

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

void MainWindow::initUis() {
    // Time dynamic display
    m_timerCurrentTime = new QTimer;
    m_timerCurrentTime->setInterval(100);
    m_timerCurrentTime->start();

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

    // dashboard
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

// Video topic init
void MainWindow::initVideos() {
    QSettings video_topic_setting("junbot_gui", "settings");
    QStringList names = video_topic_setting.value("video/names").toStringList();
    QStringList topics = video_topic_setting.value("video/topics").toStringList();
    if (topics.size() == 4) {
        if (topics[0] != "") {
            m_model->m_rosNode.Sub_Image(topics[0], 0);
        }
        if (topics[1] != "") {
            m_model->m_rosNode.Sub_Image(topics[1], 1);
        }
        if (topics[2] != "") {
            m_model->m_rosNode.Sub_Image(topics[2], 2);
        }
        if (topics[3] != "") {
            m_model->m_rosNode.Sub_Image(topics[3], 3);
        }
    }

    connect(&m_model->m_rosNode, &QNode::Show_image, this,
            &MainWindow::slot_show_image);
}

// Read and display topic list
void MainWindow::initTopicList() {
    ui->topic_listWidget->clear();
    ui->topic_listWidget->addItem(QString("%1   (%2)").arg("Name", "Type"));
    QMap<QString, QString> topic_list = m_model->m_rosNode.get_topic_list();
    for (QMap<QString, QString>::iterator iter = topic_list.begin();
         iter != topic_list.end(); iter++) {
        ui->topic_listWidget->addItem(
                QString("%1   (%2)").arg(iter.key(), iter.value()));
    }
}

// Other setting
void MainWindow::initOthers() {
    m_timerChart = new QTimer;
    m_timerPubImageMap = new QTimer;
    m_timerPubImageMap->setInterval(100);
    m_timerChart->setInterval(100);

    connect(m_timerPubImageMap, SIGNAL(timeout()), this,
            SLOT(slot_pubImageMapTimeOut()));

    m_timerPubImageMap->start();
    m_timerChart->start();
}

bool MainWindow::connectMaster(QString master_ip, QString ros_ip, bool use_envirment) {

    if (use_envirment) {
        if (!m_model->m_rosNode.init()) {
            return false;
        } else {
            initVideos();
            initTopicList();
            initOthers();
        }
    } else {
        if (!m_model->m_rosNode.init(master_ip.toStdString(), ros_ip.toStdString())) {
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


void MainWindow::slot_batteryState(sensor_msgs::BatteryState msg) {
    ui->label_power->setText(QString::number(msg.voltage).mid(0, 5) + "V");
    double percentage = msg.percentage;
    ui->progressBar->setValue(percentage > 100 ? 100 : percentage);

    if (percentage <= 20) {
        ui->progressBar->setStyleSheet(
                "QProgressBar::chunk {background-color: red;width: 20px;} QProgressBar "
                "{border: 2px solid grey;border-radius: 5px;text-align: center;}");
    } else {
        ui->progressBar->setStyleSheet(
                "QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: "
                "center;}");
    }
}

void MainWindow::slot_rosShutdown() {
    slot_updateRobotStatus(AppEnums::QRobotStatus::None);
}

void MainWindow::refreshTopicList() {
    initTopicList();
}

void MainWindow::Slider_raw_valueChanged(int value) {
    ui->label_raw->setText(QString::number(value));
}

void MainWindow::Slider_linear_valueChanged(int value) {
    ui->label_linear->setText(QString::number(value));
}

// Speed control related button processing slot function
void MainWindow::slot_cmd_control() {
    QPushButton *btn = qobject_cast<QPushButton *>(sender());
    char key = btn->text().toStdString()[0];
    QString button_key = btn->objectName();

    float liner = ui->horizontalSlider_linear->value() * 0.01;
    float turn = ui->horizontalSlider_raw->value() * 0.01;
    bool is_all = false;

    if (button_key == "forward") {
        // forward
        m_model->m_rosNode.move_base(is_all ? 'I' : 'i', liner, turn);
    } else if (button_key == "back") {
        // backward
        m_model->m_rosNode.move_base(is_all ? '<' : ',', liner, turn);
    } else if (button_key == "r_left") {
        // rotate left
        m_model->m_rosNode.move_base(is_all ? 'J' : 'j', liner, turn);
    } else if (button_key == "r_right") {
        // rotate right
        m_model->m_rosNode.move_base(is_all ? 'L' : 'l', liner, turn);
    } else {
        // stop
        m_model->m_rosNode.move_base(is_all ? 'K' : 'k', liner, turn);
    }
}

void MainWindow::slot_set_2D_Goal() {
    emit signalSet2DGoal();
}

void MainWindow::slot_set_2D_Pos() {
    emit signalSet2DPose();
}

void MainWindow::slot_show_image(int frame_id, QImage image) {
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

void MainWindow::slot_dis_connect() {
    ros::shutdown();
    slot_rosShutdown();
    emit signalDisconnect();
    this->close();
}

void MainWindow::slot_pubImageMapTimeOut() {
    QImage image(600, 600, QImage::Format_RGB888);
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    m_qgraphicsScene->render(&painter);
    m_model->m_rosNode.pub_imageMap(image);
}

void MainWindow::slot_updateCursorPos(QPointF pos) {
    QPointF mapPos = m_model->m_rosNode.transScenePoint2Word(pos);
    ui->label_pos_map->setText("x: " + QString::number(mapPos.x()).mid(0, 4) +
                               "  y: " + QString::number(mapPos.y()).mid(0, 4));
    ui->label_pos_scene->setText("x: " + QString::number(pos.x()).mid(0, 4) +
                                 "  y: " + QString::number(pos.y()).mid(0, 4));
}

void MainWindow::slot_updateRobotStatus(AppEnums::QRobotStatus status) {
    switch (status) {
        case AppEnums::QRobotStatus::None: {
            QTimer::singleShot(100, [this]() {
                ui->pushButton_status->setIcon(
                        QIcon(":/image/data/images/status/status_none.png"));
                m_roboItem->setRobotColor(AppEnums::QRobotColor::Blue);
            });
        }
            break;
        case AppEnums::QRobotStatus::Normal: {
            QTimer::singleShot(200, [this]() {
                ui->pushButton_status->setIcon(
                        QIcon(":/image/data/images/status/status_normal.png"));
                m_roboItem->setRobotColor(AppEnums::QRobotColor::Blue);
            });
        }
            break;
        case AppEnums::QRobotStatus::Error: {
            QTimer::singleShot(300, [this]() {
                ui->pushButton_status->setIcon(
                        QIcon(":/image/data/images/status/status_error.png"));
                m_roboItem->setRobotColor(AppEnums::QRobotColor::Red);
            });
        }
            break;
        case AppEnums::QRobotStatus::Warning: {
            QTimer::singleShot(400, [this]() {
                ui->pushButton_status->setIcon(
                        QIcon(":/image/data/images/status/status_warn.png"));
                m_roboItem->setRobotColor(AppEnums::QRobotColor::Yellow);
            });
        }
            break;
    }
}

void MainWindow::connections() {
    connect(&m_model->m_rosNode, &QNode::loggingUpdated, this, &MainWindow::updateLoggingView);
    connect(&m_model->m_rosNode, &QNode::rosShutdown, this, &MainWindow::slot_rosShutdown);
    connect(&m_model->m_rosNode, &QNode::Master_shutdown, this, &MainWindow::slot_rosShutdown);

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
    connect(&m_model->m_rosNode, &QNode::updateRobotStatus, this, &MainWindow::slot_updateRobotStatus);

    // Robot battery
    connect(&m_model->m_rosNode, &QNode::batteryState, this, &MainWindow::slot_batteryState);

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
    connect(&m_model->m_rosNode, SIGNAL(updateMap(QImage)), m_roboItem,
            SLOT(paintMaps(QImage)));
    connect(&m_model->m_rosNode, SIGNAL(plannerPath(QPolygonF)), m_roboItem,
            SLOT(paintPlannerPath(QPolygonF)));
    connect(&m_model->m_rosNode, SIGNAL(updateRoboPose(QRobotPose)), m_roboItem,
            SLOT(paintRoboPos(QRobotPose)));
    connect(&m_model->m_rosNode, SIGNAL(updateLaserScan(QPolygonF)), m_roboItem,
            SLOT(paintLaserScan(QPolygonF)));
    connect(m_roboItem, SIGNAL(cursorPos(QPointF)), this,
            SLOT(slot_updateCursorPos(QPointF)));

    // map
    connect(m_roboItem, SIGNAL(signalPub2DPos(QRobotPose)), &m_model->m_rosNode,
            SLOT(slot_pub2DPos(QRobotPose)));
    connect(m_roboItem, SIGNAL(signalPub2DGoal(QRobotPose)), &m_model->m_rosNode,
            SLOT(slot_pub2DGoal(QRobotPose)));
    connect(this, SIGNAL(signalSet2DPose()), m_roboItem, SLOT(slot_set2DPos()));
    connect(this, SIGNAL(signalSet2DGoal()), m_roboItem, SLOT(slot_set2DGoal()));
}

// Setting menu bar => change display? 
void MainWindow::setCurrentMenu(QPushButton *cur_btn) {
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

void MainWindow::updateLoggingView()
{
    ui->view_logging->scrollToBottom();
}


void MainWindow::readSettings() {
    QSettings settings("junbot_gui", "settings");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
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

void MainWindow::slot_closeWindows()
{
    this->close();
}

void MainWindow::slot_minWindows()
{
    this->showMinimized();
}

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
