#include "QLoginWidget.h"
#include "AppConstants.h"
#include "ui_loginwidget.h"

#include <QCompleter>
#include <QDebug>
#include <QHostAddress>
#include <QMessageBox>
#include <QMovie>
#include <QNetworkInterface>
#include <QPainter>
#include <QPropertyAnimation>
#include <QStringListModel>

// TODO: Update image for login widget and createUser widget
QLoginWidget::QLoginWidget(int argc, char **argv, QWidget *parent)
        : QCustomMoveWidget(parent)
        , ui(new Ui::LoginWidget) {
    ui->setupUi(this);

    this->setAttribute(Qt::WA_TranslucentBackground);
    this->setWindowFlags(Qt::FramelessWindowHint);
    this->setWindowFlags(Qt::FramelessWindowHint);

    m_model = new AppModel(argc, argv);

    initSettings();

    QImage imgClose(":/image/data/images/close.png");

    QPixmap close_pix = QPixmap::fromImage(imgClose).scaled(ui->btnWinClose_4->size(), Qt::KeepAspectRatio,
                                                              Qt::SmoothTransformation);
    ui->btnWinClose_4->setIcon(close_pix);
    ui->btnWinClose_2->setIcon(close_pix);
    ui->btnWinClose_3->setIcon(close_pix);

    // check IP in local network
    foreach (QHostAddress address, QNetworkInterface::allAddresses()) {
        if (address.protocol() == QAbstractSocket::IPv4Protocol) {
            QString addre = address.toString();

            if (addre.split(".")[0] == "192") {
                m_qRosIp = addre;
                m_qMasterIp = "http://" + addre + ":11311";
            } else if (addre.split(".")[0] == "10") {
                m_qRosIp = addre;
                m_qMasterIp = "http://" + addre + ":11311";
            } else if (addre.split(".")[0] == "172") {
                m_qRosIp = addre;
                m_qMasterIp = "http://" + addre + ":11311";
            }
        }
    }

    ui->stackedWidget->setCurrentIndex(1);

    // *****Create User (widget_2)*****
    connect(ui->createButton_2, &QPushButton::clicked, this, &QLoginWidget::createUser);

    connect(ui->closeButton, &QPushButton::clicked, this, [this] {
        ui->stackedWidget->setCurrentIndex(1);
    });
        connect(ui->btnWinClose_2, &QPushButton::clicked, [=]() {
        this->close();
    });

    // *****Login (widget_1)*****
    connect(ui->createButton_3, &QPushButton::clicked, this, [this]{
        ui->comboBox->setVisible(false);
        ui->label_19->setVisible(false);
        ui->stackedWidget->setCurrentIndex(2);
    });
    connect(ui->loginButton_2, &QPushButton::clicked, [=]() {
        QLoginWidget::getLogindata();
    });
    connect(ui->btnWinClose_4, &QPushButton::clicked, [=]() {
        this->close();
    });

    connect(ui->createUserAsAdmin_btn, &QPushButton::clicked, this, [this]{
        ui->comboBox->setVisible(true);
        ui->label_19->setVisible(true);
        ui->stackedWidget->setCurrentIndex(2);
    });

    // *****Set ip for ROS master connection*****
    connect(ui->pushButton_auto, &QPushButton::clicked, [=]() {
        ui->lineEditMasterIp->setText(m_qMasterIp);
        ui->lineEditRosIp->setText(m_qRosIp);
    });
        connect(ui->btnWinClose_3, &QPushButton::clicked, [=]() {
        this->close();
    });
        connect(ui->btnSignOut, &QPushButton::clicked, [=]() {
        ui->stackedWidget->setCurrentIndex(1);
    });

    if (ui->checkBoxAutoLogin->isChecked()) {
        ui->btnLogin->setText("CANCEL");
        ui->btnLogin->setStyleSheet(
                "border:0px;background-color:rgb(211, 215, 207);color:WHITE；");
        QTimer::singleShot(2000, this, SLOT(slot_autoLoad()));
    }
}

QLoginWidget::~QLoginWidget() {
    delete ui;
}

void QLoginWidget::slot_autoLoad() {
    if (m_bIsConnect) {
        ConnectMaster();
    }
}

void QLoginWidget::changeEvent(QEvent *e) {
    QWidget::changeEvent(e);
    switch (e->type()) {
        case QEvent::LanguageChange:
            ui->retranslateUi(this);
            break;
        default:
            break;
    }
}


void QLoginWidget::initSettings() {
    QSettings settings("junbot_gui", "Displays");
    settings.value("Grid/enable", bool(true)).toBool();
    settings.value("Grid/count", double(20)).toDouble();
    settings.value("GlobalOption/FixedFrame", "map").toString();
    settings.value("Map/enable", bool(true)).toBool();
    settings.value("Map/topic", QString("/map")).toString();
    settings.value("Map/scheme", QString("map")).toString();

    settings.value("Laser/enable", bool(true)).toBool();
    settings.value("Laser/topic", QString("/scan")).toString();
    settings.value("Polygon/enable", bool(true)).toBool();
    settings.value("Polygon/topic", QString("/move_base/local_costmap/footprint")).toString();

    settings.value("RobotModel/enable", bool(true)).toBool();
    settings.value("GlobalMap/enable", bool(true)).toBool();
    settings.value("GlobalMap/topic",QString("/move_base/global_costmap/costmap")).toString();
    settings.value("GlobalPlan/topic", QString("/move_base/NavfnROS/plan")).toString();
    settings.value("GlobalPlan/color", QString("255;0;0")).toString();
    settings.value("LocalMap/enable", bool(true)).toBool();
    settings.value("LocalMap/topic", QString("/move_base/local_costmap/costmap")).toString();
    settings.value("LocalPlan/topic",QString("/move_base/DWAPlannerROS/local_plan")).toString();
    settings.value("LocalPlan/color", QString("0;0;255")).toString();

    QSettings main_setting("junbot_gui", "settings");
    main_setting.value("video/names").toStringList();
    main_setting.value("video/topics").toStringList();
}


void QLoginWidget::on_btnLogin_clicked() {
    if (ui->btnLogin->text() != "CANCEL") {
        ConnectMaster();
    } else {
        ui->btnLogin->setText("CONNECT");
        ui->btnLogin->setStyleSheet(
                "border:0px;background-color:#F81243;color:WHITE；");
        m_bIsConnect = false;
    }
}

// Connect to ROS master => open MainWindow
void QLoginWidget::ConnectMaster() {
    int argc;
    char **argv;
    if (robotInterface != NULL) {
        robotInterface->close();
    }

    // Init RobotInterface
    robotInterface = new RobotInterface(m_model);

    // TODO: update for using input text
    if(robotInterface->connectMaster("http://localhost:11311", "127.0.0.1"))
    {
        this->hide();
        CONSOLE << "Something";
        robotInterface->show();
    }
    else 
    {
        QMessageBox::warning(NULL, "Error",
                                 "Can't connect to ROS core",
                                 QMessageBox::Yes);
        CONSOLE << "Connect Fail";
    }
    

    connect(robotInterface, &RobotInterface::signalDisconnect, [=]() {
        this->show();
    });
}

void QLoginWidget::slot_ShowWindow() {
    this->show();
    ui->btnLogin->setEnabled(true);
    ui->btnLogin->setStyleSheet(
            "border:0px;background-color:#F81243;color:WHITE；");
}

void QLoginWidget::paintEvent(QPaintEvent *) {
    QPainter painter;
    painter.fillRect(this->rect(), Qt::transparent);
}

//  TODO: Move userCreate button to after login (with admin type, not appear with normal user)
void QLoginWidget::createUser()
{
    QString name = ui->nameInput->text();
    QString pass = ui->passInput->text();
    QString type = "customer";

    CONSOLE << m_model->getCurrentUserType();

    if(m_model->getCurrentUserType() == "admin")
    {
        type = ui->comboBox->currentText();
        CONSOLE << "Type: " << type;
    }
    else{
        type = "customer";
    }
    
    QUser u(name, pass, type);

    CONSOLE << name << " " << pass;

    bool result = m_model->addUser(u);

    if (result)
    {
        QMessageBox msgBox;
        msgBox.setText("Create user successed");
        msgBox.exec();
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("User exited");
        msgBox.exec();
    }
}

void QLoginWidget::getLogindata()
{
    QString usrname = ui->username->text();
    QString usrpass = ui->lineEdit->text();
    QUser usrlogin(usrname, usrpass);

    bool checker = m_model->login(usrlogin);

    CONSOLE << m_model->getCurrentUserType();

    if(checker){
        
        if(m_model->getCurrentUserType() == "admin")
        {
            ui->createUserAsAdmin_btn->setVisible(true);
            ui->stackedWidget->setCurrentIndex(0);
        }
        else 
        {
            ui->createUserAsAdmin_btn->setVisible(false);
            ui->stackedWidget->setCurrentIndex(0);
        }
    }
    else{
        QMessageBox::warning(NULL, "Login failed",
                                 "The username or password you entered is incorrect! Please try again!",
                                 QMessageBox::Yes);
    }
}