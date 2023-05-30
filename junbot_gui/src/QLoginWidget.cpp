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

QLoginWidget::QLoginWidget(int argc, char **argv, QWidget *parent)
        : QCustomMoveWidget(parent)
        , ui(new Ui::LoginWidget) {
    ui->setupUi(this);

    this->setAttribute(Qt::WA_TranslucentBackground);
    this->setWindowFlags(Qt::FramelessWindowHint);
    this->setWindowFlags(Qt::FramelessWindowHint);

    m_model = new AppModel(argc, argv);

    initSettings();

    QImage img(":/background/data/background/test.jpg");

    ui->label_video->setPixmap(QPixmap::fromImage(img).scaled(ui->label_video->size(), Qt::KeepAspectRatioByExpanding,
                                                              Qt::SmoothTransformation));

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
    // connect(this, &QLoginWidget::createButtonClicked, this, [this] {
    //     ui->stackedWidget->setCurrentIndex(2);
    // });
    connect(ui->createButton_2, &QPushButton::clicked, this, &QLoginWidget::createUser);

    connect(ui->closeButton, &QPushButton::clicked, this, [this] {
        ui->stackedWidget->setCurrentIndex(1);
    });

    // *****Login (widget_1)*****
    // connect(ui->createButton_2, &QPushButton::clicked, this, [this] {
    //     emit createButtonClicked();
    // });
    connect(ui->createButton, &QPushButton::clicked, this, [this]{
        ui->stackedWidget->setCurrentIndex(2);
    });
    connect(ui->loginButton, &QPushButton::clicked, [=]() {
        QLoginWidget::getLogindata();
    });
    connect(ui->cancelButton, &QPushButton::clicked, [=]() {
        this->close();
    });

    // connect(this, &QLoginWidget::loginSuccess, this, [this] {
    //     ui->stackedWidget->setCurrentIndex(0);
    // });
    // connect(this, &QLoginWidget::loginFail, this, [this] {
    //     ui->stackedWidget->setCurrentIndex(0);
    // });

    // *****Set ip for ROS master connection*****
    connect(ui->pushButton_auto, &QPushButton::clicked, [=]() {
        ui->lineEditMasterIp->setText(m_qMasterIp);
        ui->lineEditRosIp->setText(m_qRosIp);
    });

    // Hint
    connect(ui->pushButton_hellp, &QPushButton::clicked, [=]() {
        QDesktopServices::openUrl(
                QUrl(QString("https://github.com/ScarecrowStraw/JunBotGUI/blob/main/README.md")));
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
    if (mainWindow != NULL) {
        mainWindow->close();
    }

    // Init MainWindow
    mainWindow = new MainWindow(argc, argv);

    connect(mainWindow, SIGNAL(signalDisconnect()), this,
            SLOT(slot_ShowWindow()));

    QCoreApplication::processEvents();
    ui->btnLogin->setText("CANCEL");
    ui->btnLogin->setStyleSheet(
            "border:0px;background-color:rgb(211, 215, 207);color:WHITE；");

    bool isConnect = mainWindow->connectMaster(
            ui->lineEditMasterIp->text(), ui->lineEditRosIp->text());

    if (isConnect) {
        QSettings connect_info("junbot_gui", "connect_info");
        connect_info.setValue("master_url", ui->lineEditMasterIp->text());
        connect_info.setValue("host_url", ui->lineEditRosIp->text());
        connect_info.setValue("auto_connect", ui->checkBoxAutoLogin->isChecked());
        ui->btnLogin->setText("CONNECT");
        ui->btnLogin->setStyleSheet(
                "border:0px;background-color:#F81243;color:WHITE；");
        this->hide();
        mainWindow->show();
    } else {
        ui->checkBoxAutoLogin->setChecked(false);
        ui->btnLogin->setText("CONNECT");
        ui->btnLogin->setStyleSheet(
                "border:0px;background-color:#F81243;color:WHITE；");
        QMessageBox::information(NULL, "Connection failed",
                                 "Connection failed! Please check your connection configuration or restart and try again!",
                                 QMessageBox::Yes);
    }
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



void QLoginWidget::createUser()
{
    QString name = ui->nameInput->text();
    QString pass = ui->passInput->text();
    QString type = ui->fullNameInput->text();

    QUser u(name, pass, type);

    CONSOLE << name << " " << pass << " " << type;

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

    if(checker){
        ui->stackedWidget->setCurrentIndex(0);
        // emit loginSuccess();
        // emit LoginUserInit();
    }
    else{
        QMessageBox::warning(NULL, "Login failed",
                                 "The username or password you entered is incorrect! Please try again!",
                                 QMessageBox::Yes);
        // emit loginFail();
    }
}