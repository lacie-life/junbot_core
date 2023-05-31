#include "AppModel.h"
#include <QWidget>

AppModel::AppModel(int argc, char **argv, QObject *parent)
    : QObject(parent)
    , m_rosNode(argc, argv)
    , m_dbManager(QDatabaseManager::instance())
    , m_currentUser(nullptr)
{
    CONSOLE << "App Init";

    readSettings();
}

AppModel::~AppModel()
{
    writeSettings();
}

void AppModel::readSettings() {
    QSettings settings("junbot_gui", "settings");
    m_masterUrl =
            settings.value("connect/master_url", QString("http://192.168.1.2:11311/"))
                    .toString();
    m_hostUrl =
            settings.value("connect/host_url", QString("192.168.1.3")).toString();
}

void AppModel::writeSettings() {
    // pending
}

void AppModel::initVideos() {
    // pending
    QSettings video_topic_setting("junbot_gui", "settings");
    QStringList names = video_topic_setting.value("video/names").toStringList();
    QStringList topics = video_topic_setting.value("video/topics").toStringList();
    if (topics.size() == 4) {
        if (topics[0] != "") {
            m_rosNode.Sub_Image(topics[0], 0);
        }
        if (topics[1] != "") {
            m_rosNode.Sub_Image(topics[1], 1);
        }
        if (topics[2] != "") {
            m_rosNode.Sub_Image(topics[2], 2);
        }
        if (topics[3] != "") {
            m_rosNode.Sub_Image(topics[3], 3);
        }
    }
}

bool AppModel::connectMaster(QString master_ip, QString ros_ip)
{
    return m_rosNode.init(master_ip.toStdString(), ros_ip.toStdString());
}

bool AppModel::login(QUser &user)
{
    bool checkdone = m_dbManager.userDao.isloginUserExits(user);

    m_currentUser = new QUser(user.name(), user.pass());

    if(checkdone){
        CONSOLE << m_currentUser->name();
        CONSOLE << "Login Success";
        return true;
    }
    else {
        CONSOLE << "Login Fail";
        return false;
    }
}

void AppModel::signOut()
{
    m_currentUser = nullptr;

    emit signalSignOut();
}

bool AppModel::addUser(QUser &user)
{
    bool result = m_dbManager.userDao.addUser(user);

    if(result){
        CONSOLE << "Add successed";
        return true;
    }
    else{
        CONSOLE << "User exited";
        return false;
    }
}

void signalRobotStatusChanged(AppEnums::QRobotStatus status){
    switch (status)
    {
    case AppEnums::QRobotStatus::None:
        // emit robotStatus_None;
        break;
    case AppEnums::QRobotStatus::Normal:
        // emit robotStatus_Normal;
        break;
    case AppEnums::QRobotStatus::Warning:
        // emit robotStatus_Warning;
        break;
    case AppEnums::QRobotStatus::Error:
        // emit robotStatus_Error;
        break;
    case AppEnums::QRobotStatus::NotReady:
        // emit 
        break;
    default:
        break;
    }
}

void AppModel::keyRecieved(int key)
{
    CONSOLE << key;
    switch (key)
    {
    case 1:
        emit signalRobotStatusChanged(AppEnums::QRobotStatus::None);
        break;
    case 2:
        emit signalRobotStatusChanged(AppEnums::QRobotStatus::Normal);
        break;
    case 3:
        emit signalRobotStatusChanged(AppEnums::QRobotStatus::Warning);
        break;
    case 4:
        emit signalRobotStatusChanged(AppEnums::QRobotStatus::Error);
        break;
    case 5:
        emit signalRobotStatusChanged(AppEnums::QRobotStatus::NotReady);
        break;
    default:
        break;
    }
}

void AppModel::keyMissonRecieved(int key)
{
    switch (key)
    {
    case 6:
        emit signalRobotMissionStatusChanged(AppEnums::QMissionStatus::Idle);
        break;
    case 7:
        emit signalRobotMissionStatusChanged(AppEnums::QMissionStatus::Running);
        break;
    case 8:
        emit signalRobotMissionStatusChanged(AppEnums::QMissionStatus::Paused);
        break;
    case 9:
        emit signalRobotMissionStatusChanged(AppEnums::QMissionStatus::Stopped);
        break;
    default:
        break;
    }
}