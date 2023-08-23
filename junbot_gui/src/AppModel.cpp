#include "AppModel.h"
#include <QWidget>

AppModel::AppModel(int argc, char **argv, QObject *parent)
    : QObject(parent)
    , m_rosNode(argc, argv)
    , m_dbManager(QDatabaseManager::instance())
    // , m_currentUser(nullptr)
{
    CONSOLE << "App Init";

    m_currentUser = new QUser("defaut", "default", "default");
    m_targets = m_dbManager.deliveryTargetDao.targets();

    readSettings();

    m_handler = QMqttHandler::getInstance();

    QJsonObject jobj;
    
    jobj["battery"] = m_currentBattery;
    jobj["battery_state"] = battery_state;
    jobj["sensor_state"] = sensor_state;
    jobj["is_controlling_state"] = battery_state;
    jobj["is_mission_state"] = is_mission_state;

    QJsonDocument jSub = QJsonDocument(jobj);

    CONSOLE << jSub;

    connect(m_handler, &QMqttHandler::MQTT_Received, this, &AppModel::setRobotMess);

    // m_handler->MQTT_Publish(m_handler->RobotNodes.at(0), jobj);

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
    bool check = m_rosNode.init(master_ip.toStdString(), ros_ip.toStdString());

    if(check)
    {
        RobotNode node;
        node.ip = "xx.xx.xx.xx";
        node.name = "robot1";
        node.current_state_topic = "robot1/state";
        node.control_topic = "robot1/control";

        m_handler->RobotNodes.append(node);

        m_handler->connectMQTT("localhost", 1883);
        
        // m_handler->MQTT_Subcrib(m_handler->RobotNodes.at(0));
    }
    return check;
}

QString AppModel::getCurrentUserType()
{
    return m_currentUser->type();
}

bool AppModel::login(QUser &user)
{
    bool checkdone = m_dbManager.userDao.isloginUserExits(user);

    m_currentUser = new QUser(user.name(), user.pass(), user.type());

    if(checkdone){
        CONSOLE << m_currentUser->name();
        CONSOLE << m_currentUser->type();
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

// TODO: Chaneg key to QAppEvents type, see more in AppConstants.h
void AppModel::keyRecieved(int key)
{
    CONSOLE << key;
    switch (key)
    {
    case 1:
        emit signalRobotStatusChanged(AppEnums::QRobotStatus::None);
        break;
    case 2:
        emit signalRobotStatusChanged(AppEnums::QRobotStatus::Ready);
        break;
    case 3:
        emit signalRobotStatusChanged(AppEnums::QRobotStatus::NotReady);
        break;
    default:
        break;
    }
}

// TODO: Chaneg key to QAppEvents type, see more in AppConstants.h
void AppModel::keyMissonRecieved(int key)
{
    switch (key)
    {
    case 4:
        emit signalRobotMissionStatusChanged(AppEnums::QMissionStatus::Idle);
        break;
    case 5:
        emit signalRobotMissionStatusChanged(AppEnums::QMissionStatus::Running);
        break;
    case 6:
        emit signalRobotMissionStatusChanged(AppEnums::QMissionStatus::Paused);
        break;
    case 7:
        emit signalRobotMissionStatusChanged(AppEnums::QMissionStatus::Stopped);
        break;
    default:
        break;
    }
}

void AppModel::sensorStatus(int key)
{
    switch(key)
    {
    case 0:
        sensor_state = AppEnums::QRobotSensor::NoSensor;
        checkRobotState();
        break;
    case 1:
        sensor_state = AppEnums::QRobotSensor::SensorOk;
        checkRobotState();
        break;
    }
}

void AppModel::controllingStatus(int key)
{
    switch(key)
    {
    case 0:
        is_controlling_state = AppEnums::QRobotControlling::NoControlling;
        checkRobotState();
        break;
    case 1:
        is_controlling_state = AppEnums::QRobotControlling::HaveControlling;
        checkRobotState();
        break;
    }
}

void AppModel::havingMissionStatus(int key)
{
    switch(key)
    {
    case 0:
        is_mission_state = AppEnums::QRobotMisson::NoMission;
        checkRobotState();
        break;
    case 1:
        is_mission_state = AppEnums::QRobotMisson::HaveMission;
        checkRobotState();
        break;
    }
}

void AppModel::batteryStatus(int battery)
{
    if(battery > 50){
        battery_state = AppEnums::QRobotBattery::Normal;
        checkRobotState();
    }else{
        battery_state = AppEnums::QRobotBattery::NeedCharge;
        checkRobotState();
        emit signalNeedCharge();
    }

    m_currentBattery = battery;
}

void AppModel::checkObstacle(QString id)
{
    if(id == "0")
    {
        is_obstacle = AppEnums::QObstacle::Human;
        emit obstacleUpdateUi(AppEnums::QObstacle::Human);
    }
    else 
    {
        is_obstacle = AppEnums::QObstacle::Stuff;
        emit obstacleUpdateUi(AppEnums::QObstacle::Stuff);
    }
}

void AppModel::checkRobotState()
{   
    if(AppEnums::QRobotBattery::Normal && AppEnums::QRobotMisson::NoMission 
        && AppEnums::QRobotControlling::NoControlling && AppEnums::QRobotSensor::SensorOk)
    {
        m_robot_status = AppEnums::QRobotStatus::Ready;
        CONSOLE << is_mission_state;
        emit signalRobotStateUpdate(m_robot_status);
    }else{
        m_robot_status = AppEnums::QRobotStatus::NotReady;
        CONSOLE << is_mission_state;
        emit signalRobotStateUpdate(m_robot_status);
    } 

    QJsonObject jobj;
    m_dateTime = QDateTime::currentDateTime();
    
    jobj["battery"] = m_currentBattery;
    jobj["battery_state"] = battery_state;
    jobj["sensor_state"] = sensor_state;
    jobj["is_controlling_state"] = battery_state;
    jobj["is_mission_state"] = is_mission_state;
    jobj["time"] = m_dateTime.toString("dd.MM.yyyy hh:mm:ss");

    QJsonDocument jSub = QJsonDocument(jobj);

    QString jString = QJsonDocument(jobj).toJson(QJsonDocument::Compact);

    bool result = m_dbManager.deliveryTargetDao.addJsonString(jString);

    CONSOLE << jSub;

    m_handler->MQTT_Publish(m_handler->RobotNodes.at(0), jobj);

    m_rosNode. publishRobotStatus(jString);

    CONSOLE << "Checking"; 
}

void AppModel::setRobotMess(QString msg)
{
    CONSOLE << msg;
}
 
// TODO: Create Position DB for this (Later, not now)
void AppModel::addNewPosition(QPoint point)
{

}

// TODO: Check RoBot Status for set Mission and MissionStatus
void AppModel::setMission(QRobotMission& mission)
{

}

void AppModel::startMission()
{

}

void AppModel::pauseMission()
{

}

void AppModel::stopMission()
{

}

