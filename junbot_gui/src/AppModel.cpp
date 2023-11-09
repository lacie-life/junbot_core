#include "AppModel.h"
#include <QWidget>

std::mutex AppModel::m_mutex;

AppModel::AppModel(int argc, char **argv, QObject *parent)
    : QObject(parent)
    , m_rosNode(argc, argv)
    , m_dbManager(QDatabaseManager::instance())
    // , m_currentUser(nullptr)
{
    CONSOLE << "App Init";

    m_currentUser = new QUser("defaut", "default", "default");
    m_targets = m_dbManager.deliveryTargetDao.targets();

    // readSettings();

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

    connect(m_handler, &QMqttHandler::mqttSubControl, this, &AppModel::slotMqttSubControl);
    connect(m_handler, &QMqttHandler::mqttSubTarget, this, &AppModel::slotMqttSubTarget);
    connect(m_handler, &QMqttHandler::mqttSubLogin, this, &AppModel::slotMqttSubLogin);
    connect(m_handler, &QMqttHandler::MQTTConnected, this, &AppModel::initMQTTSub);

    // m_handler->MQTT_Publish(m_handler->RobotNodes.at(0), jobj);

    connect(&m_rosNode, &QNode::updateBatteryPercentage,  this, &AppModel::batteryStatus);
}

AppModel::~AppModel()
{
    writeSettings();
}

void AppModel::slotMqttSubControl(QString msg)
{
  float liner = 0.5;
  float turn = 0.5;
  bool is_all = false;

if(msg == "forward"){
  CONSOLE << "hihi";
    // forward
    m_rosNode.move_base(is_all ? 'I' : 'i', liner, turn);
    CONSOLE << "my forward: " << liner;
  } else if(msg == "back"){
    // backward
    m_rosNode.move_base(is_all ? '<' : ',', liner, turn);
    CONSOLE << "my backward: " << liner;
  } else if (msg == "turn left") {
    // rotate left
    m_rosNode.move_base(is_all ? 'J' : 'j', liner, turn);
    CONSOLE << "left_value: ";
  } else if (msg == "turn right") {
    // rotate right
    m_rosNode.move_base(is_all ? 'L' : 'l', liner, turn);
    CONSOLE << "right_value: ";
  } else {
    // stop
    m_rosNode.move_base(is_all ? 'K' : 'k', liner, turn);
  }
}

void AppModel::slotMqttSubTarget(const QList<QString>& names, const QList<int>& x, 
                            const QList<int>& y, const QList<int>& z)
{

  std::vector<QRobotPose> goals;
  std::vector<int> goals_Id;
  for(int i = 0; i < names.size(); i++){

    QRobotPose goal = {(double)x[i],
      (double)y[i],
      (double)z[i],
      1
      };
    goals.push_back(goal);

    // TODO: add id to goals
    goals_Id.push_back(i + 1);
  }

  CONSOLE << goals.size();

  bool check;

  check = false;
  check = m_rosNode.set_multi_goal("Frame", goals, goals_Id);

  connect(&m_rosNode, &QNode::updateGoalReached, this, [=](){
    QMessageBox::information(NULL, "Notification",
                                 "Robot has arrived the target!",
                                 QMessageBox::Ok);

    emit acceptedTarget();  
  });

  connect(this, &AppModel::acceptedTarget, this, [=](){
    m_rosNode.sendNextTarget();
  });
}

bool AppModel::slotMqttSubLogin(QString username, QString password)
{
    m_mobileUser = new QUser(username, password, "customer");

    bool checkdone = m_dbManager.userDao.isloginUserExits(*m_mobileUser);

    if(checkdone){
        CONSOLE << "Login Success";
        QJsonObject jobj;
        jobj["response"] = "success";
        m_handler->MQTT_Publish("robot1/login_userInforesponse", jobj);

        return true;
    }
    else {
        CONSOLE << "Login Fail";
        QJsonObject jobj;
        jobj["response"] = "fail";
        m_handler->MQTT_Publish("robot1/login_userInforesponse", jobj);
        CONSOLE << "haha";
        return false;           
    }
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

        m_handler->MQTT_Subcrib(m_handler->RobotNodes.at(0));
    }

    m_timer.setInterval(1000);
    connect(&m_timer, &QTimer::timeout, this, &AppModel::checkRobotState);
    m_timer.start();

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

void AppModel::initMQTTSub()
{
    m_handler->MQTT_Subcrib("robot1/login_request");
    m_handler->MQTT_Subcrib("robot1/deliver");
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
        // checkRobotState();
        break;
    case 1:
        sensor_state = AppEnums::QRobotSensor::SensorOk;
        // checkRobotState();
        break;
    }
}

void AppModel::controllingStatus(int key)
{
    switch(key)
    {
    case 0:
        is_controlling_state = AppEnums::QRobotControlling::NoControlling;
        // checkRobotState();
        break;
    case 1:
        is_controlling_state = AppEnums::QRobotControlling::HaveControlling;
        // checkRobotState();
        break;
    }
}

void AppModel::havingMissionStatus(int key)
{
    switch(key)
    {
    case 0:
        is_mission_state = AppEnums::QRobotMisson::NoMission;
        // checkRobotState();
        break;
    case 1:
        is_mission_state = AppEnums::QRobotMisson::HaveMission;
        // checkRobotState();
        break;
    }
}

void AppModel::batteryStatus(float battery)
{
    if(battery > 50.0){
        battery_state = AppEnums::QRobotBattery::Normal;
        // checkRobotState();
    }else{
        battery_state = AppEnums::QRobotBattery::NeedCharge;
        // checkRobotState();
        // emit signalNeedCharge();
    }
    m_currentBattery = battery;

    emit signalBatteryPercentage(m_currentBattery);
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
    // CONSOLE << "Checking 1 !!!!!";
    // m_mutex.lock();
    if(AppEnums::QRobotBattery::Normal && AppEnums::QRobotMisson::NoMission 
        && AppEnums::QRobotControlling::NoControlling && AppEnums::QRobotSensor::SensorOk)
    {
        m_robot_status = AppEnums::QRobotStatus::Ready;
        // CONSOLE << is_mission_state;
        emit signalRobotStateUpdate(m_robot_status);
    }else{
        m_robot_status = AppEnums::QRobotStatus::NotReady;
        // CONSOLE << is_mission_state;
        emit signalRobotStateUpdate(m_robot_status);
    } 

    // CONSOLE << "Checking 2 !!!!!";

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

    // CONSOLE << jSub;

    // CONSOLE << "Checking 3 !!!!!";

    m_handler->MQTT_Publish(m_handler->RobotNodes.at(0), jobj);

    m_rosNode.publishRobotStatus(jString);


    // CONSOLE << "Checking"; 
    m_mutex.unlock();

    // CONSOLE << "Checking 4"; 
    // m_mutex.unlock();

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

