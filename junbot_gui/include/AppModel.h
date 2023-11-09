#ifndef APPMODEL_H
#define APPMODEL_H

#include <QObject>
#include <QString>
#include <QKeyEvent>
#include <QQueue>
#include <QDateTime>
#include <QTimer>
#include <QMessageBox>

#include <memory.h>
#include <mutex>

#include "AppConstants.h"
#include "QDatabaseManager.h"
#include "QUser.h"
#include "QDeliveryTarget.h"
#include "QNode.h"
#include "QRobotItem.h"
#include "QRobotUltis.h"
#include "QRobotMission.h"
#include "QAddTargetDialog.h"
#include "QMqttHandler.h"

#include "QDeliveryTargetDAO.h"

class AppModel : public QObject {
    Q_OBJECT
public:
    explicit AppModel(int argc, char **argv, QObject *parent = nullptr);

    ~AppModel();

    // App Settings
    void readSettings();
    void writeSettings();
    void initVideos();

    // User Manager
    bool login(QUser& user);
    void signOut();
    bool addUser(QUser& user);
    QString getCurrentUserType();

    // ROS Connection
    bool connectMaster(QString master_ip, QString ros_ip);

    // Robot Status
    void sensorStatus(int key);

    // Is Controlling Status
    void controllingStatus(int key);

    // Having Mission
    void havingMissionStatus(int key);

    //Mission Status
    void addNewPosition(QPoint point);
    void setMission(QRobotMission& mission);
    void startMission();
    void stopMission();
    void pauseMission();

public slots:
    void keyRecieved(int key);
    void keyMissonRecieved(int key);
    void setRobotMess(QString msg);
    void checkObstacle(QString id);

    // Battery Status
    void batteryStatus(float battery);
    void checkRobotState();

    void slotMqttSubControl(QString msg);
    void slotMqttSubTarget(const QList<QString>& names, const QList<int>& x, 
                            const QList<int>& y, const QList<int>& z);
    bool slotMqttSubLogin(QString username, QString password);

    void initMQTTSub();

signals:
    void signalSet2DPose();
    void signalSet2DGoal();
    void signalDisconnect();
    void signalSignOut();
    void signalLogin();
    void signalRobotStatusChanged(AppEnums::QRobotStatus status);
    void signalRobotMissionStatusChanged(AppEnums::QMissionStatus status);
    void signalObstacle(AppEnums::QObstacle status);
    void signalMissionDone();
    void signalMissionError();
    void signalMisionStarted();
    void signalRobotStateUpdate(AppEnums::QRobotStatus status);
    void signalNeedCharge();
    void obstacleUpdateUi(AppEnums::QObstacle type);
    void signalBatteryPercentage(float bt);
    void acceptedTarget();

public:
    // Ros interface
    QNode m_rosNode;
    QVector<QDeliveryTarget> m_targets;

private:
    QString m_masterUrl;
    QString m_hostUrl;

    QDateTime m_dateTime;
    QTimer m_timer;

    float m_currentBattery = 0.0;

    QUser* m_currentUser;
    QUser* m_mobileUser;
    QDatabaseManager& m_dbManager;

    QMqttHandler* m_handler; 

    AppEnums::QRobotStatus m_stattus = AppEnums::QRobotStatus::None;
    AppEnums::QMissionStatus m_misstionStatus = AppEnums::QMissionStatus::Idle;
    
    // TODO: Multi Mission control
    // Push to mission Queue if current mission is running
    QQueue<QRobotMission> m_missions;
    QRobotMission m_currentMission;

    AppEnums::QRobotBattery battery_state = AppEnums::QRobotBattery::Nothing;
    AppEnums::QRobotSensor sensor_state = AppEnums::QRobotSensor::NoSensor;
    AppEnums::QRobotControlling is_controlling_state = AppEnums::QRobotControlling::NoControlling;
    AppEnums::QRobotMisson is_mission_state = AppEnums::QRobotMisson::HaveMission;
    AppEnums::QObstacle is_obstacle = AppEnums::QObstacle::NoObstacle;

    AppEnums::QRobotStatus m_robot_status = AppEnums::QRobotStatus::None;

    static std::mutex m_mutex;
};

#endif // APPMODEL_H
