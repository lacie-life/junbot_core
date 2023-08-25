#ifndef APPCONSTANTS_H
#define APPCONSTANTS_H

#include <QObject>

#ifndef MACRO_DEFINE
#define MACRO_DEFINE

#define CONSOLE qDebug() << "[" << __FUNCTION__ << "] "

#endif

#define CONFIG_PATH "./data/config.yaml"

extern QString db_path;

class AppEnums : public QObject {
Q_OBJECT
    Q_ENUMS(QRobotStatus)
    Q_ENUMS(QRobotColor)
    Q_ENUMS(QLogLevel)
    Q_ENUMS(QMissionStatus)
    Q_ENUMS(QAppEvents)
    Q_ENUMS(QRobotBattery)
    Q_ENUMS(QRobotSensor)
    Q_ENUMS(QRobotControlling)
    Q_ENUMS(QRobotMission)

public:
    enum QRobotStatus {
        None,
        Ready,
        NotReady,
    };

    enum QLogLevel {
        Debug,
        Info,
        Warn,
        Err,
        Fatal,
    };

    enum QMissionStatus {
        Idle,
        Running,
        Paused,
        Stopped,
    };

    enum QRobotColor {
        Blue,
        Red,
        Yellow,
    };

    enum QRobotBattery {
        Nothing,
        Normal,
        NeedCharge,
    };

    enum QRobotSensor {
        NoSensor,
        SensorOk,
    };

    enum QRobotControlling {
        NoControlling,
        HaveControlling,
    };

    enum QRobotMisson {
        NoMission,
        HaveMission,
    };

    enum QAppEvents
    {
        RobotStatusChanged,
    };
    enum QObstacle
    {
        NoObstacle,
        Human,
        Stuff,
    };
    enum QNotification
    {
        BatteryLow
    };
};

#endif // APPCONSTANTS_H
