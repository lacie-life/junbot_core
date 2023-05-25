#ifndef APPCONSTANTS_H
#define APPCONSTANTS_H

#include <QObject>

#ifndef MACRO_DEFINE
#define MACRO_DEFINE

#define CONSOLE qDebug() << "[" << __FUNCTION__ << "] "

#endif

#define CONFIG_PATH "./data/config.yaml"

class AppEnums : public QObject {
Q_OBJECT
    Q_ENUMS(QRobotStatus)
    Q_ENUMS(QRobotColor)
    Q_ENUMS(QLogLevel)
    Q_ENUMS(QMissionStatus)

public:
    enum QRobotStatus {
        None,
        Normal,
        Warning,
        Error,
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
};

#endif // APPCONSTANTS_H
