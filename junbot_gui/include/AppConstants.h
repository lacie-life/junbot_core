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
  Q_ENUMS(QRobotAction)
  Q_ENUMS(QDisplayMode)

public:
  enum QRobotStatus
  {
    None,
    Normal,
    Warning,
    Error,
  };

  enum QRobotColor
  {
    Blue,
    Red,
    Yellow,
  };

  enum QLogLevel
  {
    Debug,
    Info,
    Warn,
    Err,
    Fatal,
  };

  enum QRobotAction
  {
    UpLeft = 0,
    Up,
    UpRight,
    Left,
    Stop,
    Right,
    DownLeft,
    Down,
    DownRight,
  };

  enum QDisplayMode
  {
    Robot,
    Control,
  };
};

#endif // APPCONSTANTS_H
