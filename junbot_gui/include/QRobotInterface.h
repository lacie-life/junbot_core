#ifndef ROBOTINTERFACE_H
#define ROBOTINTERFACE_H

#include <QMainWindow>
#include <sensor_msgs/BatteryState.h>

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QTimer>
#include <QTreeWidgetItem>
#include <QPushButton>
#include <QMessageBox>
#include <QSlider>
#include <QGridLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QLabel>
#include <QTextEdit>

#include <QImage>
#include <QPixmap>

#include "QRobotUltis.h"
#include "QRobotItem.h"
#include "QAddTargetDialog.h"
#include "AppModel.h"
#include "QMqttHandler.h"

namespace Ui {
class RobotInterface;
}

class RobotInterface : public QMainWindow
{
  Q_OBJECT

public:
  RobotInterface(AppModel *model = nullptr, QWidget *parent = nullptr);
  ~RobotInterface();

  void readSettings();   // Load up qt program settings at startup
  void writeSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent *event);  // Overloaded function
  void showNoMasterMessage(); // what's this mean ?

  void initUis();
  bool connectMaster(QString master_ip, QString ros_ip);

public slots:
  /******************************************
     ** Auto-connections (connectSlotsByName())
     *******************************************/
  void slot_batteryState(sensor_msgs::BatteryState);
  void slot_batteryPercentage(float msg);
  void slot_batteryVoltage(float msg);
  void slot_rosShutdown();
  void slot_cmd_control();
  
  void slot_dis_connect();
  void slot_updateRobotStatus(AppEnums::QRobotStatus);
  void slot_updateRobotMissonStatus(AppEnums::QMissionStatus);
  void slot_obstacle(AppEnums::QObstacle);

  void slot_settingTarget();

  void slotRemoveTarget();

  void updateTargetSlot(QDeliveryTarget target);

  void runNextTarget();
  void slotRun();

signals:
  void signalDisconnect();
  void signalKeyPressed(int key);
  void acceptedTarget();
  void updateControllingStatus(int i);
  void updateMissionStatus(int i);
  void checkBattery(float bt);

private:
  void connections();
  void removeTarget(int i);

  void updateTagerSlotUI();

protected:
  // void keyPressEvent(QKeyEvent *event) override;

private:
  Ui::RobotInterface *ui;
  bool isPressedWidget;

  AppModel* m_model;
  QAddNewTarget* m_addNewTarget;

  QMqttHandler* m_handler; 

  QVector<QPushButton*> m_targetButton;

  QVector<bool> slot_state;
  QVector<QDeliveryTarget> slot_target;

  bool m_useEnviorment = false;
  bool m_autoConnect = false;
  QString m_masterUrl;
  QString m_hostUrl;
  double m_turnLightThre = 0.1;
  QGraphicsScene *m_qgraphicsScene = nullptr;
  QRobotItem *m_roboItem = nullptr;
//  QVariantList m_sendVelList, m_recvVelList, m_timeList;

  int line_max = 10;

  int m_targetSelected = NULL;

  QTimer *m_timerChart;
  QTimer *m_timerPubImageMap;
  QTimer *m_timerCurrentTime;
};

#endif // ROBOTINTERFACE_H
