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

#include <QImage>
#include <QPixmap>

#include "QRobotUltis.h"
#include "QRobotItem.h"
#include "AppModel.h"

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
  void slot_rosShutdown();
  void slot_cmd_control();
  void slot_buttonChangeColorA();
  void slot_buttonChangeColorB();
  void slot_buttonChangeColorC();
  
  void slot_dis_connect();
  void slot_updateRobotStatus(AppEnums::QRobotStatus);
  void slot_updateRobotMissonStatus(AppEnums::QMissionStatus);

signals:
  void signalDisconnect();
  void signalKeyPressed(int key);

private:
  void connections();
  void display_rviz();

protected:
  void keyPressEvent(QKeyEvent *event) override;

private:
  Ui::RobotInterface *ui;
  bool isPressedWidget;

  AppModel* m_model;

//  QStandardItemModel *treeView_rviz_model = nullptr;

//  // Store the address of the control currently displayed
//  // by the rviz treewidget and the parent of the control
//  QMap<QWidget *, QTreeWidgetItem *> widget_to_parentItem_map;

//  // Store the corresponding relationship of the status bar display name status item
//  QMap<QString, QTreeWidgetItem *> tree_rviz_stues;

//  // Store the current value of display item name, parameter name and value
//  QMap<QTreeWidgetItem *, QMap<QString, QString>> tree_rviz_values;

  bool m_useEnviorment = false;
  bool m_autoConnect = false;
  QString m_masterUrl;
  QString m_hostUrl;
  double m_turnLightThre = 0.1;
  QGraphicsScene *m_qgraphicsScene = nullptr;
  QRobotItem *m_roboItem = nullptr;
//  QVariantList m_sendVelList, m_recvVelList, m_timeList;

  int line_max = 10;

  QTimer *m_timerChart;
  QTimer *m_timerPubImageMap;
  QTimer *m_timerCurrentTime;
};

#endif // ROBOTINTERFACE_H
