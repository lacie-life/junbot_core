#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <sensor_msgs/BatteryState.h>

#include <QComboBox>
#include <QDesktopWidget>
#include <QHBoxLayout>
#include <QQueue>
#include <QSoundEffect>
#include <QSpinBox>
#include <QStandardItemModel>
#include <QTimer>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QVariant>
#include <map>
#include <QProcess>
#include <QPushButton>

#include "QRobotUltis.h"
#include "QRobotItem.h"
#include "QJoyStick.h"
#include "QNode.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = nullptr);
  ~MainWindow();

  void readSettings();   // Load up qt program settings at startup
  void writeSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent *event);  // Overloaded function
  void showNoMasterMessage();
  void initUis();
  void initVideos();
  void initTopicList();
  void initOthers();
  bool connectMaster(QString master_ip, QString ros_ip, bool use_envirment = false);

public slots:
  /******************************************
     ** Auto-connections (connectSlotsByName())
     *******************************************/
  void on_actionAbout_triggered();
  void slot_batteryState(sensor_msgs::BatteryState);
  void slot_rosShutdown();
  void refreshTopicList();
  void slot_cmd_control();

  /******************************************
    ** Manual connections
    *******************************************/
  void updateLoggingView();  // no idea why this can't connect automatically
  void Slider_raw_valueChanged(int v);
  void Slider_linear_valueChanged(int value);
  void slot_set_2D_Goal();
  void slot_set_2D_Pos();
  void slot_set_select();
  void slot_move_camera_btn();

  // Set interface
  void slot_setting_frame();
  void slot_set_mutil_goal_btn();

  // return flight
  void slot_return_point();

  // Robot position
  void slot_position_change(QString, double, double, double, double);

  // display image
  void slot_show_image(int, QImage);
  void slot_dis_connect();
  void slot_hide_table_widget();
  void slot_closeWindows();
  void slot_minWindows();
  void slot_maxWindows();
  void slot_pubImageMapTimeOut();
  void slot_updateCursorPos(QPointF pos);
  void slot_changeMapType(int);
  void slot_updateRobotStatus(AppEnums::QRobotStatus);
  //  void on_horizontalSlider_raw_valueChanged(int value);

signals:
  void signalSet2DPose();
  void signalSet2DGoal();
  void signalSetMoveCamera();
  void signalDisconnect();

private:
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void mouseReleaseEvent(QMouseEvent *event);
  void connections();
  void display_rviz();
  void setCurrentMenu(QPushButton *btn);

private:
  Ui::MainWindow *ui;
  bool isPressedWidget;
  QPoint m_lastPos;
  QNode m_qnode;
  QStandardItemModel *treeView_rviz_model = nullptr;

  // Store the address of the control currently displayed
  // by the rviz treewidget and the parent of the control
  QMap<QWidget *, QTreeWidgetItem *> widget_to_parentItem_map;

  // Store the corresponding relationship of the status bar display name status item
  QMap<QString, QTreeWidgetItem *> tree_rviz_stues;

  // Store the current value of display item name, parameter name and value
  QMap<QTreeWidgetItem *, QMap<QString, QString>> tree_rviz_values;

  bool m_useEnviorment = false;
  bool m_autoConnect = false;
  AppEnums::QDisplayMode m_showMode;
  QString m_masterUrl;
  QString m_hostUrl;
  double m_turnLightThre = 0.1;
  QGraphicsScene *m_qgraphicsScene = nullptr;
  QRobotItem *m_roboItem = nullptr;
  QVariantList m_sendVelList, m_recvVelList, m_timeList;

  int line_max = 10;

  QTimer *m_timerChart;
  QTimer *m_timerPubImageMap;
  QTimer *m_timerCurrentTime;

};
#endif // MAINWINDOW_H
