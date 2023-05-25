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
#include <QMouseEvent>

#include "QRobotUltis.h"
#include "QRobotItem.h"
#include "AppModel.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char **argv, QWidget *parent = nullptr);

    ~MainWindow();

    void readSettings();   // Load up qt program settings at startup
    void writeSettings();  // Save qt program settings when closing

    void closeEvent(QCloseEvent *event);  // Overloaded function
    void showNoMasterMessage();

    void initUis();

    void initTopicList();

    void initOthers();

    bool connectMaster(QString master_ip, QString ros_ip);

public slots:

    void slot_batteryState(sensor_msgs::BatteryState);

    void slot_rosShutdown();

    void refreshTopicList();

    void slot_cmd_control();

    void updateLoggingView();
    void Slider_raw_valueChanged(int v);

    void Slider_linear_valueChanged(int value);

    void slot_set_2D_Goal();

    void slot_set_2D_Pos();

    void slot_show_image(int, QImage);

    void slot_dis_connect();

    void slot_closeWindows();

    void slot_minWindows();

    void slot_maxWindows();

    void slot_pubImageMapTimeOut();

    void slot_updateCursorPos(QPointF pos);

    void slot_updateRobotStatus(AppEnums::QRobotStatus);

signals:

    void signalSet2DPose();

    void signalSet2DGoal();

    void signalDisconnect();

private:
    void mousePressEvent(QMouseEvent *event);

    void mouseMoveEvent(QMouseEvent *event);

    void mouseReleaseEvent(QMouseEvent *event);

    void connections();

    void setCurrentMenu(QPushButton *btn);

private:
    Ui::MainWindow *ui;
    bool isPressedWidget;
    QPoint m_lastPos;

    QGraphicsScene *m_qgraphicsScene = nullptr;
    QRobotItem *m_roboItem = nullptr;
    QVariantList m_sendVelList, m_recvVelList, m_timeList;

    AppModel* m_model;

    QTimer *m_timerChart;
    QTimer *m_timerPubImageMap;
    QTimer *m_timerCurrentTime;

};

#endif // MAINWINDOW_H
