#ifndef APPMODEL_H
#define APPMODEL_H

#include "AppConstants.h"
#include "QNode.h"
#include "QRobotItem.h"
#include "QRobotUltis.h"

#include "QDBHandler.h"

class AppModel : public QObject {
Q_OBJECT
public:
    explicit AppModel(QObject *parent = nullptr);

    ~AppModel();

    void readSettings();
    void writeSettings();

    void initUis();
    void initVideos();
    void initTopicList();
    void initOthers();

    bool connectMaster(QString master_ip, QString ros_ip, bool use_envirment = false);

public slots:
    void updateBatteryState(const sensor_msgs::BatteryState::ConstPtr &msg);
    void rosShutdown();
    void refreshTopicList();
    void cmdControl();
    void set2DGoal();
    void set2DPos();
    void disConnectMaster();

signals:
    void signalSet2DPose();
    void signalSet2DGoal();
    void signalDisconnect();

private:
    void connections();

private:
    QString m_masterUrl;
    QString m_hostUrl;

    QNode *m_rosNode = nullptr;
    QRobotItem *m_robotItem = nullptr;
    QDBHandler *m_dbHandler = nullptr;
};

#endif // APPMODEL_H