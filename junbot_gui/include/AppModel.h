#ifndef APPMODEL_H
#define APPMODEL_H

#include "AppConstants.h"
#include "QNode.h"
#include "QRobotItem.h"
#include "QRobotUltis.h"

class AppModel : public QObject {
Q_OBJECT
public:
    explicit AppModel(int argc, char **argv, QObject *parent = nullptr);

    ~AppModel();

    void readSettings();
    void writeSettings();

    void initVideos();

    bool connectMaster(QString master_ip, QString ros_ip);

public slots:
    void updateBatteryState(const sensor_msgs::BatteryState::ConstPtr &msg);
    void rosShutdown();
    void cmdControl();
    void set2DGoal();
    void set2DPos();
    void disConnectMaster();

signals:
    void signalSet2DPose();
    void signalSet2DGoal();
    void signalDisconnect();

public:
    // Ros interface
    QNode m_rosNode;

private:
    void connections();

private:
    QString m_masterUrl;
    QString m_hostUrl;
};

#endif // APPMODEL_H
