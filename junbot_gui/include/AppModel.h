#ifndef APPMODEL_H
#define APPMODEL_H

#include <QObject>
#include <QString>

#include <memory.h>

#include "AppConstants.h"
#include "QDatabaseManager.h"
#include "QUser.h"
#include "QNode.h"
#include "QRobotItem.h"
#include "QRobotUltis.h"

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

    // ROS Connection
    bool connectMaster(QString master_ip, QString ros_ip);

public slots:

signals:
    void signalSet2DPose();
    void signalSet2DGoal();
    void signalDisconnect();
    void signalSignOut();
    void signalLogin();

public:
    // Ros interface
    QNode m_rosNode;

private:
    QString m_masterUrl;
    QString m_hostUrl;

    QUser* m_currentUser;
    QDatabaseManager& m_dbManager;

};

#endif // APPMODEL_H
