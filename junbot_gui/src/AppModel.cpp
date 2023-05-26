#include "AppModel.h"
#include <QWidget>

AppModel::AppModel(int argc, char **argv, QObject *parent)
    : QObject(parent)
    , m_rosNode(argc, argv)
{
    readSettings();
    connections();
}

AppModel::~AppModel()
{
    writeSettings();
}

void AppModel::readSettings() {
    QSettings settings("junbot_gui", "settings");
    m_masterUrl =
            settings.value("connect/master_url", QString("http://192.168.1.2:11311/"))
                    .toString();
    m_hostUrl =
            settings.value("connect/host_url", QString("192.168.1.3")).toString();
}

void AppModel::writeSettings() {
    // pending
}

void AppModel::initVideos() {
    // pending
    QSettings video_topic_setting("junbot_gui", "settings");
    QStringList names = video_topic_setting.value("video/names").toStringList();
    QStringList topics = video_topic_setting.value("video/topics").toStringList();
    if (topics.size() == 4) {
        if (topics[0] != "") {
            m_rosNode.Sub_Image(topics[0], 0);
        }
        if (topics[1] != "") {
            m_rosNode.Sub_Image(topics[1], 1);
        }
        if (topics[2] != "") {
            m_rosNode.Sub_Image(topics[2], 2);
        }
        if (topics[3] != "") {
            m_rosNode.Sub_Image(topics[3], 3);
        }
    }
}

bool AppModel::connectMaster(QString master_ip, QString ros_ip)
{
    return m_rosNode.init(master_ip.toStdString(), ros_ip.toStdString());
}

void AppModel::updateBatteryState(const sensor_msgs::BatteryState_<std::allocator<void>>::ConstPtr &msg) {

}

void AppModel::rosShutdown() {

}

void AppModel::cmdControl() {

}

void AppModel::set2DGoal() {

}

void AppModel::set2DPos() {

}

void AppModel::disConnectMaster() {

}

void AppModel::connections() {

    // TODO: Refactor MainWindow to use AppModel
}
