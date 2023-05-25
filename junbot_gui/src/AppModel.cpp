#include "AppModel.h"
#include <QWidget>

AppModel::AppModel(int argc, char **argv, QObject *parent)
    : QObject(parent)
    , m_dbHandler(QDBHandler::getInstance())
    , m_rosNode(argc, argv)
{
    readSettings();
    connections();
}

AppModel::~AppModel()
{
    writeSettings();
    QDBHandler::resetInstance();
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