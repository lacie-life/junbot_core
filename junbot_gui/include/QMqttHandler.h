#ifndef QMQTT_HANDLER_H
#define QMQTT_HANDLER_H

#include <QObject>
#include <QString>
#include <QtMqtt/QtMqtt>
#include <QtMqtt/QMqttClient>
#include <QVector>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <QFile>
#include <QMutex>

struct RobotNode {
    QString name;
    QString ip;
    QString current_state_topic;
    QJsonObject current_state_message; // TODO: Which state need to publish for mobile app ? 
    QString control_topic;
    QJsonObject control_message; // TODO: How to create control message ? Which feature need to control ?
};

class QMqttHandler : public QObject
{
    Q_OBJECT

    static QMqttHandler *m_instance;

public:
    static QMqttHandler *getInstance();
    ~QMqttHandler();

    bool loadMQTTSetting(QString path); // Load MQTT topics
    int initBokerHost(QString path);
    void connectMQTT(QString brokerName, qint16 port);

    QString mqttMessage() const;
    RobotNode currentRobotNode() const;

public slots:
    void onMQTT_Connected();
    void onMQTT_disconnected();
    void onMQTT_Received(const QByteArray &message, const QMqttTopicName &topic);

    void MQTT_Publish(RobotNode node, QJsonObject message);
    void MQTT_Subcrib(RobotNode node);

    void setMqttMessage(QJsonObject &msg);
    void setCurrentRobotNode(RobotNode node);

signals:
    void MQTT_Received(QString msg);
    void mqttMessageChanged(QString msg);

public:
    QVector<RobotNode> RobotNodes;

    QString device_path;
    QString broker_path;

private:
    QMqttHandler(QObject* parent = nullptr);
    QMqttHandler(const QMqttHandler&) = delete;
    void operator =(const QMqttHandler&) = delete;

    QMqttClient *m_client;
    RobotNode m_current_robot_node;
    QString m_mqttMessage;

};

#endif // QMQTT_HANDLER_H