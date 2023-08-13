#include "QMqttHandler.h"
#include "AppConstants.h"

QMqttHandler *QMqttHandler::m_instance = nullptr;

QMqttHandler::QMqttHandler(QObject* parent)
    : QObject(parent)
{
}

QMqttHandler *QMqttHandler::getInstance()
{
    if(m_instance == nullptr)
    {
        m_instance = new QMqttHandler();
    }
    return m_instance;
}

QMqttHandler::~QMqttHandler()
{

}

bool QMqttHandler::loadMQTTSetting(QString path)
{
    QFile file(path.toStdString().c_str());
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QString val = file.readAll();
    file.close();
    QJsonArray a = QJsonDocument::fromJson(val.toUtf8()).array();

    return true;
}

int QMqttHandler::initBokerHost(QString path)
{
    broker_path = path;

    return 0;
}

void QMqttHandler::connectMQTT(QString brokerName, qint16 port)
{
    m_client = new QMqttClient(this);

    m_client->setHostname(brokerName);
    m_client->setPort(port);

    m_client->connectToHost();

    connect(m_client, &QMqttClient::connected, this, &QMqttHandler::onMQTT_Connected);
    connect(m_client, &QMqttClient::disconnected, this, &QMqttHandler::onMQTT_disconnected);
    connect(m_client, &QMqttClient::messageReceived, this, &QMqttHandler::onMQTT_Received);
}

void QMqttHandler::onMQTT_Connected()
{
    CONSOLE << "Connected to MQTT Broker";

    MQTT_Subcrib(RobotNodes.at(0));
}

void QMqttHandler::onMQTT_disconnected()
{
    CONSOLE << "Disconnected to MQTT Broker";
}

void QMqttHandler::onMQTT_Received(const QByteArray &message, const QMqttTopicName &topic)
{
    CONSOLE << "Received message: " << message << " from topic: " << topic.name();

    QJsonObject msg = QJsonDocument::fromJson(message).object();

    setMqttMessage(msg);

    emit MQTT_Received(m_mqttMessage);
}

void QMqttHandler::MQTT_Publish(RobotNode node, QJsonObject message)
{
    QMqttTopicName topic(node.current_state_topic);

    // this->RobotNodes.at(0).current_state_message = message;

    m_client->publish(topic, QJsonDocument(message).toJson());
}

void QMqttHandler::MQTT_Subcrib(RobotNode node)
{
    CONSOLE << node.control_topic;
    
    QMqttTopicFilter filter(node.control_topic);

    m_client->subscribe(filter);
}

void QMqttHandler::setMqttMessage(QJsonObject &msg)
{

}

void QMqttHandler::setCurrentRobotNode(RobotNode node)
{
    
}