#include "QDeliveryTarget.h"

QDeliveryTarget::QDeliveryTarget()
{

}

QDeliveryTarget::QDeliveryTarget(QString name)
    : m_name(name)
{

}

QDeliveryTarget::QDeliveryTarget(QString name, QString x_axis, QString y_axis, QString z_axis)
    : m_name(name),
      m_xAxis(x_axis),
      m_yAxis(y_axis),
      m_zAxis(z_axis)  
{

}

QDeliveryTarget::QDeliveryTarget(QString x_axis, QString y_axis, QString z_axis)
    : m_xAxis(x_axis),
      m_yAxis(y_axis),
      m_zAxis(z_axis)
{

}

QDeliveryTarget::~QDeliveryTarget()
{
    
}

QString QDeliveryTarget::name() const
{
    return m_name;
}

QString QDeliveryTarget::x_axis() const
{
    return m_xAxis;
}

QString QDeliveryTarget::y_axis() const
{
    return m_yAxis;
}

QString QDeliveryTarget::z_axis() const
{
    return m_zAxis;
}

int QDeliveryTarget::id() const
{
    return m_id;
}

void QDeliveryTarget::setId(const int &id)
{
    m_id = id;   
}

void QDeliveryTarget::setName(const QString &name)
{
    m_name = name;
}
    
void QDeliveryTarget::setX(const QString &x_axis)
{
    m_xAxis = x_axis;
}

void QDeliveryTarget::setY(const QString &y_axis)
{
    m_yAxis = y_axis;
}

void QDeliveryTarget::setZ(const QString &z_axis)
{
    m_zAxis = z_axis;
}
    