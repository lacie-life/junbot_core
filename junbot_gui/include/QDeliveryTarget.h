#ifndef QDELIVERYTARGET_H
#define QDELIVERYTARGET_H

#include <QObject>
#include <QString>
#include "AppConstants.h"

class QDeliveryTarget
{
public:
    explicit QDeliveryTarget();
    QDeliveryTarget(QString name);
    QDeliveryTarget(QString name, QString x_axis, QString y_axis, QString z_axis);
    QDeliveryTarget(QString x_axis, QString y_axis, QString z_axis);
    
    ~QDeliveryTarget();

    QString name() const;
    QString x_axis() const;
    QString y_axis() const;
    QString z_axis() const;
    int id() const;

    void setId(const int &id);
    void setName(const QString &name);
    void setX(const QString &x_axis);
    void setY(const QString &y_axis);
    void setZ(const QString &z_axis);

signals:

private:
    int m_id;
    QString m_name;
    QString m_xAxis;
    QString m_yAxis;
    QString m_zAxis;
};

#endif // QDELIVERYTARGET_H