#ifndef QDELIVERYTARGETDAO_H
#define QDELIVERYTARGETDAO_H

#include <QObject>
#include "AppConstants.h"

class QSqlDatabase;
class QDeliveryTarget;

class QDeliveryTargetDAO
{

public:
    QDeliveryTargetDAO(QSqlDatabase& database);
    void init() const;

    bool addTarget(QDeliveryTarget& user) const;
    bool isTargetExits(QDeliveryTarget& user) const;
    bool addJsonString(QString str) const;

    //TODO: Add CRUD function
    void updateTarget(const QDeliveryTarget& user) const;
    void removeTarget(int id) const;
    QVector<QDeliveryTarget> targets() const;

    bool isRecivedTargetExits(QDeliveryTarget& user) const;

private:
    QSqlDatabase& m_database;
};

#endif // QDELIVERYTARGETDAO_H