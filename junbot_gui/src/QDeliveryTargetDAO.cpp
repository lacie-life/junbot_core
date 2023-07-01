#include "QDeliveryTargetDAO.h"
#include "QDatabaseManager.h"
#include "QDeliveryTarget.h"

#include <QSqlDatabase>
#include <QSqlQuery>
#include <QVariant>
#include <QDebug>
#include <QSqlError>

class QSqlDatabase;
class QDeliveryTarget;

QDeliveryTargetDAO::QDeliveryTargetDAO(QSqlDatabase& database)
    : m_database(database)
{

}

void QDeliveryTargetDAO::init() const
{
    if(!m_database.tables().contains("delivery_target"))
    {
        QSqlQuery query(m_database);
        query.exec("CREATE TABLE delivery_target (id INTEGER PRIMARY KEY AUTOINCREMENT, name text NOT NULL, x_axis text NOT NULL,  y_axis text NOT NULL, z_axis text NOT NULL)");

        if (query.lastError().type() == QSqlError::ErrorType::NoError) {
            qDebug() << "Query OK:"  << query.lastQuery();
        } else {
            qWarning() << "Query KO:" << query.lastError().text();
            qWarning() << "Query text:" << query.lastQuery();
        }
    }
    CONSOLE << "DeliveryTarget init sucess";
}

bool QDeliveryTargetDAO::addTarget(QDeliveryTarget& user) const
{

}

bool QDeliveryTargetDAO::isTargetExits(QDeliveryTarget& user) const
{

}

QVector<QDeliveryTarget> QDeliveryTargetDAO::targets() const
{
    QSqlQuery query(m_database);

    QVector<QDeliveryTarget> results;

    query.prepare("SELECT * FROM delivery_target");

    query.exec();

    QDatabaseManager::debugQuery(query);

    int i = 0;

    while(query.next()){
        QDeliveryTarget temp(query.value("name").toString(), 
                             query.value("x_axis").toString(),
                             query.value("y_axis").toString(),
                             query.value("z_axis").toString());

        CONSOLE << "Target " << i << query.value("name");

        results.push_back(temp);
        i++;
    }
    return results;
}

bool QDeliveryTargetDAO::isRecivedTargetExits(QDeliveryTarget& user) const
{

}
