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
    if(!isTargetExits(user))
    {
        QSqlQuery query(m_database);
        query.prepare("INSERT INTO delivery_target (name, x_axis, y_axis, z_axis) VALUES (:name, :x_axis, :y_axis, :z_axis)");

        query.bindValue(":name", user.name(), QSql::In);
        query.bindValue(":x_axis", user.x_axis(), QSql::In);
        query.bindValue(":y_axis", user.y_axis(), QSql::In);
        query.bindValue(":z_axis", user.z_axis(), QSql::In);

        query.exec();
        user.setId(query.lastInsertId().toInt());

        QDatabaseManager::debugQuery(query);

        return true;
    }
    else{
        return false;
    }
}

bool QDeliveryTargetDAO::addJsonString(QString str) const{
    QSqlQuery query(m_database);
    query.prepare("INSERT INTO json_state (jState) VALUES (:jState)");

    query.bindValue(":jState", str, QSql::In);

    query.exec();

    QDatabaseManager::debugQuery(query);

    return true;
}

bool QDeliveryTargetDAO::isTargetExits(QDeliveryTarget& user) const
{
    QSqlQuery query(m_database);
    query.prepare("SELECT * FROM delivery_target WHERE name = :name");

    query.bindValue(":name", user.name());

    query.exec();

    QDatabaseManager::debugQuery(query);

    while(query.next()){
        if(user.name() == query.value("name").toString()){
            return true;
        }else{
            return false;
        }
    }
    return false;
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
