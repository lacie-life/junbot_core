#include "QUserDAO.h"
#include "QDatabaseManager.h"
#include "QUser.h"

#include <QSqlDatabase>
#include <QSqlQuery>
#include <QVariant>
#include <QDebug>
#include <QSqlError>

QUserDAO::QUserDAO(QSqlDatabase& database)
    : m_database(database)
{
}

void QUserDAO::init() const
{
    if(!m_database.tables().contains("users"))
    {
        QSqlQuery query(m_database);
        query.exec("CREATE TABLE users (id INTEGER PRIMARY KEY AUTOINCREMENT, name text NOT NULL, pass text NOT NULL, type text NOT NULL)");

        if (query.lastError().type() == QSqlError::ErrorType::NoError) {
            qDebug() << "Query OK:"  << query.lastQuery();
        } else {
            qWarning() << "Query KO:" << query.lastError().text();
            qWarning() << "Query text:" << query.lastQuery();
        }
    }
    CONSOLE << "User init sucess";
}

bool QUserDAO::addUser(QUser& user) const
{
    // TODO: Check exited user

    if (!isUserExits(user))
    {
        QSqlQuery query(m_database);
        query.prepare("INSERT INTO users (name, pass, type) VALUES (:name, :pass, :type)");

        qDebug() << user.name();
        qDebug() << user.pass();
        // qDebug() << user.type();

        query.bindValue(":name", user.name(), QSql::In);
        query.bindValue(":pass", user.pass(), QSql::In);
        query.bindValue(":type", user.type(), QSql::In);

        query.exec();
        user.setId(query.lastInsertId().toInt());

        QDatabaseManager::debugQuery(query);

        return true;
    }
    else {
        return false;
    }
}

bool QUserDAO::isUserExits(QUser &user) const
{
    QSqlQuery query(m_database);
    query.prepare("SELECT * FROM users WHERE name = :name");

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

bool QUserDAO::isloginUserExits(QUser &user) const
{
    QSqlQuery query(m_database);
    query.prepare("SELECT * FROM users WHERE name = :name");

    query.bindValue(":name", user.name());

    query.exec();

    QDatabaseManager::debugQuery(query);

    while(query.next()){
        if(user.name() == query.value("name").toString()){
            if(query.value("pass").toString() == user.pass()){
                user.setId(query.value("id").toString().toInt());
                user.setType(query.value("type").toString());
                return true;
            }
            else{
                return false;
            }
        }
    }
    return false;
}