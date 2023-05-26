#ifndef QDATABASEMANAGER_H
#define QDATABASEMANAGER_H

#include <QObject>
#include <QMutex>
#include <memory>

#include "QUserDAO.h"

class QSqlDatabase;
class QSqlQuery;


const QString DATABASE_FILENAME = "data/user.db";

class QDatabaseManager
{

public:
    static void debugQuery(const QSqlQuery& query);

    QDatabaseManager(const QString& path = DATABASE_FILENAME);

    static QDatabaseManager& instance();
    ~QDatabaseManager();

protected:
    QDatabaseManager& operator= (const QDatabaseManager& rhs);

public:
    const QUserDAO userDao;

private:
    static QSqlDatabase* m_database;
};

#endif // QDATABASEMANAGER_H
