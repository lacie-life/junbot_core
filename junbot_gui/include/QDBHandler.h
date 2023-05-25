#ifndef QDBHANDLER_H
#define QDBHANDLER_H

#include <QDebug>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QFile>
#include <QDebug>
#include <QSqlError>
#include <QSqlQueryModel>

class QDBHandler
{
public:
    static QDBHandler* getInstance();
    static void resetInstance();
    QSqlDatabase getDBInstance();
    ~QDBHandler();

private:
    QDBHandler();
    static QDBHandler *instance;
    void init();
    QSqlDatabase m_db;
};

#endif // QDBHANDLER_H