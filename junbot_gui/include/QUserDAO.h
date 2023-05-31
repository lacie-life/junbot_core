#ifndef QUSERDAO_H
#define QUSERDAO_H

#include <QObject>
#include "AppConstants.h"

class QSqlDatabase;
class QUser;

class QUserDAO
{

public:
    QUserDAO(QSqlDatabase& database);
    void init() const;

    bool addUser(QUser& user) const;
    bool isUserExits(QUser& user) const;

    //TODO: Add CRUD function
    void updateUser(const QUser& user) const;
    void removeUser(int id) const;
    QVector<QUser*> users() const;

    bool isloginUserExits(QUser& user) const;

private:
    QSqlDatabase& m_database;
};

#endif // QUSERDAO_H
