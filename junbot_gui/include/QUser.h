#ifndef QUSER_H
#define QUSER_H

#include <QObject>
#include <QString>
#include "AppConstants.h"

//TODO: Encode password before add to database

class QUser
{
public:
    explicit QUser();
    QUser(QString name, QString pass, QString fullName, QString type);
    QUser(QString name, QString pass, QString type);
    QUser(QString name, QString pass);

    ~QUser();

    QString name() const;
    QString pass() const;
    QString fullName() const;
    int id() const;
    QString type() const;

    void setId(const int& id);
    void setName(const QString& name);
    void setPass(const QString& pass);
    void setFullName(const QString& fullName);
    void setType(const QString& type);

signals:

private:
    int m_id;
    QString m_name;
    QString m_pass;
    QString m_fullName;
    QString m_type;
};

#endif // QUSER_H
