#include "QDBHandler.h"

QDBHandler *QDBHandler::instance = nullptr;

QDBHandler::QDBHandler() {
    init();
}

QDBHandler::~QDBHandler() {
    m_db.close();
}

QDBHandler *QDBHandler::getInstance() {
    if (instance == nullptr) {
        instance = new QDBHandler();
    }
    return instance;
}

QSqlDatabase QDBHandler::getDBInstance() {
    return m_db;
}

void QDBHandler::init() {
    m_db = QSqlDatabase::addDatabase("QSQLITE", "Data");
    m_db.setDatabaseName("./data/robot.db");
    if (!m_db.open()) {
        qDebug() << "Error: Failed to connect database." << m_db.lastError();
    } else {
        qDebug() << "Succeed to connect database.";
    }
}

void QDBHandler::resetInstance() {
    if (instance != nullptr) {
        delete instance;
        instance = nullptr;
    }
}
