#include "QAppConfig.h"

QAppConfig::QAppConfig(QObject* parent, QString path)
  : QObject{}
{

}

QAppConfig::~QAppConfig()
{

}

bool QAppConfig::getBool(const std::string &key)
{

}


template<typename T>
T QAppConfig::get(const std::string &key)
{

}

template<typename T>
std::vector<T> QAppConfig::getVector(const std::string &key)
{

}
