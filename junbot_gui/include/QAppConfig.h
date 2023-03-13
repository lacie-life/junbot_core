#ifndef QAPPCONFIG_H
#define QAPPCONFIG_H

#include "AppConstants.h"

#include <QObject>
#include <QCoreApplication>
#include <QFile>
#include <QDir>
#include <QVector>

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <assert.h>
#include <memory>

class QAppConfig : public QObject
{
  Q_OBJECT
public:
  explicit QAppConfig(QObject* parent = nullptr, QString config_path = CONFIG_PATH);
  ~QAppConfig();

public:
  template <typename T>
  static T get(const std::string &key);

  // Get a vector of content by key
  template<typename T>
  static std::vector<T> getVector(const std::string &key);

  static bool getBool(const std::string &key);

signals:

private:
  cv::FileStorage m_file;

};

#endif // QAPPCONFIG_H
