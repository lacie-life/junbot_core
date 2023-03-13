#ifndef QLOGINWIDGET_H
#define QLOGINWIDGET_H

#include <QObject>
#include <QComboBox>
#include <QCheckBox>
#include <QDesktopServices>
#include <QSpinBox>

#include "QCustomWidget.h"
#include "mainwindow.h"

namespace Ui {
class LoginWidget;
}

class QLoginWidget : public QCustomMoveWidget
{
  Q_OBJECT
public:
  QLoginWidget(QWidget* parent = nullptr);
  ~QLoginWidget();

signals:
  void signalRotate();

protected:
  void changeEvent(QEvent* e);

private slots:

  void on_btnLogin_clicked();

  //    void on_btnRegedit_clicked();

  void SltAnimationFinished();

  void SltEditFinished();

  void on_checkBoxAutoLogin_clicked(bool checked);
  void slot_autoLoad();
  void slot_ShowWindow();
  void slot_writeSettings();

private:
  Ui::LoginWidget* ui;
  bool m_bConnected;
  MainWindow* mainWindow = NULL;
  QString m_qRosIp;
  QString m_qMasterIp;
  QComboBox* fixed_box;
  QSpinBox* Cell_Count_Box;
  QComboBox* Grid_Color_Box;
  QComboBox* Laser_Topic_box;
  QComboBox* Polygon_Topic_box;
  QComboBox* Map_Topic_box;
  QComboBox* Map_Color_Scheme_box;
  QComboBox* Path_Topic_box;
  QComboBox* Path_Color_box;
  // Navigate
  QComboBox* Global_CostMap_Topic_box;
  QComboBox* GlobalMapColorScheme_box;
  QComboBox* Local_CostMap_Topic_box;
  QComboBox* LocalMapColorScheme_box;
  QComboBox* Global_Planner_Topic_box;
  QComboBox* Global_Planner_Color_box;
  QComboBox* Local_Planner_Topic_box;
  QComboBox* Local_Planner_Color_box;
  QCheckBox* LocalMap_Check;
  QCheckBox* GlobalMap_Check;
  QCheckBox* Path_Check;
  QCheckBox* Grid_Check;
  QCheckBox* TF_Check;
  QCheckBox* Laser_Check;
  QCheckBox* Polygon_Check;
  QCheckBox* RobotModel_Check;
  QCheckBox* Map_Check;
  bool m_bIsConnect = true;

private:
  void InitWidget();
  void readSettings();
  void ConnectMaster();

protected:
  void paintEvent(QPaintEvent*);
};

#endif // QLOGINWIDGET_H
