#ifndef QLOGINWIDGET_H
#define QLOGINWIDGET_H

#include <QObject>
#include <QComboBox>
#include <QCheckBox>
#include <QDesktopServices>
#include <QSpinBox>

#include "QCustomWidget.h"
#include "QRobotInterface.h"
#include "AppModel.h"

namespace Ui {
    class LoginWidget;
}

class QLoginWidget : public QCustomMoveWidget {
Q_OBJECT
public:
    QLoginWidget(int argc, char **argv, QWidget *parent = nullptr);

    ~QLoginWidget();

signals:
    void createButtonClicked();
    // void loginSuccess();
    // void loginFail();
    // void LoginUserInit();
    void closeCreateAccountWidget();

public slots:
    void getLogindata();
    void createUser();

private slots:
    void on_btnLogin_clicked();
    void slot_autoLoad();
    void slot_ShowWindow();

private:
    Ui::LoginWidget *ui;
    bool m_bConnected;
    RobotInterface* robotInterface = NULL;
    QString m_qRosIp;
    QString m_qMasterIp;
    bool m_bIsConnect = true;
    AppModel *m_model;

private:
    void initSettings();
    void ConnectMaster();

protected:
    void paintEvent(QPaintEvent *);
    void changeEvent(QEvent *e);
};

#endif // QLOGINWIDGET_H
