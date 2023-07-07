#ifndef QADDNEWTARGETDIALOG_H
#define QADDNEWTARGETDIALOG_H

#include <QWidget>
#include <QObject>

namespace Ui {
class AddNewTarget;
}

class AppModel;

class QAddNewTarget : public QWidget 
{
    Q_OBJECT

public:
    QAddNewTarget(AppModel *model = nullptr, QWidget *parent = nullptr);

    ~QAddNewTarget();

private:
    Ui::AddNewTarget *ui;
    AppModel *m_model;
};

#endif // QADDNEWTARGETDIALOG_H