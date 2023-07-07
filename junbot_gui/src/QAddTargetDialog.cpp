#include "QAddTargetDialog.h"
#include "AppConstants.h"
#include "AppModel.h"
#include "ui_addNewTarget.h"

#include <QCompleter>

QAddNewTarget::QAddNewTarget(AppModel *model, QWidget *parent)
        : QWidget(parent)
        , ui(new Ui::AddNewTarget) {
    ui->setupUi(this);

    m_model = model;
}

QAddNewTarget::~QAddNewTarget() {
    delete ui;
}