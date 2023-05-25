#include "QCustomWidget.h"
#include <QApplication>
#include <QDesktopWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QPropertyAnimation>
#include <QPushButton>
#include <QShowEvent>
#include <QStyleOption>
#include <QVBoxLayout>

QCustomWidget::QCustomWidget(QWidget *parent) : QWidget(parent) {}

QCustomWidget::~QCustomWidget() {}

void QCustomWidget::paintEvent(QPaintEvent *e) {
    QStyleOption opt;
    opt.init(this);
    QPainter p(this);
    style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);
}


QCustomMoveWidget::QCustomMoveWidget(QWidget *parent) : QCustomWidget(parent) {
#ifdef Q_OS_WIN32
    this->setWindowFlags(this->windowFlags() | Qt::FramelessWindowHint);
#else
    this->setWindowFlags(Qt::FramelessWindowHint);
#endif
}

QCustomMoveWidget::~QCustomMoveWidget() {}


void QCustomMoveWidget::mouseMoveEvent(QMouseEvent *e) {
    if (m_mousePressed && (e->buttons() && Qt::LeftButton)) {
        this->move(e->globalPos() - mousePoint);
        e->accept();

        if ("MainWindow" == this->objectName()) {
            QPoint pos = e->globalPos() - mousePoint;
        }
    }
}

void QCustomMoveWidget::mousePressEvent(QMouseEvent *e) {
    if (e->button() == Qt::LeftButton) {
        m_mousePressed = true;
        mousePoint = e->globalPos() - this->pos();
        e->accept();
    }
}

void QCustomMoveWidget::mouseReleaseEvent(QMouseEvent *e) {
    m_mousePressed = false;
}

///////////////////////////////////////////////////////
/// \brief CustomDialog::CustomDialog
/// \param parent
///
QCustomDialog::QCustomDialog(QWidget *parent) : QDialog(parent) {
#ifdef Q_OS_WIN32
    this->setWindowFlags(this->windowFlags() | Qt::FramelessWindowHint);
#else
    this->setWindowFlags(Qt::FramelessWindowHint);
#endif
}

QCustomDialog::~QCustomDialog() {}

/**
 * @brief CustomMoveWidget::mouseMoveEvent
 * @param e
 */
void QCustomDialog::mouseMoveEvent(QMouseEvent *e) {
    if (m_mousePressed && (e->buttons() && Qt::LeftButton)) {
        this->move(e->globalPos() - mousePoint);
        e->accept();
    }
}

void QCustomDialog::mousePressEvent(QMouseEvent *e) {
    if (e->button() == Qt::LeftButton) {
        m_mousePressed = true;
        mousePoint = e->globalPos() - this->pos();
        e->accept();
    }
}

void QCustomDialog::mouseReleaseEvent(QMouseEvent *) { m_mousePressed = false; }

///////////////////////////////////////////////////////////////////////////
/// \brief CBaseDialog::CustomDialog
/// \param parent
///
CBaseDialog::CBaseDialog(QWidget *parent) : QCustomDialog(parent) {
    this->setObjectName("CBaseDialog");
    this->setMinimumSize(300, 150);
    m_nNormalSize = QSize(300, 150);
    this->setAttribute(Qt::WA_DeleteOnClose);

    widgetWinTitle = new QWidget(this);
    widgetWinTitle->setFixedHeight(30);
    widgetWinTitle->setObjectName("widgetWinTitle");

    labelWinIcon = new QLabel(this);
    labelWinIcon->setAlignment(Qt::AlignCenter);
    labelWinIcon->setObjectName("labelWinIcon");
    labelWinIcon->setFixedSize(QSize(30, 30));
//  labelWinIcon->setPixmap(QPixmap(":/resource/images/robot.png"));

    labelWinTitle = new QLabel(this);
    labelWinTitle->setObjectName("labelWinTitle");
    labelWinTitle->setFixedHeight(30);
    labelWinTitle->setText(tr("Note"));

    btnWinMin = new QPushButton(this);
    btnWinMin->setObjectName("btnWinMin");
//  btnWinMin->setIcon(QIcon(":/resource/common/ic_min_white.png"));

    btnWinClose = new QPushButton(this);
    btnWinClose->setObjectName("btnWinClose");
//  btnWinClose->setIcon(QIcon(":/resource/common/ic_close_white.png"));

    QHBoxLayout *horLayoutBtn = new QHBoxLayout();
    horLayoutBtn->setContentsMargins(0, 0, 0, 0);
    horLayoutBtn->setSpacing(2);
    horLayoutBtn->addWidget(btnWinMin);
    horLayoutBtn->addWidget(btnWinClose);

    QHBoxLayout *horLayoutWinTitle = new QHBoxLayout(widgetWinTitle);
    horLayoutWinTitle->setContentsMargins(10, 0, 0, 0);
    horLayoutWinTitle->setSpacing(10);
    horLayoutWinTitle->addWidget(labelWinIcon, 0);
    horLayoutWinTitle->addWidget(labelWinTitle, 1);
    horLayoutWinTitle->addStretch();
    horLayoutWinTitle->addLayout(horLayoutBtn);

    /////////////////////////////////////////////////////////////////////////////
    widgetBody = new QWidget(this);
    widgetBody->setObjectName("widgetBody");

    QVBoxLayout *verLayoutWindow = new QVBoxLayout(this);
    verLayoutWindow->setContentsMargins(2, 2, 2, 2);
    verLayoutWindow->setSpacing(0);
    verLayoutWindow->addWidget(widgetWinTitle);
    verLayoutWindow->addWidget(widgetBody, 1);

    connect(btnWinMin, SIGNAL(clicked(bool)), this, SLOT(showMinimized()));
    connect(btnWinClose, SIGNAL(clicked(bool)), this, SLOT(close()));
}

CBaseDialog::~CBaseDialog() {}

/**
 * @brief CBaseDialog::SetWinIcon
 * @param pixmap
 */
void CBaseDialog::SetWinIcon(QPixmap pixmap) {
    if (pixmap.isNull()) return;
    if (pixmap.width() > 30 || pixmap.height() > 30) {
        pixmap = pixmap.scaled(30, 30);
    }

    labelWinIcon->setPixmap(pixmap);
}

/**
 * @brief CBaseDialog::SetWinTitle
 * @param text
 */
void CBaseDialog::SetWinTitle(const QString &text) {
    if (text.isEmpty()) return;
    labelWinTitle->setText(text);
}

///////////////////////////////////////////////////////////////////////////
/// \brief CMessageBox::CMessageBox
/// \param parent
CMessageBox::CMessageBox(QWidget *parent) : CBaseDialog(parent) {
    labelIcon = new QLabel(widgetBody);
    labelIcon->setFixedSize(QSize(64, 64));
    labelIcon->setAlignment(Qt::AlignCenter);
    labelIcon->setPixmap(QPixmap(":/resource/images/ic_info.png"));

    labelMsgContent = new QLabel(widgetBody);
    labelMsgContent->setWordWrap(true);
    labelMsgContent->setMinimumHeight(64);
    labelMsgContent->setObjectName("labelMsgContent");
    labelMsgContent->setStyleSheet(QString("QLabel {font: 12px;}"));

    QHBoxLayout *horLayoutContent = new QHBoxLayout();
    horLayoutContent->setContentsMargins(2, 2, 2, 2);
    horLayoutContent->setSpacing(10);
    horLayoutContent->addWidget(labelIcon);
    horLayoutContent->addWidget(labelMsgContent, 1);

    btnCancel = new QPushButton(widgetBody);
    //    btnCancel->setFocusPolicy(Qt::NoFocus);
    btnCancel->setText(tr("Cancle"));

    btnOk = new QPushButton(widgetBody);
    //    btnOk->setFocusPolicy(Qt::NoFocus);

    QHBoxLayout *horLayoutBtn = new QHBoxLayout();
    horLayoutBtn->setContentsMargins(2, 2, 2, 2);
    horLayoutBtn->setSpacing(10);
    horLayoutBtn->addStretch();
    horLayoutBtn->addWidget(btnCancel);
    horLayoutBtn->addWidget(btnOk);

    QVBoxLayout *verLayout = new QVBoxLayout(widgetBody);
    verLayout->setContentsMargins(6, 6, 6, 6);
    verLayout->addLayout(horLayoutContent, 1);
    verLayout->addLayout(horLayoutBtn);

    connect(btnCancel, SIGNAL(clicked(bool)), this, SLOT(reject()));
    connect(btnOk, SIGNAL(clicked(bool)), this, SLOT(accept()));

    m_nTimerCnt = 10;

    m_timer = new QTimer(this);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(SltTimerOut()));

    this->SetWinTitle(tr("Information"));
    this->setWindowTitle(tr("Information"));
}

CMessageBox::~CMessageBox() {}

void CMessageBox::ShowMessage(const QString &content, const quint8 &msgType,
                              const QString &title) {
    if (content.isEmpty()) return;

    if (!title.isEmpty()) this->SetWinTitle(title);
    labelMsgContent->setText(content);

    btnCancel->setVisible(CMessageBox::E_Question == msgType);

    if (CMessageBox::E_Information == msgType)
        labelIcon->setPixmap(QPixmap(":/resource/images/ic_info.png"));
    else if (CMessageBox::E_Warning == msgType)
        labelIcon->setPixmap(QPixmap(":/resource/images/ic_warning.png"));
    else if (CMessageBox::E_Question == msgType)
        labelIcon->setPixmap(QPixmap(":/resource/images/ic_question.png"));
    else
        labelIcon->setPixmap(QPixmap(":/resource/images/ic_info.png"));
}

/**
 * @brief CMessageBox::StartTimer
 */
void CMessageBox::StartTimer() {
    if (m_timer->isActive()) m_timer->stop();
    m_timer->start(1000);
}

/**
 * @brief CMessageBox::ShowInfomation
 * @param content
 * @param title
 * @return
 */
int CMessageBox::Infomation(QWidget *parent, const QString &content,
                            const QString &title) {

    CMessageBox *messageBox = new CMessageBox(parent);
    messageBox->ShowMessage(content, CMessageBox::E_Information, title);
    messageBox->StartTimer();
    messageBox->SltTimerOut();
    return messageBox->exec();
}

/**
 * @brief CMessageBox::ShowQuestion
 * @param parent
 * @param content
 * @param title
 * @return
 */
int CMessageBox::Question(QWidget *parent, const QString &content,
                          const QString &title) {
    CMessageBox *messageBox = new CMessageBox(parent);
    messageBox->ShowMessage(content, CMessageBox::E_Question, title);
    return messageBox->exec();
}

/**
 * @brief CMessageBox::ShowWarning
 * @param parent
 * @param content
 * @param title
 * @return
 */
int CMessageBox::Warning(QWidget *parent, const QString &content,
                         const QString &title) {
    CMessageBox *messageBox = new CMessageBox(parent);
    messageBox->ShowMessage(content, CMessageBox::E_Warning, title);
    return messageBox->exec();
}

void CMessageBox::SltTimerOut() {
    btnOk->setText(tr("(%1)").arg(m_nTimerCnt--));
    if (m_nTimerCnt < 0) {
        m_timer->stop();
        this->accept();
    }
}

///////////////////////////////////////////////////////////////////////////
/// \brief CInputBox::CInputBox
/// \param parent
CInputDialog::CInputDialog(QWidget *parent) : CBaseDialog(parent) {
    this->setAttribute(Qt::WA_DeleteOnClose, false);

    labelText = new QLabel(widgetBody);
    labelText->setText(tr(":"));

    lineEditInput = new QLineEdit(widgetBody);
    lineEditInput->setEchoMode(QLineEdit::Normal);

    btnCancel = new QPushButton(widgetBody);
    btnCancel->setFocusPolicy(Qt::NoFocus);
    btnCancel->setText(tr(""));

    btnOk = new QPushButton(widgetBody);
    btnOk->setFocusPolicy(Qt::NoFocus);
    btnOk->setText(tr(""));

    QHBoxLayout *horLayoutBtn = new QHBoxLayout();
    horLayoutBtn->setContentsMargins(10, 10, 10, 10);
    horLayoutBtn->setSpacing(10);
    horLayoutBtn->addStretch();
    horLayoutBtn->addWidget(btnCancel);
    horLayoutBtn->addWidget(btnOk);

    QVBoxLayout *verLayout = new QVBoxLayout(widgetBody);
    verLayout->setContentsMargins(10, 10, 10, 10);
    verLayout->addWidget(labelText);
    verLayout->addWidget(lineEditInput);
    verLayout->addLayout(horLayoutBtn);

    connect(btnCancel, SIGNAL(clicked(bool)), this, SLOT(reject()));
    connect(btnOk, SIGNAL(clicked(bool)), this, SLOT(accept()));
    connect(lineEditInput, SIGNAL(returnPressed()), this, SLOT(accept()));
}

CInputDialog::~CInputDialog() {}

/**
 * @brief CInputBox::GetInputText
 * @param parent
 * @param text
 * @param title
 * @param mode
 * @return
 */
QString CInputDialog::GetInputText(QWidget *parent, const QString &text,
                                   const QString &title,
                                   QLineEdit::EchoMode mode) {
    CInputDialog dlg(parent);
    dlg.SetInputText(text);
    dlg.SetWinTitle(title);
    dlg.SetEchoMode(mode);

    if (CInputDialog::Accepted == dlg.exec()) {
        return dlg.GetText();
    }

    return QString();
}

QString CInputDialog::GetText() const { return lineEditInput->text(); }

/**
 * @brief CInputBox::SetInputText
 * @param text
 */
void CInputDialog::SetInputText(const QString &text) {
    if (text.isEmpty()) return;
    lineEditInput->setText(text);
    lineEditInput->setFocus();
    lineEditInput->selectAll();
}

/**
 * @brief CInputBox::SetEchoMode
 * @param mode
 */
void CInputDialog::SetEchoMode(QLineEdit::EchoMode mode) {
    lineEditInput->setEchoMode(mode);
}
