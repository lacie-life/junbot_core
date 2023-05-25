#ifndef QCUSTOMWIDGET_H
#define QCUSTOMWIDGET_H

#include <QWidget>
#include <QPaintEvent>
#include <QDialog>
#include <QLineEdit>
#include <QMutex>
#include <QTimer>

QT_BEGIN_NAMESPACE
class QPushButton;

class QLabel;

class QLineEdit;

class QHBoxLayout;

class QVBoxLayout;

QT_END_NAMESPACE

class QCustomWidget : public QWidget {
Q_OBJECT
public:
    explicit QCustomWidget(QWidget *parent = nullptr);

    ~QCustomWidget();

signals:

public slots:

protected:
    void paintEvent(QPaintEvent *e);

};

class QCustomMoveWidget : public QCustomWidget {
Q_OBJECT
public:
    explicit QCustomMoveWidget(QWidget *parent = 0);

    ~QCustomMoveWidget();

protected:
    QPoint mousePoint;
    bool m_mousePressed;

    void mouseMoveEvent(QMouseEvent *e);

    void mousePressEvent(QMouseEvent *e);

    void mouseReleaseEvent(QMouseEvent *e);
};

class QCustomDialog : public QDialog {
Q_OBJECT
public:
    explicit QCustomDialog(QWidget *parent = 0);

    ~QCustomDialog();

protected:
    QPoint mousePoint;
    bool m_mousePressed;
    QSize m_nNormalSize;

    void mouseMoveEvent(QMouseEvent *e);

    void mousePressEvent(QMouseEvent *e);

    void mouseReleaseEvent(QMouseEvent *);
};

class CBaseDialog : public QCustomDialog {
Q_OBJECT
public:
    explicit CBaseDialog(QWidget *parent = 0);

    ~CBaseDialog();

    void SetWinIcon(QPixmap pixmap);

    void SetWinTitle(const QString &text);

private:
    QWidget *widgetWinTitle;
    QLabel *labelWinIcon;
    QLabel *labelWinTitle;
    QPushButton *btnWinMin;
    QPushButton *btnWinClose;

protected:
    QWidget *widgetBody;
};

class CMessageBox : public CBaseDialog {
Q_OBJECT

public:
    typedef enum {
        E_Information = 0x01,
        E_Warning,
        E_Question,
        E_MSGTYPE_Error,
    } E_MSGTYPE;

public:
    explicit CMessageBox(QWidget *parent = 0);

    ~CMessageBox();

    void ShowMessage(const QString &content,
                     const quint8 &msgType = CMessageBox::E_Information,
                     const QString &title = "");

    void StartTimer();

    static int Infomation(QWidget *parent, const QString &content,
                          const QString &title = "Information");

    static int Question(QWidget *parent, const QString &content,
                        const QString &title = "Question");

    static int Warning(QWidget *parent, const QString &content,
                       const QString &title = "Warning");

protected:
private:
    QLabel *labelIcon;
    QLabel *labelMsgContent;

    QPushButton *btnOk;
    QPushButton *btnCancel;

    QTimer *m_timer;
    int m_nTimerCnt;
public slots:

    void SltTimerOut();
};

class CInputDialog : public CBaseDialog {
Q_OBJECT

public:
    explicit CInputDialog(QWidget *parent = 0);

    ~CInputDialog();

    static QString GetInputText(QWidget *parent, const QString &text = "",
                                const QString &title = "Go",
                                QLineEdit::EchoMode mode = QLineEdit::Normal);

    QString GetText() const;

    void SetInputText(const QString &text);

    void SetEchoMode(QLineEdit::EchoMode mode);

private:
    static CInputDialog *self;

    QLabel *labelText;
    QLineEdit *lineEditInput;

    QPushButton *btnOk;
    QPushButton *btnCancel;
};

#endif // QCUSTOMWIDGET_H
