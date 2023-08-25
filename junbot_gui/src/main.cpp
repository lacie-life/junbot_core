#include <QApplication>
#include <QDesktopWidget>
#include <QtGui>

#include "QLoginWidget.h"

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QApplication app(argc, argv);
    QLoginWidget w(argc, argv);
    w.show();

    app.connect(&app, &QApplication::lastWindowClosed, &app, QApplication::quit);

    return app.exec();
}
