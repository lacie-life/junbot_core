#include "mainwindow.h"

#include <QApplication>
#include <QDesktopWidget>
#include <QtGui>

#include "QLoginWidget.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QLoginWidget w;
    w.show();

    app.connect(&app, &QApplication::lastWindowClosed, &app, QApplication::quit);

    return app.exec();
}
