#include "mainwindow.h"
#include <QApplication>
#include <QSplashScreen>

int initPlatPPS,initVertPPS, initLayer;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    w.show();

    return a.exec();
}
