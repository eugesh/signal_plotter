#include <stdio.h>
#include <math.h>
#include <QApplication>
#include <QTextCodec>
#include <QTime>
// #include <QUdpSocket>
// #include <QHostAddress>
#include <QProcess>
#include <QImage>
#include <QFile>
#include <QTextStream>
#include <QGraphicsView>
#include "main_window.h"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    // QTextCodec::setCodecForTr(QTextCodec::codecForName("Windows-1251"));
    QTextCodec::setCodecForTr(QTextCodec::codecForName("UTF-8"));

    MainWindow view;

    view.show();
    return app.exec();
}
