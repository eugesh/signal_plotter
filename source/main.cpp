#include <QApplication>
#include <QTextCodec>
#include <QTime>
#include "main_window.h"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    // QTextCodec::setCodecForTr(QTextCodec::codecForName("Windows-1251"));
    // QTextCodec::setCodecForTr(QTextCodec::codecForName("UTF-8"));

    MainWindow view;

    view.show();
    return app.exec();
}
