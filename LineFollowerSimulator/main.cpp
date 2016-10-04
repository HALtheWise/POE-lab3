#include "mainwindow.h"
#include <QApplication>

int analogRead(){
    //surveys area and takes mean
    //maps back to 0~1023
    return 0;
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
