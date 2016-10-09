#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QGraphicsView>
#include <QGraphicsScene>

#include "utils.h"
#include "route.h"
#include "robot.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
private:
    QGraphicsScene scene;

    Route route;
    Robot robot;

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
public:
    void keyReleaseEvent(QKeyEvent*);
    void resetRoute(int n);
public slots:
    void resetRobot();
    void resetRoute();
    void senseRobot();
};

#endif // MAINWINDOW_H
