#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QFileDialog>

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

    //Route route;
    //Robot robot;

    int timerId;
    bool auto_control;
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

    void senseRobot();
    void setLeftPower(int);
    void setRightPower(int);
    void resetPower();
    void setIRHeight(int);

    void resetRoute();
    void loadRoute();
    void saveRoute();

    void setAuto(bool);
    void setSimAccel(double);
   protected:
    void timerEvent(QTimerEvent *event);
};

#endif // MAINWINDOW_H
