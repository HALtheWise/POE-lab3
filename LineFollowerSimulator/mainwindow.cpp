#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>

#include <QGraphicsPolygonItem>

#include <QKeyEvent>
#include <QImage>
#include <QRgb>

QImage image(512,512,QImage::Format_ARGB32);

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    scene(new QGraphicsScene(0,0,512,512,this)),
    robot(scene),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->graphicsView->setScene(&scene);

    connect(ui->resetRobotBtn, SIGNAL (released()), this, SLOT (resetRobot()));
    connect(ui->resetRouteBtn, SIGNAL (released()), this, SLOT (resetRoute()));
    ui->routePointsEdit->setValidator(new QIntValidator(3,100,this));

    resetRobot();
    resetRoute(10);

    scene.addItem(&route.poly_item);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::resetRoute(int n){
    if(n >= 3){
        //somehow validator didn't work...?
        route.reset(n);
    }
}

void MainWindow::resetRoute(){
    QString s = ui->routePointsEdit->text();
    int n = s.toInt();
    resetRoute(n);
}

void MainWindow::resetRobot(){
    // not implemented yet
    // put robot on the path
    // and give it heading tangent to path
}


void MainWindow::senseRobot(){
    std::cout << "SENSEROBOT " << std::endl;
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    image.fill(Qt::white);
    QRectF area(0,0,512,512);
    robot.setVisible(false);
    scene.render(&painter,area,area);
    robot.setVisible(true);

    painter.end();

    robot.sense(image);
    image.save("WTF.png");
}

void MainWindow::keyReleaseEvent(QKeyEvent *event){

    switch(event->key()){
    case Qt::Key_Up:
        robot.move(1.0,0.0);
        break;
    case Qt::Key_Down:
        robot.move(-1.0,0.0);
        break;
    case Qt::Key_Left:
        robot.move(0.0,1.0);
        break;
    case Qt::Key_Right:
        robot.move(0.0,-1.0);
        break;
    }
    senseRobot();
}
