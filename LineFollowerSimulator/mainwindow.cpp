#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>

#include <QGraphicsPolygonItem>

#include <QKeyEvent>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    scene(new QGraphicsScene(this)),
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
    // and give it tangent heading to path
}


void MainWindow::senseRobot(){
    QPixmap pixmap;
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    scene.render(&painter);
    painter.end();


    QImage monomap = pixmap.toImage().convertToFormat(QImage::Format_Mono);
    int cR = robot.coneRadius();
    for(int offsetX = -cR; offsetX < cR; ++offsetX){
        for(int offsetY = -cR; offsetX < cR; ++offsetX){
            QPointF offset(offsetX, offsetY);
            QRgb col = monomap.pixel((robot.pos + offset).toPoint());
            std::cout << col << std::endl;
        }
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *event){

    switch(event->key()){
    case Qt::Key_Up:
        robot.move(-1.0,0.0);
        break;
    case Qt::Key_Down:
        robot.move(1.0,0.0);
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
