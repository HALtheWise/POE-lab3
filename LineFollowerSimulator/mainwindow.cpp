#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>

#include <QGraphicsPolygonItem>

#include <QKeyEvent>
#include <QImage>
#include <QRgb>

QImage image(PXL_DIMS,PXL_DIMS,QImage::Format_ARGB32);

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    scene(new QGraphicsScene(0,0,PXL_DIMS,PXL_DIMS,this)),
    robot(scene, QPointF(PXL_DIMS/2,PXL_DIMS/2),0.0, QPointF(IR_OFFSETX, IR_OFFSETY), IR_HEIGHT, IR_FOV),
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
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    image.fill(Qt::white);

    QRectF area(0,0,PXL_DIMS,PXL_DIMS);

    // be invisible so that it doesn't detect itself
    robot.setVisible(false);
    scene.render(&painter,area,area);
    robot.setVisible(true);

    painter.end();
    robot.sense(image);

    std::cout << "IR VALUE : LEFT {" << robot.ir_val_l << "} , RIGHT {" << robot.ir_val_r << '}' << std::endl;
}

void MainWindow::keyReleaseEvent(QKeyEvent *event){

    //key teleop for testing

    switch(event->key()){
    case Qt::Key_Up:
        robot.setVelocity(-0.1,0.0);
        //robot.move(1.0,0.0);
        break;
    case Qt::Key_Down:
        robot.setVelocity(0.1,0.0);
        break;
    case Qt::Key_Left: // rotate counterclockwise
        robot.setVelocity(0.0,-0.1);
        break;
    case Qt::Key_Right: // rotate clockwise
        robot.setVelocity(0.0,0.1);
        break;
    }

    senseRobot();
}
