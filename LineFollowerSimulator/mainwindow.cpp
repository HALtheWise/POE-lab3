#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "utils.h"
#include "pid.h"

#include "arduino.h"

#include <iostream>

#include <QGraphicsPolygonItem>

#include <QKeyEvent>
#include <QImage>
#include <QRgb>

QImage image(PXL_DIMS,PXL_DIMS,QImage::Format_ARGB32);

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    scene(new QGraphicsScene(0,0,PXL_DIMS,PXL_DIMS,this)),
    ui(new Ui::MainWindow)
{
    robot = new Robot(scene, QPointF(PXL_DIMS/2,PXL_DIMS/2),0.0, QPointF(IR_OFFSETX, IR_OFFSETY), IR_HEIGHT, IR_FOV);
    route = new Route();

    auto_control = false;
    ui->setupUi(this);
    ui->graphicsView->setScene(&scene);

    connect(ui->resetRobotBtn, SIGNAL (released()), this, SLOT (resetRobot()));
    connect(ui->resetRouteBtn, SIGNAL (released()), this, SLOT (resetRoute()));

    connect(ui->actionLoad_Route, SIGNAL(triggered(bool)), this, SLOT(loadRoute()));
    connect(ui->actionSave_Route, SIGNAL(triggered(bool)), this, SLOT(saveRoute()));
    connect(ui->actionExit,SIGNAL(triggered(bool)), this, SLOT(close()));

    resetRoute(10);
    resetRobot();

    scene.addItem(&route->poly_item);

    setup(); // begin arduino

    timerId = startTimer(DT * 1000 / SIMULATION_ACCELARATION);// DT in sec
}

MainWindow::~MainWindow()
{
    killTimer(timerId);
    delete ui;
}

void MainWindow::resetRoute(int n){
    if(n >= 3){
        //somehow validator didn't work...?
        route->reset(n);
    }
}

void MainWindow::resetRoute(){
    int n = ui->n_routes_spin->value();
    resetRoute(n);
    robot->reset(route->poly);
}

void MainWindow::resetRobot(){
    robot->reset(route->poly);
}


void MainWindow::senseRobot(){
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    image.fill(Qt::white);

    QRectF area(0,0,PXL_DIMS,PXL_DIMS);

    // be invisible so that it doesn't detect itself
    robot->setVisible(false);
    scene.render(&painter,area,area);
    robot->setVisible(true);

    painter.end();
    robot->sense(image);

    ui->ir_left_edit->setText(QString::number(robot->ir_val_l));
    ui->ir_right_edit->setText(QString::number(robot->ir_val_r));

    //std::cout << "IR VALUE : LEFT {" << robot->ir_val_l << "} , RIGHT {" << robot->ir_val_r << '}' << std::endl;
}

void MainWindow::keyReleaseEvent(QKeyEvent *event){

    //key teleop for testing

    switch(event->key()){
    case Qt::Key_Up:
        ui->l_pow_slider->setValue(ui->l_pow_slider->value() + 1);
        break;
    case Qt::Key_Down:
        ui->l_pow_slider->setValue(ui->l_pow_slider->value() -  1);
        break;
    case Qt::Key_Left: // rotate counterclockwise

        ui->r_pow_slider->setValue(ui->r_pow_slider->value() - 1);
        break;
    case Qt::Key_Right: // rotate clockwise
        ui->r_pow_slider->setValue(ui->r_pow_slider->value() + 1);
        break;
    }

    //senseRobot();
}

void MainWindow::timerEvent(QTimerEvent *){
    senseRobot();

    if(auto_control){
        loop();
    }

    robot->update();

}

void MainWindow::setRightPower(int pow){
    robot->setPowerR(pow);
}

void MainWindow::setLeftPower(int pow){
    robot->setPowerL(pow);
}

void MainWindow::resetPower(){
    //basically stop the robot
    ui->l_pow_slider->setValue(0);
    ui->r_pow_slider->setValue(0);
    setLeftPower(0);
    setRightPower(0);
}


void MainWindow::setIRHeight(int h){
    robot->setIRHeight(c2p(h / 10.0)); // scaling factor for slider
}

void MainWindow::setAuto(bool a){
    //std::cout << a << std::endl;
    auto_control = a;
    if(!a){
        //stop the robot when converting back to manual mode
        resetPower();
    }
}


void MainWindow::setSimAccel(double accel){
    //restart timer with new accelaration
    killTimer(timerId);
    SIMULATION_ACCELARATION = accel;
    timerId = startTimer(DT * 1000 / SIMULATION_ACCELARATION);
}

void MainWindow::saveRoute(){
    //QFileDialog::setDefaultSuffix(".txt");
    QString filter = "Text File (*.txt)";
    QString fileName = QFileDialog::getSaveFileName(this,
        tr("Save Route"), "",
        filter,&filter);

    if(!fileName.endsWith(".txt", Qt::CaseSensitive)){
        fileName.append(".txt");
    }

    route->save(fileName);
}

void MainWindow::loadRoute(){
    QString filter = "Text File (*.txt)";
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Load Route"), "", filter);
    if(fileName != ""){
        route->load(fileName);
    }
}
