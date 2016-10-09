#include "robot.h"
#include <iostream>


// 1 pxl = .5 cm
// 270 x 270 cm world

// robot = 20x16 cm


std::ostream& operator<<(std::ostream& os, QPointF p){
    os << "{X : " << p.x() << ", Y : " << p.y() << "}";
}

float voltageToRPM(float v){
    // converts voltage to Angular Velocity
    return 0.0;
}

// RobotBody
Robot::Robot(QGraphicsScene& scene, QPointF pos, float theta, QPointF irOffset, float h, float fov):
    pos(pos),
    theta(theta),
    irOffset(irOffset),
    h(h),
    fov(fov),
    cr(h*tan(fov/2))
{
    vel_l = vel_r = 0.;

    body = new RobotItem(pos,QPointF(ROBOT_LENGTH,ROBOT_WIDTH),irOffset, theta, cr);
    scene.addItem(body);

    //ir_l = scene.addEllipse(0,0,0,0);
    //ir_r = scene.addEllipse(0,0,0,0);
    //body = scene.addRect(0,0,ROBOT_WIDTH,ROBOT_LENGTH);

    //QList<QGraphicsItem*> items({body,ir_l, ir_r});

    //robot = scene.createItemGroup(items);
}

Robot::~Robot(){
    delete body;
}

void Robot::reset(QPointF p, float t){
    pos=p; theta=t;
    body->setPos(pos,theta);
}

void Robot::move(float delta, float dtheta){
    theta += dtheta * DT;
    pos += delta * QPointF(cos(theta), -sin(theta)) * DT;
    update();
}

void Robot::update(){
    float w = (vel_l - vel_r) / WHEEL_DISTANCE;
    float R = (vel_l + vel_r) / (2*w);

    QPointF ICC = pos + R * QPointF(-sin(theta), -cos(theta));
    std::cout << ICC << std::endl;

    float x = pos.x();
    float y = pos.y();

    pos.setX(
                cos(w*DT) * (x - ICC.x()) +
                -sin(w*DT) * (y - ICC.y()) +
                ICC.x()
                );
    pos.setY(
                sin(w*DT) * (x - ICC.x()) +
                cos(w*DT) * (y - ICC.y()) +
                ICC.y()
                );
    theta += w * DT;

    body->setPos(pos, theta);
}

void Robot::setVelocity(float left, float right){
    vel_l = left, vel_r = right;
    update();
}

void Robot::sense(QImage& image){

    float l_1 = irOffset.x();
    float l_2 = irOffset.y();

    QPointF ir_root = pos + QPointF(l_1 * cos(theta), -l_1 * sin(theta));
    QPointF ir_l = ir_root - QPointF(l_2 * sin(theta), l_2 * cos(theta));
    QPointF ir_r = ir_root + QPointF(l_2 * sin(theta), l_2 * cos(theta));

    float cR = coneRadius();
    int i_cR = round(cR);

    //std::cout << "Cone RADIUS : " << cR << std::endl;
    int n = 0;
    float sum_l=0;
    float sum_r = 0;

    for(int offsetX = -i_cR; offsetX <= i_cR; ++offsetX){
        for(int offsetY = -i_cR; offsetY <= i_cR; ++offsetY){
            if((offsetX * offsetX + offsetY * offsetY) < (cR * cR)){
                QPointF offset(offsetX, offsetY);

                QColor col_l = image.pixelColor((ir_l + offset).toPoint());
                QColor col_r = image.pixelColor((ir_r + offset).toPoint());

                sum_l += col_l.redF();
                sum_r += col_r.redF();
                ++n;


            }
        }
    }

    // set ir values

    if(n){
        ir_val_l = sum_l / n;
        ir_val_r = sum_r / n;
    }
}

float Robot::coneRadius(){
    return h*tan(fov/2);
}

void Robot::setVisible(bool visible){
    //ir_l->setVisible(visible);
    //ir_r->setVisible(visible);
    body->setVisible(visible);
}
