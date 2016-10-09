#include "robot.h"
#include <iostream>


// 1 pxl = .5 cm
// 270 x 270 cm world

// robot = 20x16 cm


std::ostream& operator<<(std::ostream& os, QPointF p){
    os << "{X : " << p.x() << ", Y : " << p.y() << "}";
    return os;
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
}

Robot::~Robot(){
    delete body;
}

void Robot::reset(QPointF p, float t){
    pos=p; theta=t;
    body->setPos(pos,theta);
}

void Robot::reset(QPolygonF route){
    pos = route.front();
    float dst_x = route[1].x();
    float dst_y = route[1].y();

    theta = atan2(pos.y() - dst_y,dst_x - pos.x());

}

void Robot::move(float delta, float dtheta){
    theta += dtheta * DT;
    pos += delta * QPointF(cos(theta), -sin(theta)) * DT;
    update();
}

void Robot::update(){
    float w = (vel_r - vel_l) / WHEEL_DISTANCE;
    if(w != 0){
        float R = (vel_l + vel_r) / (2*w);

        QPointF ICC = pos + R * QPointF(-sin(theta), -cos(theta));
        // ICC = virtual center of rotation

        //std::cout << ICC << std::endl;

        float x = pos.x();
        float y = pos.y();
        float iccx = ICC.x();
        float iccy = ICC.y();

        pos.setX(
                    cos(-w*DT) * (x - iccx) +
                    -sin(-w*DT) * (y - iccy) +
                    iccx
                    );
        pos.setY(
                    sin(-w*DT) * (x - iccx) +
                    cos(-w*DT) * (y - iccy) +
                    iccy
                    );
        theta += w * DT;
    }else{
        // if w == 0, then division by zero would be bad..
        // since it's a special (and well-defined) case, we should handle this
        pos += vel_l * QPointF(cos(theta), -sin(theta)) * DT;
    }


    body->setPos(pos, theta);
}
void Robot::setVelocityL(float v){
    vel_l = v;
}

void Robot::setVelocityR(float v){
    vel_r = v;
}

void Robot::setVelocity(float l, float r){
    setVelocityL(l);
    setVelocityR(r);
}

void Robot::sense(QImage& image){

    float l_1 = irOffset.x();
    float l_2 = irOffset.y();

    QPointF ir_root = pos + QPointF(l_1 * cos(theta), -l_1 * sin(theta));
    QPointF ir_l = ir_root - QPointF(l_2 * sin(theta), l_2 * cos(theta));
    QPointF ir_r = ir_root + QPointF(l_2 * sin(theta), l_2 * cos(theta));

    float cR = coneRadius();
    int i_cR = round(cR);

    int n = 0;
    float sum_l=0;
    float sum_r = 0;

    for(int offsetX = -i_cR; offsetX <= i_cR; ++offsetX){
        for(int offsetY = -i_cR; offsetY <= i_cR; ++offsetY){
            if((offsetX * offsetX + offsetY * offsetY) < (cR * cR)){
                QPointF offset(offsetX, offsetY);

                QRgb col_l = image.pixel((ir_l + offset).toPoint());
                QRgb col_r = image.pixel((ir_r + offset).toPoint());

                sum_l += qGray(col_l) / 255.0;
                sum_r += qGray(col_r) / 255.0;
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
    body->setVisible(visible);
}
