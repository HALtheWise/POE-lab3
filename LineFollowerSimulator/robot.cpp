#include "utils.h"
#include "robot.h"

#include <iostream>

// 1 pxl = .5 cm
// 270 x 270 cm world

// robot = 20x16 cm

Robot* robot = nullptr;

float coneRadius(float h, float fov){
    return h*tan(fov/2);
}

// RobotBody
Robot::Robot(QGraphicsScene& scene, QPointF pos, float theta, QPointF irOffset, float h, float fov):
    pos(pos),
    theta(theta),
    irOffset(irOffset),
    h(h),
    fov(fov),
    cr(coneRadius(h,fov))
{
    vel_l = vel_r = 0.;

    body = new RobotItem(pos,QPointF(ROBOT_LENGTH,ROBOT_WIDTH),irOffset, theta, cr);
    // RobotItem will take care of graphics
    scene.addItem(body);
}

Robot::~Robot(){
    if(body){
        delete body;
        body = nullptr;
    }
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
        // update position based on obtained kinematics prediction

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

void Robot::setPowerL(int pow){
    // convert power to velocity
    // then convert it back to pixel units
    setVelocityL(c2p(pow2vel(pow)));
}


void Robot::setPowerR(int pow){
    // convert power to velocity
    // then convert it back to pixel units
    setVelocityR(c2p(pow2vel(pow)));
}


void Robot::setVelocity(float l, float r){
    setVelocityL(l);
    setVelocityR(r);
}

void Robot::setIRHeight(float h){
    this->h = h;
    this->cr = coneRadius(h,fov);
    body->setCR(this->cr);
}

void Robot::sense(QImage& image){
    // poll a conical section of the ground-plane beneath the IR sensors
    // and average out the readings of pixels

    float l_1 = irOffset.x();
    float l_2 = irOffset.y();

    QPointF ir_root = pos + QPointF(l_1 * cos(theta), -l_1 * sin(theta));
    QPointF ir_l = ir_root - QPointF(l_2 * sin(theta), l_2 * cos(theta));
    QPointF ir_r = ir_root + QPointF(l_2 * sin(theta), l_2 * cos(theta));

    float cR = coneRadius(h,fov);
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

void Robot::setVisible(bool visible){
    body->setVisible(visible);
}
