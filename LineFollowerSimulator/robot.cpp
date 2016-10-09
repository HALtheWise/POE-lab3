#include "robot.h"
#include <iostream>


// 1 pxl = .5 cm
// 270 x 270 cm world


// robot = 20x16 cm

float voltageToRPM(float v){
    // converts voltage to Angular Velocity
    return 0.0;
}

Robot::Robot(QGraphicsScene& scene, QPointF pos, float theta, float l_1, float l_2, float h, float fov):
    pos(pos),theta(theta),l_1(l_1),l_2(l_2),h(h),fov(fov)
{
    //ir_l = QGraphicsEllipseItem();
    //ir_r = QGraphicsEllipseItem();
    //body = QGraphicsEllipseItem();

    ir_l = scene.addEllipse(0,0,0,0);
    ir_r = scene.addEllipse(0,0,0,0);
    body = scene.addEllipse(0,0,0,0);

    QList<QGraphicsItem*> items({body,ir_l, ir_r});

    robot = scene.createItemGroup(items);
}

void Robot::reset(QPointF p, float t){
    pos=p; theta=t;
}

void Robot::move(float delta, float dtheta){
    theta += dtheta;
    pos += QPointF(delta * cos(theta), delta * -sin(theta));

    QPointF ir_root_pos = pos + QPointF(l_1 * cos(theta), -l_1 * sin(theta));
    QPointF ir_l_pos = ir_root_pos + QPointF(l_2 * sin(theta), l_2 * cos(theta));
    QPointF ir_r_pos = ir_root_pos - QPointF(l_2 * sin(theta), l_2 * cos(theta));

    //body->setRect(0,0,0,0);

    body->setRect(pos.x() - 10.0, pos.y() - 10.0, 20.0, 20.0);
    ir_l->setRect(ir_l_pos.x() -2.5,ir_l_pos.y() - 2.5,5.0,5.0);
    ir_r->setRect(ir_r_pos.x() -2.5 ,ir_r_pos.y() - 2.5 ,5.0,5.0);

}

void Robot::sense(QImage& image){
    QPointF ir_root = pos + QPointF(l_1 * cos(theta), -l_1 * sin(theta));
    QPointF ir_l = ir_root + QPointF(l_2 * sin(theta), l_2 * cos(theta));
    QPointF ir_r = ir_root - QPointF(l_2 * sin(theta), l_2 * cos(theta));
    std::cout << "ROBOT POSITION : " << pos.x() << ',' << pos.y() << std::endl;

    float cR = coneRadius();
    int i_cR = round(cR);

    //std::cout << "Cone RADIUS : " << cR << std::endl;
    int n = 0;
    float sum_l=0;
    float sum_r = 0;

    std::cout << "i_CR : " << i_cR << std::endl;
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
    ir_l->setVisible(visible);
    ir_r->setVisible(visible);
    body->setVisible(visible);
}
