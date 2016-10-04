#include "robot.h"

Robot::Robot(QGraphicsScene& scene, QPointF pos, float theta, float l_1, float l_2, float h, float fov):
    pos(pos),theta(theta),l_1(l_1),l_2(l_2),h(h),fov(fov)
{
    //ir_1 = QGraphicsEllipseItem();
    //ir_2 = QGraphicsEllipseItem();
    //body = QGraphicsEllipseItem();
    ir_1 = scene.addEllipse(0,0,0,0);
    ir_2 = scene.addEllipse(0,0,0,0);
    body = scene.addEllipse(0,0,0,0);

    QList<QGraphicsItem*> items({body,ir_1, ir_2});
    robot = scene.createItemGroup(items);
}

void Robot::reset(QPointF p, float t){
    pos=p; theta=t;
}

void Robot::move(float delta, float dtheta){
    theta += dtheta;
    pos += QPointF(delta * cos(theta), delta * -sin(theta));

    QPointF ir_root_pos = pos + QPointF(l_1 * cos(theta), -l_1 * sin(theta));
    QPointF ir_1_pos = ir_root_pos + QPointF(l_2 * sin(theta), l_2 * cos(theta));
    QPointF ir_2_pos = ir_root_pos - QPointF(l_2 * sin(theta), l_2 * cos(theta));

    //body->setRect(0,0,0,0);

    body->setRect(pos.x() - 5.0, pos.y() - 5.0, 10.0, 10.0);
    ir_1->setRect(ir_1_pos.x() -2.5,ir_1_pos.y() - 2.5,5.0,5.0);
    ir_2->setRect(ir_2_pos.x() -2.5 ,ir_2_pos.y() - 2.5 ,5.0,5.0);

}

void Robot::sense(){
    QPointF ir_root = pos + QPointF(l_1 * cos(theta), -l_1 * sin(theta));

    QPointF ir_1 = ir_root + QPointF(l_2 * sin(theta), l_2 * cos(theta));

    QPointF ir_2 = ir_root - QPointF(l_2 * sin(theta), l_2 * cos(theta));

    // radius of scan area = h * tan(fov/2), assuming "Cone" model

}

float Robot::coneRadius(){
    return h*tan(fov/2);
}
