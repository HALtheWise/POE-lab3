#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>
#include <QPointF>

#include <QGraphicsEllipseItem>
#include <QGraphicsScene>

class Robot
{

public:
    QPointF pos; // current position
    float theta; // heading, measured from horz.  radians

    float l_1; // vertical offset of IR sensor, inches
    float l_2; // horizontal offset of IR sensor, inches
    float h; // height of IR sensor, inches
    float fov; //field of view of IR sensor, radians

    QGraphicsItemGroup* robot;
    QGraphicsEllipseItem *body; //approximation
    QGraphicsEllipseItem *ir_1; //approximation
    QGraphicsEllipseItem *ir_2; //approximation


public:
    Robot(QGraphicsScene& scene, QPointF pos=QPointF(128,128), float theta=0.0, float l_1=1.0, float l_2=1.0, float h=0.5, float fov=M_PI/4);

    void draw();
    void reset(QPointF pos, float theta);
    void move(float delta, float dtheta);
    void sense();
    float coneRadius();
};

#endif // ROBOT_H
