#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>
#include <QPointF>

#include <QPolygonF>

#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>

#include <QGraphicsScene>

#include "utils.h"
#include "robotitem.h"


class Robot
{

public:
    QPointF pos; // current position
    float theta; // heading, measured from horz.  radians

    QPointF irOffset;

    float vel_l, vel_r;

    float h; // height of IR sensor, inches
    float fov; //field of view of IR sensor, radians

    float ir_val_l; //value of ir reflectance sensors
    float ir_val_r;

    // frequently computed values
    float cr; //cone radius

    RobotItem* body;

    //QGraphicsItemGroup* robot;
    //QGraphicsRectItem *body; //approximation
    //QGraphicsEllipseItem *ir_l; //approximation
    //QGraphicsEllipseItem *ir_r; //approximation

public:
    Robot(QGraphicsScene& scene,
          QPointF pos, float theta,
          QPointF irOffset, float h, float fov);
    ~Robot();

    void reset(QPointF pos, float theta);
    void reset(QPolygonF route);

    void update();
    void move(float delta, float dtheta);
    void setVelocity(float left, float right);
    void setVelocityR(float r);
    void setVelocityL(float l);
    void setIRHeight(float h);

    void sense(QImage& img);
    void setVisible(bool visible);
};

#endif // ROBOT_H
