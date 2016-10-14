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

    QPointF irOffset; // position of ir from center

    float vel_l, vel_r; //left-right velocity of motors

    float h; // height of IR sensor, inches
    float fov; //field of view of IR sensor, radians

    float ir_val_l; //value of ir reflectance sensors
    float ir_val_r;

    // frequently computed values
    float cr; //cone radius

    RobotItem* body;

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

    void setPowerR(int r);
    void setPowerL(int l);

    void setVelocityR(float r);
    void setVelocityL(float l);

    void setIRHeight(float h);

    void sense(QImage& img);
    void setVisible(bool visible);
};

extern Robot* robot;

#endif // ROBOT_H
