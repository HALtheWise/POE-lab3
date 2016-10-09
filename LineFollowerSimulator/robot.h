#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>
#include <QPointF>

#include <QGraphicsEllipseItem>
#include <QGraphicsScene>

/*
class RobotBody : public QGraphicsRectItem{
public:
    RobotBody();

};
*/

class Robot
{

public:
    QPointF pos; // current position
    float theta; // heading, measured from horz.  radians

    float l_1; // vertical offset of IR sensor, inches
    float l_2; // horizontal offset of IR sensor, inches
    float h; // height of IR sensor, inches
    float fov; //field of view of IR sensor, radians

    float ir_val_l; //value of ir reflectance sensors
    float ir_val_r;

    QGraphicsItemGroup* robot;
    QGraphicsEllipseItem *body; //approximation
    QGraphicsEllipseItem *ir_l; //approximation
    QGraphicsEllipseItem *ir_r; //approximation


public:
    Robot(QGraphicsScene& scene, QPointF pos=QPointF(256,256), float theta=0.0, float l_1=10., float l_2=5.0, float h=2.0, float fov=M_PI/4);

    void draw();
    void reset(QPointF pos, float theta);
    void move(float delta, float dtheta);
    void sense(QImage& img);
    void setVisible(bool visible);
    float coneRadius();
};

#endif // ROBOT_H
