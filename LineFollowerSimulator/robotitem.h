#ifndef ROBOTITEM_H
#define ROBOTITEM_H

#include <QObject>
#include <QPointF>
#include <QWidget>
#include <QPainter>
#include <QGraphicsItem>

#include "utils.h"

class RobotItem : public QGraphicsItem{
private:
    QPointF pos;
    QPointF dims;
    QPointF irOffset;
    float theta;
    //frequently requested derived features
    float cr;
public:
    RobotItem(QPointF pos, QPointF dims, QPointF irOffset, float theta, float cr);
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void setPos(QPointF pos, float theta);
    void setCR(float cr);
    QPointF irPos_r();
    QPointF irPos_l();

};

#endif // ROBOTITEM_H
