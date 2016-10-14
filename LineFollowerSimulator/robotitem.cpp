#include "robotitem.h"


RobotItem::RobotItem(QPointF pos, QPointF dims, QPointF irOffset, float theta, float cr):
pos(pos),dims(dims),irOffset(irOffset),theta(theta),cr(cr){
    setFlag(QGraphicsItem::ItemIsSelectable);
    setFlag(QGraphicsItem::ItemIsMovable);
    setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    setAcceptDrops(true);
}

QRectF RobotItem::boundingRect() const{
    return QRectF(pos - dims/2, pos + dims/2);
}

void RobotItem::setPos(QPointF p, float t){
    pos = p;
    theta = t;
}

void RobotItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){

    painter->translate(pos);
    painter->rotate(r2d(-theta));

    painter->drawRect(QRectF(-dims/2,dims/2));
    painter->drawEllipse(QPointF(irOffset.x(), -irOffset.y()),cr,cr); // cr = cone radius
    painter->drawEllipse(irOffset,cr,cr);

}

void RobotItem::setCR(float cr){
    this->cr = cr;
}
