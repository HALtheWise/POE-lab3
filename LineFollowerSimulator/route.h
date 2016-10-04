#ifndef ROUTE_H
#define ROUTE_H

#include <QVector>
#include <QPointF>
#include <QPolygonF>
#include <QGraphicsPolygonItem>
#include <QPen>

struct Route
{
    QVector<QPointF> route;
    QPolygonF poly;
    QGraphicsPolygonItem poly_item;
    QPen routePen;
public:
    Route();

    void draw();
    void reset(int n); //randomize to length
};

#endif // ROUTE_H
