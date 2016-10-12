#ifndef ROUTE_H
#define ROUTE_H

#include <QVector>
#include <QPointF>
#include <QPolygonF>
#include <QGraphicsPolygonItem>
#include <QPen>
#include <QString>
#include <QFile>
#include <QTextStream>

#include "utils.h"

struct Route
{
    //QVector<QPointF> route;
    QPolygonF poly;
    QGraphicsPolygonItem poly_item;
    QPen routePen;
public:
    Route();

    void draw();
    void reset(int n); //randomize to length
    void reset(const QVector<QPointF>& route);

    void save(const QString& filename);
    void load(const QString& filename);
};

extern Route* route;

#endif // ROUTE_H
