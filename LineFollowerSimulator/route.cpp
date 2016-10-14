#include "route.h"
#include <random>
#include <iostream>

const double R_MIN = PXL_DIMS/8;
const double R_MAX = PXL_DIMS/2;

Route* route;

Route::Route()
{
    QBrush routeBrush = QBrush(QColor::fromRgb(0,0,0),Qt::SolidPattern);
    routePen = QPen(routeBrush, c2p(1.7));
}

void Route::reset(int n){
    // reinitialize to a route of n points

    QVector<QPointF> route;

    float x = R_MAX;
    float y = R_MAX; // at center

    for(int i=0; i<n; ++i){
        float t = (2 * M_PI) * i / n;
        float r = R_MIN + (R_MAX - R_MIN) * float(rand())/RAND_MAX;
        route.push_back(QPointF(x + r*cos(t), y + r*sin(t)));
    }
    std::cout << "N : " << n << " ROUTE SIZE : " << route.size() << std::endl;
    reset(route);
}

void Route::reset(const QVector<QPointF> &route){
    // reset from vector
    poly = QPolygonF(route);
    poly_item.setPolygon(poly);
    poly_item.setPen(routePen);
}

void Route::save(const QString& filename){
    // save route to file
    QFile file(filename);
    if (file.open(QIODevice::ReadWrite)) {
        QTextStream stream(&file);

        for(auto& p : poly){
            stream << p.x() << ',' << p.y() << '\n';
        }
    }
    file.close();
}

void Route::load(const QString& filename){
    // load route from file
    QFile file(filename);
    if(file.open(QIODevice::ReadOnly)) {
        QTextStream stream(&file);
        QVector<QPointF> route;

        while(!stream.atEnd()){
            QString line = stream.readLine();
            QStringList pt_s = line.split(",");
            QPointF pt(pt_s[0].toFloat(),pt_s[1].toFloat());
            route.push_back(pt);
        }

        file.close();
        Route::reset(route);
    }
}
