#include "route.h"
#include <random>
#include <iostream>

const double R_MIN = PXL_DIMS/8;
const double R_MAX = PXL_DIMS/2;


Route::Route()
{
    QBrush routeBrush = QBrush(QColor::fromRgb(0,0,0),Qt::SolidPattern);
    routePen = QPen(routeBrush, c2p(1.7));
}

void Route::reset(int n){
    route.clear();

    float x = R_MAX;
    float y = R_MAX; // at center

    for(int i=0; i<n; ++i){
        float t = (2 * M_PI) * i / n;
        float r = R_MIN + (R_MAX - R_MIN) * float(rand())/RAND_MAX;
        route.push_back(QPointF(x + r*cos(t), y + r*sin(t)));
    }
    std::cout << "N : " << n << " ROUTE SIZE : " << route.size() << std::endl;

    poly = QPolygonF(route);

    poly_item.setPolygon(poly);
    poly_item.setPen(routePen);
}

void Route::draw(){

}
