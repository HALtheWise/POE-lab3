#include "utils.h"


const float CM_DIMS = 256; // 256 cm x 256 cm

const float PXL_DIMS = 512; // 512 cm x 512 cm

const float DT = 0.01;

float SIMULATION_ACCELARATION = 1.;

const float ROBOT_LENGTH = c2p(20);
const float ROBOT_WIDTH = c2p(16);

const float WHEEL_DISTANCE = c2p(13.0);

const float IR_OFFSETX = c2p(8);
const float IR_OFFSETY = c2p(4);

const float IR_HEIGHT = c2p(2);

const float IR_FOV = d2r(60);

float i2c(float in){
    // inches to cm
    return 2.54 * in;
}

float c2p(float cm){
    // cm to pxl
    return cm * PXL_DIMS / CM_DIMS;
}

float min(float a, float b){
    return a<b?a:b;
}

float max(float a, float b){
    return a>b?a:b;
}

float d2r(float d){
    return d / 180. * M_PI;
}
float r2d(float r){
    return r / M_PI * 180.;
}


float map(float val, float input_min, float input_max, float output_min, float output_max){
    return (val - input_min) / (input_max - input_min) * (output_max - output_min) + output_min;
}

float pow2vel(float v){
    return map(v, 0., 255., 0., 180.); // 180 is a rough extrapolation

    // based on 0=0
    // 50 = ~ 68cm / 2 sec.
    // 96 = ~ 135 cm / 2 sec.
    // 128 = ~ 185cm / 2 sec.
    // rough calibration, but didn't have a better way
}

float limit(float x, float min, float max){
    if(x < min)
        return min;
    if(x > max)
        return max;
    return x;
}


std::ostream& operator<<(std::ostream& os, QPointF p){
    os << "(" << p.x() << "," << p.y() << ")";
    return os;
}

typedef unsigned char byte;
