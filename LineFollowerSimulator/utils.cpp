#include "utils.h"


const float CM_DIMS = 256; // 256 cm x 256 cm

const float PXL_DIMS = 512; // 512 cm x 512 cm

const float DT = 1.0;



const float ROBOT_LENGTH = c2p(20);
const float ROBOT_WIDTH = c2p(16);

const float WHEEL_DISTANCE = c2p(13.0);

const float IR_OFFSETX = c2p(5);
const float IR_OFFSETY = c2p(5);

const float IR_HEIGHT = c2p(4);

const float IR_FOV = d2r(60);


float c2p(float cm){
    return cm * PXL_DIMS / CM_DIMS;
}

float min(float a, float b){
    return a<b?a:b;
}

float d2r(float d){
    return d / 180. * M_PI;
}
float r2d(float r){
    return r / M_PI * 180.;
}
