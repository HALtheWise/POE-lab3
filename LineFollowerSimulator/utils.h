#ifndef __UTILS_H__
#define __UTILS_H__

#include <math.h>

extern const float CM_DIMS; // 256 cm x 256 cm
extern const float PXL_DIMS; // 512 cm x 512 cm
extern const float DT; // delta time

extern float c2p(float cm); // cm 2 pxl
extern float min(float a, float b);

extern float d2r(float d);
extern float r2d(float r);


extern const float ROBOT_WIDTH;
extern const float ROBOT_LENGTH;

extern const float WHEEL_DISTANCE;

extern const float IR_OFFSETX;
extern const float IR_OFFSETY;

extern const float IR_HEIGHT;
extern const float IR_FOV;

#endif // UTILS_H
