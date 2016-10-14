#ifndef PID_H
#define PID_H

//currently disabled over PID_v1, which is written for the arduino

#ifdef _USE_MY_PID_
#include "utils.h"

struct PID
{
public:
    float k_p, k_i, k_d; // pid consts
    float e_d; // last error
    float e_i; // sum of errors
    float o_min, o_max; // output limits

    // I'm making these public mostly because I'm lazy
public:
    float compute(float err);
    void set(float,float,float);
    PID(float k_p, float k_i, float k_d, float o_min = 0.0, float o_max = 1.0);
};
#endif

#endif // PID_H
