#include "pid.h"

#ifdef _USE_MY_PID_

PID::PID(float k_p, float k_i, float k_d, float o_min, float o_max):
    k_p(k_p), k_i(k_i), k_d(k_d), o_min(o_min), o_max(o_max)
{
    e_d = e_i = 0.0; //
}

float PID::compute(float err){
    e_i += err * DT; // simulate "discrete integral"
    float res = k_p * err + k_i * e_i + k_d * (err - e_d) / DT;
    // technically we can fuse k_i as k_i * DT and k_d as k_d/DT... not doing that here for clarity

    e_d = err; //remember last error
    return limit(res, o_min, o_max);
}

void PID::set(float p, float i, float d){
    k_p = p, k_i = i, k_d = d;
}

#endif
