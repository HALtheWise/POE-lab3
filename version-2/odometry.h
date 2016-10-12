#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#include "arduino.h"
#include "utils.h"

class Pose
{
public:
	Pose();
	void odometryUpdate(int leftPower, int rightPower, int timestep);
	void writeOut( void );
	void reset();
	// Measured in cm, with positive being forward of the starting position
	double distAlong;
	// Measured in degrees, with positive being nose left.
	double angleFrom;
};

#endif
