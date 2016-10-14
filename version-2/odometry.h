#include <Arduino.h>

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

Pose::Pose() {
	reset();
}

void Pose::writeOut( void ) {
	Serial.print("(d, theta) = (");
	Serial.print(distAlong);
	Serial.print(", ");
	Serial.print(angleFrom);
	Serial.print(")");
}

void Pose::reset(){
	distAlong = 0;
	angleFrom = 128;
}

const double MAX_FORWARD_SPEED = 100;
const double MAX_TURN_SPEED = 1000;

double adjustPower( int power ){
	double powerfrac = power / 255.0;
	return powerfrac - .05 * powerfrac * powerfrac;
}

// Update the estimated pose of the robot based on wheel (estimated) odometry.
void Pose::odometryUpdate( int leftPower, int rightPower, int timestep) {
	double turnFudge = (state == STATE_MEMORIZE) ? MEMORIZATION_TURN_FUDGE : 1;

	double leftPowerFrac  = adjustPower(leftPower);
	double rightPowerFrac = adjustPower(rightPower);
	double forwardSpeed = (leftPowerFrac + rightPowerFrac) / 2 * MAX_FORWARD_SPEED;
	double turnSpeed = (rightPowerFrac - leftPowerFrac) / 2 * MAX_TURN_SPEED * turnFudge;

	distAlong += forwardSpeed * timestep / 1000.0;
	angleFrom += turnSpeed * timestep / 1000.0;
}