#include <Arduino.h>

// This file attempts to estimate the ego-motion of the robot
// using the commanded powers of the motors. Expect accuracy of += 15%

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
	angleFrom = 0;
}

// Neither of these constants matter much as long as they do not
// change between recording and playback.
// MAX_FORWARD_SPEED is scaled to approximately 1 = 1cm on our robot,
// while MAX_TURN_SPEED is approximately 1 = 1/10 degree.
const double MAX_FORWARD_SPEED = 100;
const double MAX_TURN_SPEED = 1000;

double adjustPower( int power ){
	double powerfrac = power / 255.0;
	return powerfrac - .05 * powerfrac * powerfrac;
}

// Update the estimated pose of the robot based on wheel (estimated) odometry.
// Currently uses an almost linear approximation, with slight falloff at high powers.
// timestep is in ms
void Pose::odometryUpdate( int leftPower, int rightPower, int timestep) {
	if (timestep > 100)
	{
		// Prevent large timesteps from causing jumps in odometry readings (for example, on first boot)
		return;
	}

	double leftPowerFrac  = adjustPower(leftPower);
	double rightPowerFrac = adjustPower(rightPower);
	double forwardSpeed = (leftPowerFrac + rightPowerFrac) / 2 * MAX_FORWARD_SPEED;
	double turnSpeed = (rightPowerFrac - leftPowerFrac) / 2 * MAX_TURN_SPEED;

	distAlong += forwardSpeed * timestep / 1000.0;
	angleFrom += turnSpeed * timestep / 1000.0;
}