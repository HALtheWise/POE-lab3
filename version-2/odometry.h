class Pose
{
public:
	// Measured in cm, with positive being forward of the starting position
	double distAlong;
	// Measured in degrees, with positive being nose left.
	double angleFrom;
};

Pose robotPose = {0,0};

inline double batteryVoltage() {
	return 12;
}

const double MAX_FORWARD_SPEED = 100;
const double MAX_TURN_SPEED = 500;

// Update the estimated pose of the robot based on wheel (estimated) odometry.
void odometryUpdate(Pose *robotPose, int leftPower, int rightPower, int timestep) {
	double forwardSpeed = (leftPower + rightPower) / 2 * MAX_FORWARD_SPEED / 255.0;
	double turnSpeed = (rightPower - leftPower) / 2 * MAX_TURN_SPEED / 255.0;

	robotPose->distAlong += forwardSpeed * timestep / 1000.0;
	robotPose->angleFrom += turnSpeed * timestep / 1000.0;
}