#include "odometry.h"

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

const double MAX_FORWARD_SPEED = 189; // ~189 cm / s at POW = 255 (12V, although unachievable)
const double MAX_TURN_SPEED = 500; // units of ...??

double adjustPower( int power ){
    double powerfrac = power / 255.0;
    return powerfrac - .05 * powerfrac * powerfrac;
}

// Update the estimated pose of the robot based on wheel (estimated) odometry.
void Pose::odometryUpdate( int leftPower, int rightPower, int timestep) {
    double leftPowerFrac  = adjustPower(leftPower);
    double rightPowerFrac = adjustPower(rightPower);

    double forwardSpeed = (leftPower + rightPower) / 2 / 255. * MAX_FORWARD_SPEED;

    //Serial.println(leftPower);
    double turnSpeed = (rightPower - leftPower) / 2 / 255. * MAX_TURN_SPEED;

    distAlong += forwardSpeed * timestep / 1000.0;
    angleFrom += turnSpeed * timestep / 1000.0;
}
