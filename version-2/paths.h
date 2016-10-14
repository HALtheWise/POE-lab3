#ifndef __PATHS_H__
#define __PATHS_H__

#include "odometry.h"

#ifndef __AVR__
#include "arduino.h"
#include "utils.h"
#else
#include <Arduino.h>
#endif

class PathPoint
{
private:
    // flags stores a packed representation of other information
    // included in a PathPoint. The LSB represents whether the robot
    // is expected to deviate from the line there, and the next three
    // bits represent robot speed 0...7
    byte flags;
public:
    double wrappedAngle;

    PathPoint();
    bool getOffLine( void );
    void setOffLine( bool offline );
    byte getSpeed( void );
    void setSpeed( byte speed );
};

class Path
{
public:
    Path(int length, bool useLeft);
    ~Path();
    PathPoint *points;
    int allocatedPoints = 0;
    int usedPoints = 0;

    bool useLeftSensor = false;

    void writeOut( void );
    bool attemptUpdate( Pose *pose );
    PathPoint *getPoint( double distAlong );
};
#endif
