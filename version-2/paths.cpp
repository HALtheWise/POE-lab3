#include "paths.h"

#ifdef __AVR__
#include <Arduino.h>
#endif


PathPoint::PathPoint( void ){
    flags = 0x0e; // Full speed, on line
    wrappedAngle = 0;
}

bool PathPoint::getOffLine( void ){
    return flags & 0x01;
}

void PathPoint::setOffLine( bool offline ){
    flags = (flags & 0xfe) | (offline ? 1 : 0);
}

byte PathPoint::getSpeed( void ){
    return (flags & 0x0e) >> 1;
}

void PathPoint::setSpeed( byte speed ){
    flags = (flags & 0xf1) | (speed << 1);
}

Path::Path(int length, bool useLeft){
    useLeftSensor = useLeft;

    allocatedPoints = length;

    points = (PathPoint*) malloc(sizeof(PathPoint) * length);

    for (int i = 0; i < allocatedPoints; ++i)
    {
        points[i] = PathPoint();
    }
}

Path::~Path(){
    if(points){
        free(points);
        points = nullptr;
    }

}

PathPoint *Path::getPoint( double distAlong ){
    //Serial.println(usedPoints);
    int index = min(distAlong, usedPoints); // getting rid of -1 here ...
    if (index < 0)
    {
        return nullptr;
    }
    return &points[index];
}

bool Path::attemptUpdate( Pose *pose ) {
    if (int(pose->distAlong) > usedPoints && usedPoints < allocatedPoints){
        usedPoints++;
        points[usedPoints].wrappedAngle = pose->angleFrom;
        return true;
    } else {
        return false;
    }
}

void Path::writeOut(){
    Serial.println("id\tspeed\tisOffLine\tangle");
    for (int i = 0; i < usedPoints; ++i)
    {
        Serial.print(i);
        Serial.print("\t");
        Serial.print(points[i].getSpeed(), BIN);
        Serial.print("\t");
        Serial.print(points[i].getOffLine(), BIN);
        Serial.print("\t");
        Serial.print(points[i].wrappedAngle);
        Serial.println();
    }
}
