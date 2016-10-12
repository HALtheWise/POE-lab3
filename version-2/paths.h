class PathPoint
{
private:
	// flags stores a packed representation of other information
	// included in a PathPoint. The LSB represents whether the robot
	// is expected to deviate from the line there, and the next three
	// bits represent robot speed 0...7
	byte flags;
public:
	byte wrappedAngle;

	PathPoint();
	bool getOffLine( void );
	void setOffLine( bool offline );
	byte getSpeed( void );
	void setSpeed( byte speed );
};

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



class Path
{
public:
	Path();

	PathPoint points[100];
	const int allocatedPoints = 100;
	int usedPoints = 0;

	bool useLeftSensor = false;

	void writeOut( void );
	bool attemptUpdate( Pose *pose );
	PathPoint *getPoint( double distAlong );
};

Path::Path(){
	for (int i = 0; i < allocatedPoints; ++i)
	{
		points[i] = PathPoint();
	}
}

PathPoint *Path::getPoint( double distAlong ){
	int index = min(distAlong, usedPoints - 1);
	if (index < 0)
	{
		return NULL;
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