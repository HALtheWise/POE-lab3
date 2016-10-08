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
	const int numPoints = 100;

	void writeOut( void );
};

Path::Path(){
	for (int i = 0; i < numPoints; ++i)
	{
		points[i] = PathPoint();
	}
}

void Path::writeOut(){
	for (int i = 0; i < numPoints; ++i)
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