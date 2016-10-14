class PathPoint
{
public:
	double wrappedAngle;

	PathPoint();
};

PathPoint::PathPoint( void ){
	wrappedAngle = 0;
}

class Path
{
public:
	Path(int length, bool useLeft);

	PathPoint *points;
	int allocatedPoints; // = 0;
	int usedPoints; // = 0;

	bool useLeftSensor; // = false;

	void writeOut( void );
	bool attemptUpdate( Pose *pose );
	void smooth( byte smoothingLength );
	PathPoint *getPoint( double distAlong );
};

Path::Path(int length, bool useLeft){
	useLeftSensor = useLeft;

	allocatedPoints = length;

	points = (PathPoint*) malloc(sizeof(PathPoint) * length);

	for (int i = 0; i < allocatedPoints; ++i)
	{
		points[i] = PathPoint();
	}

	points[0].wrappedAngle = 0;
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
		Serial.print(points[i].wrappedAngle);
		Serial.println();
	}
}

void Path::smooth(byte smoothingLength){
	for (int i = 0; i < usedPoints - smoothingLength; ++i)
	{
		double total = 0;
		for (int j = 0; j < smoothingLength; ++j)
		{
			double point = points[i+j].wrappedAngle;

			total += point;
		}
		double average = total / smoothingLength;

		points[i].wrappedAngle = average;
	}
}
