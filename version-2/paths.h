// PathPoint represents the data associated with each point
// along a recorded segment of path.
// Currently this is just an angle because the speed adjustment flags
// needed to be removed before they could be fully implemented.
class PathPoint
{
public:
	double wrappedAngle;

	PathPoint();
};

PathPoint::PathPoint( void ){
	wrappedAngle = 0;
}

// A Path is a collection of PathPoints that represents a segment of the course.
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

// Note that the points array is dynamically allocated to save space.
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

// Updates the path with new information if the information is not currently contained in the path.
bool Path::attemptUpdate( Pose *pose ) {
	if (int(pose->distAlong) > usedPoints && usedPoints < allocatedPoints){
		usedPoints++;
		points[usedPoints].wrappedAngle = pose->angleFrom;
		return true;
	} else {
		return false;
	}
}

// Prints Path data to serial for debugging and analysis.
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

// Runs a n-point leading average to enable both forward-looking controls predictions
// and to smooth out irregular behavior in the training follow.
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
