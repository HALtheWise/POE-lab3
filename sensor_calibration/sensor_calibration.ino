/* 
*	Sensor calibration code for Lab 3 of POE Fall 2016 as taught at Olin College
*	This code averages sensor readings over a period of 200ms, then communicates that
*		data out on the serial port with format "leftSensorAvg,rightSensorAvg\n"
*
*	Authors: Eric Miller (eric@legoaces.org) and Jamie Cho (yoonyoung.cho@students.olin.edu)
*/

#include <stdlib.h>

const int averagingDuration = 200;

// Pin setup (must match hardware)
const byte leftSensorPin  = A0;
const byte rightSensorPin = A1;
// const byte leftMotorPin = 9;
// const byte rightMotorPin = 10;

long lastActionTime;

void setup()
{
	Serial.begin(9600);

	lastActionTime = millis();

	// Initialize pins
	// Note that the analog sensors don't need initialization
}

long totalLeft = 0;
long totalRight = 0;
int count = 0;

void loop()
{
	int leftRead = 0;
	int rightRead = 0;
	getMeasurements(&leftRead, &rightRead);

	totalLeft += leftRead;
	totalRight += rightRead;
	count++;

	// Every (configurable) milliseconds, average together the readings recieved and transmit them
	long time = millis();
	if (time - lastActionTime > averagingDuration){
		float leftAvg = float(totalLeft) / count;
		float rightAvg = float(totalRight) / count;

		writeSerial(leftAvg, rightAvg);

		lastActionTime = time;
		totalRight = totalLeft = 0;
		count = 0;

	}
}

void writeSerial(float leftAvg, float rightAvg)
{
	Serial.print(leftAvg);
	Serial.print(",\t");
	Serial.print(rightAvg);
	Serial.print("\n");
}

void getMeasurements(int *leftRead, int *rightRead)
{
	*leftRead = analogRead(leftSensorPin);
	*rightRead = analogRead(rightSensorPin);
}