/* 
*	Version 1 Arduino code for Lab 3 of POE Fall 2016 as taught at Olin College
*	This code demonstrates reading the sensor values and controlling the motors
*	using a trivial bang-bang control scheme. 
*	This is not intended to be final code, but merely a proof of concept.
*
*	Authors: Eric Miller (eric@legoaces.org) and Jamie Cho (yoonyoung.cho@students.olin.edu)
*/

#include <stdlib.h>

// Imports for Motor Shield, taken from https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/using-dc-motors
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 


const int averagingDuration = 10; //This is the inverse of the main loop time

const byte leftSensorPin  = A0;
const byte rightSensorPin = A1;

Adafruit_DCMotor *leftMotor  = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);


long lastActionTime;

void setup()
{
	Serial.begin(9600);

	lastActionTime = millis();

	// Initialize pins
	// Note that the analog sensors don't need initialization
	AFMS.begin();

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
	Serial.print(",");
	Serial.print(rightAvg);
	Serial.print("\n");
}

void driveMotors(int leftPower, int rightPower){
	// Inputs leftPower and rightPower vary from -255...255
	// Code in this function is based on https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/using-dc-motors

	// For each motor, decide whether to run it FORWARD, BACKWARD, (or RELEASE)
	byte leftDirection	= (leftPower  > 0) ? FORWARD : BACKWARD;
	byte rightDirection	= (rightPower > 0) ? FORWARD : BACKWARD;
	
	// Set motor speeds
	leftMotor	->	setSpeed(abs(leftPower));
	rightMotor	->	setSpeed(abs(rightPower));

	// Set motor directions
	leftMotor	->	run(leftDirection);
	rightMotor	->	run(rightDirection);

}

void getMeasurements(int *leftRead, int *rightRead)
{
	*leftRead = analogRead(leftSensorPin);
	*rightRead = analogRead(rightSensorPin);
}