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

// Controlling constants

const int LOOP_DURATION = 10; //(ms) This is the inverse of the main loop frequency

const int FORWARD_POWER = 100; // 0...255
const int TURN_POWER = 100; // 0...255


// Pin setup (must match hardware)
const byte leftSensorPin  = A0;
const byte rightSensorPin = A1;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
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
	if (time - lastActionTime > LOOP_DURATION) {
		float leftAvg = float(totalLeft) / count;
		float rightAvg = float(totalRight) / count;


		lineFollowBang(leftAvg, rightAvg);		

		// Reset counting variables
		lastActionTime = time;
		totalRight = totalLeft = 0;
		count = 0;

	}
}

// Implements non-blocking bang-bang control of motors.
void lineFollowBang(float leftAvg, float rightAvg)
{
	float error = lineOffset(leftAvg, rightAvg);

	// whether the robot will turn right or left (positive is right)
	// Notice negation of error value for negative feedback behavior
	float turnFactor = error > 0 ? -1 : 1;

	int leftPower	= FORWARD_POWER + turnFactor * TURN_POWER;
	int rightPower	= FORWARD_POWER - turnFactor * TURN_POWER;

	driveMotors(leftPower, rightPower);
}

// Returns the estimated offset of the robot from the line, with 
// positive values indicating that the robot is right of the line
// and negative values indicating the robot is left of the line.
// Inputs are raw readings from left and right sensors, 0...1023
//
// Currently the returned value is dimensionless, and
// represents only the data derivable from the immediate sensor
// readings.
float lineOffset(float leftAvg, float rightAvg)
{
	return rightAvg - leftAvg;
}

// void writeSerial(float leftAvg, float rightAvg)
// {
// 	Serial.print(leftAvg);
// 	Serial.print(",");
// 	Serial.print(rightAvg);
// 	Serial.print("\n");
// }

void driveMotors(int leftPower, int rightPower){
	// Inputs leftPower and rightPower vary from -255...255
	// Code in this function is based on https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/using-dc-motors

	// For each motor, decide whether to run it FORWARD, BACKWARD, (or RELEASE)
	// These are ternary operators, returning FORWARD if power > 0 and backward otherwise.
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