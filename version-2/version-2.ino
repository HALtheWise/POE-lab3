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

#include <PID_v1.h>

// Include path management code
#include "paths.h"

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

// Global variable setup (things that change each loop)
long lastActionTime;

// Setup PID controller
double PIDerror=0, PIDsetpoint=0, PIDoutput;
double kp=1,ki=0,kd=0;

PID pid(&PIDerror, &PIDoutput, &PIDsetpoint, kp, ki, kd, DIRECT);

void setup()
{
	Serial.begin(9600);

	lastActionTime = millis();

	// Initialize pins
	// Note that the analog sensors don't need initialization
	AFMS.begin();

	pid.SetMode(AUTOMATIC);
	pid.SetSampleTime(LOOP_DURATION);
}

long totalLeft = 0;
long totalRight = 0;
int count = 0;

void loop()
{
	handleIncomingSerial();

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


		lineFollowPid(leftAvg, rightAvg);		

		// Reset counting variables
		totalRight = totalLeft = 0;
		count = 0;

		// This formulation attempts to ensure average loop duration is LOOP_DURATION,
		// without causing hyperactive behavior if something blocks for a while.
		lastActionTime = lastActionTime + LOOP_DURATION*int((time-lastActionTime) / LOOP_DURATION);
	}
}

// Implements non-blocking bang-bang control of motors.
void lineFollowPid(float leftAvg, float rightAvg)
{
	float error = lineOffset(leftAvg, rightAvg);
	PIDerror = error;

	pid.Compute();

	// whether the robot will turn right or left (positive is right)
	float turnFactor = PIDoutput;

	int leftPower	= FORWARD_POWER + turnFactor * TURN_POWER;
	int rightPower	= FORWARD_POWER - turnFactor * TURN_POWER;

	normalizePowers(&leftPower, &rightPower, 255);

	driveMotors(leftPower, rightPower);
}

// normalizePowers ensures that 
// abs(left) < limit and abs(right) < limit
// while maintaining their ratio.
// Useful for constraining desired speeds to be
// achievable by the motors.
void normalizePowers(int *left, int *right, int limit){
	int maxabs = max(abs(*left), abs(*right));
	if(maxabs > limit)
	{
	    *left = *left * (limit / maxabs);
	    *right = *right * (limit / maxabs);
	}
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

void handleIncomingSerial()
{
	if(Serial.available() > 0){

		Serial.setTimeout(100);
		// Read first number from serial stream
		kp = Serial.parseFloat();
		Serial.read();
		ki = Serial.parseFloat();
		Serial.read();
		kd = Serial.parseFloat();
		Serial.read();

		// Ingest remainder of serial buffer in case something went wrong
		while(Serial.available()){
		    Serial.read();
		}

		pid.SetTunings(kp, ki, kd);
		writeSerial();
	}
}

void writeSerial()
{
	Serial.println("Tunings set to (kp, ki, kd) = ");
	Serial.print("\t(");
	Serial.print(kp);
	Serial.print(", ");
	Serial.print(ki);
	Serial.print(", ");
	Serial.print(kd);
	Serial.print(")\n");
}

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