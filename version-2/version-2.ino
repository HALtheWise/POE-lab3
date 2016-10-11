/* 
*	Version 1 Arduino code for Lab 3 of POE Fall 2016 as taught at Olin College
*	This code attempts to follow the line while recording (and eventually being able to play back) a motion path.
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
#include "odometry.h"

// Controlling constants

const int LOOP_DURATION = 10; //(ms) This is the inverse of the main loop frequency

const int FORWARD_POWER = 20; // 0...255
const int TURN_POWER = 20; // 0...255

const int MIN_SENSOR = 920;
const int MAX_SENSOR = 960;


// Pin setup (must match hardware)
const byte leftSensorPin  = A1;
const byte rightSensorPin = A0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor  = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

// Global variable setup (things that change each loop)
long lastActionTime;

byte state = 1;
const byte STATE_STOP = 0;
const byte STATE_FOLLOWING = 1;
const byte STATE_REPLAY = 2;

int leftPower = 0, rightPower = 0; // range -255...255

Pose robotPose;

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
	pid.SetOutputLimits(-1, 1);
}

long totalLeft = 0;
long totalRight = 0;
int count = 0;

int loopCount = 0;

void loop()
{
	handleIncomingSerial();

	int leftRead = 0;
	int rightRead = 0;
	getMeasurements(&leftRead, &rightRead);

	totalLeft += leftRead;
	totalRight += rightRead;
	count++;

	// Every (configurable) milliseconds, average together the readings recieved and handle them
	int dt = millis() - lastActionTime;

	if (dt > LOOP_DURATION) {
		float leftAvg = float(totalLeft) / count;
		float rightAvg = float(totalRight) / count;

		if(state == STATE_FOLLOWING){
			//Serial.println(lineOffset(leftAvg, rightAvg));
			lineFollowPid(leftAvg, rightAvg);

			if(robotPose.distAlong > 100){
				Serial.println("finished course, replaying");
			    state = STATE_REPLAY;
			}
		}else{
			leftPower = 0;
			rightPower = 0;
		}

		driveMotors();

		robotPose.odometryUpdate(leftPower, rightPower, dt);

		if(loopCount % 100 == 0){
			writePoseSerial();
		}


		if(dt > LOOP_DURATION * 2){
		    Serial.print("WARNING: Main loop running too slow: ");
		    Serial.println(dt);
		}

		// Reset counting variables
		totalRight = totalLeft = 0;
		count = 0;
		loopCount++;

		// This formulation attempts to ensure average loop duration is LOOP_DURATION,
		// without causing hyperactive behavior if something blocks for a while.
		lastActionTime = lastActionTime + LOOP_DURATION*int(dt / LOOP_DURATION);
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

	leftPower	= FORWARD_POWER + turnFactor * TURN_POWER;
	rightPower	= FORWARD_POWER - turnFactor * TURN_POWER;

	normalizePowers(&leftPower, &rightPower, 255);
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
	return map(rightAvg, MIN_SENSOR, MAX_SENSOR, -100, 100) / 100.0;
	//return rightAvg - leftAvg;
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
		writeTuningsSerial();
	}
}

void writePoseSerial(){
	Serial.print("Pose is: \t");
	robotPose.writeOut();
	Serial.println();
}

void writeTuningsSerial()
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

void driveMotors(){
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