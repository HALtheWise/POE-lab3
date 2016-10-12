/* 
*	Version 2 Arduino code for Lab 3 of POE Fall 2016 as taught at Olin College
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
#include "odometry.h"
#include "paths.h"

// Controlling constants

const int LOOP_DURATION = 10; //(ms) This is the inverse of the main loop frequency

const int FORWARD_POWER = 25; // 0...255
const int TURN_POWER = 25; // 0...255

const int MIN_SENSOR_LEFT 	= 281;
const int MAX_SENSOR_LEFT 	= 804;
const int MIN_SENSOR_RIGHT	= 533;
const int MAX_SENSOR_RIGHT 	= 860;

const double FOLLOW_MULTIPLIER = 1.5;

// Pin setup (must match hardware)
const byte leftSensorPin  = A0;
const byte rightSensorPin = A1;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor  = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

// Global variable setup (things that change each loop)
long lastActionTime;

byte state = 1;
const byte STATE_STOP = 0;
const byte STATE_MEMORIZE = 1;
const byte STATE_REPLAY = 2;


int leftPower = 0, rightPower = 0; // range -255...255

// Declare and allocate robot pose and paths

Pose robotPose;

Path path1;
Path path2;
Path path3;

Path *paths[] = {&path1, &path2, &path3};
const byte numPaths = 3;
byte currentPathId = 0;

Path *currentPath = paths[currentPathId];

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
	pid.SetSampleTime(LOOP_DURATION - 2);
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

		robotPose.odometryUpdate(leftPower, rightPower, dt);

		if(state == STATE_MEMORIZE){
			//Serial.println(lineOffset(leftAvg, rightAvg));
			memorizeLine(leftAvg, rightAvg);
			
			if(state == STATE_REPLAY){
				// The memorizeLine function decided to change to STATE_REPLAY

				Serial.println("finished course, replaying.");

				stop();

				for(int i=0; i<numPaths; i++){
					Serial.println(i);
					paths[i]->writeOut();
				}

				delay(3000);

				currentPathId = 0;
				currentPath = paths[0];

				robotPose.reset();
			}
			
		}else if(state == STATE_REPLAY){

			replayLine(leftAvg, rightAvg);
			
			if(loopCount % 100 == 0){
				writePoseSerial();
			}

			if(robotPose.distAlong > 100){
				Serial.println("finished replay, stopping.");

			    state = STATE_STOP;
			}

		}else{
			stop();
		}

		driveMotors();


		// This code intentionally refuses to run every 10th loop to prevent feeding badly
		if(dt > LOOP_DURATION * 2 && loopCount % 10){
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

		// If something messed up such that this loop took ridiculously long, prevent
		// massive values of dt next time through the loop.
		// This happens during state transitions sometimes.
		if (millis() - lastActionTime > 500) {
			lastActionTime = millis();
		}
	}
}

// Implements non-blocking bang-bang control of motors.
void memorizeLine(float leftAvg, float rightAvg)
{
	bool useLeftSensor = currentPath->useLeftSensor;
	
	float error = lineOffset(leftAvg, rightAvg, useLeftSensor);
	PIDerror = error;

	pid.Compute();

	// whether the robot will turn right or left (positive is right)
	float turnFactor = PIDoutput;

	leftPower	= FORWARD_POWER + turnFactor * TURN_POWER;
	rightPower	= FORWARD_POWER - turnFactor * TURN_POWER;

	normalizePowers(&leftPower, &rightPower, 255);

	// Reading from sensor off of the line
	float offReading = lineOffset(leftAvg, rightAvg, !useLeftSensor);

	if (offReading > 0 || robotPose.distAlong > 200){
		// The robot's off-line sensor has seen a line
		stop();
		delay(1000);
		robotPose.reset();

		if(currentPathId < numPaths - 1){
		    currentPathId++;
		    currentPath = paths[currentPathId];
		}else{
			state = STATE_REPLAY;
		}

	}else{
		// The robot is still following normally
		currentPath->attemptUpdate( &robotPose );
	}


	if(loopCount % 100 == 0){
		writePoseSerial();
	}
}

void replayLine(float leftAvg, float rightAvg) {
	bool useLeftSensor = currentPath->useLeftSensor;

	PathPoint *target = currentPath->getPoint(robotPose.distAlong);

	// error is positive if the path is left of the robot
	// TODO: Handle intentional offsets from sensor feedback
	double pathError = byte(target->wrappedAngle - byte(robotPose.angleFrom));
	if(pathError > 127){
	    pathError = pathError - 256;
	}
	pathError /= 255;


	double lineError = lineOffset(leftAvg, rightAvg, useLeftSensor);
	PIDerror = lineError;

	pid.Compute();


	// whether the robot will turn right or left (positive is right)
	double turnFactor = -pathError * 30 + PIDoutput * 0.3;

	leftPower	= FORWARD_POWER + turnFactor * TURN_POWER;
	rightPower	= FORWARD_POWER - turnFactor * TURN_POWER;

	leftPower 	*= FOLLOW_MULTIPLIER;
	rightPower 	*= FOLLOW_MULTIPLIER;

	if(loopCount % 50 == 0){
	    Serial.print("pathError = ");
	    Serial.print(pathError);
	    Serial.print("\t:\t");
	    Serial.println(turnFactor);
	}

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
	    *left = (*left * limit) / maxabs;
	    *right = (*right * limit) / maxabs;
	}
}

// Returns how much the selected sensor is on the line, with
// -1 reflecting "completely off" and 1 meaning "completely  on"
float lineOffset(float leftAvg, float rightAvg, bool useLeftSensor)
{
	if(useLeftSensor){
		return map(leftAvg, MIN_SENSOR_LEFT, MAX_SENSOR_LEFT, -100, 100) / 100.0;
	} else {
		return map(rightAvg, MIN_SENSOR_RIGHT, MAX_SENSOR_RIGHT, -100, 100) / 100.0;
	}
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

void stop(){
	leftPower = 0;
	rightPower = 0;
	driveMotors();
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