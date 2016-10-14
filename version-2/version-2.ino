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

// Import of PID library for training run, see libraries.md for download instructions
#include <PID_v1.h>

// Setup state machine for robot 
byte state = 1;
const byte STATE_STOP = 0;
const byte STATE_MEMORIZE = 1;
const byte STATE_REPLAY = 2;

// Include path management code
#include "odometry.h"
#include "paths.h"

// Controlling constants

const int LOOP_DURATION = 10; //(ms) This is the inverse of the main loop frequency

const int FORWARD_POWER_INITIAL = 30; // 0...255
const int TURN_POWER_INITIAL = 30; // 0...255

const float OUTER_TURN_LIMIT = 0.05;

const int POWER_REPLAY = 40; // 0...255

const double PATH_STEERING_RATE = .20; // Measured in fraction / degree, path-based replay steering constant.

const byte SMOOTHING_LENGTH = 4;

const double LINE_ANGLE_ADJUSTMENT_RATE_AWAY = 300.0/1000; // Measured in degrees per ms, maximum line-based odometry adjustment factor right.
const double LINE_ANGLE_ADJUSTMENT_RATE_TOWARD = 150.0/1000; // Measured in degrees per ms, maximum line-based odometry adjustment factor right.

const int MIN_SENSOR_LEFT 	= 360;
const int MAX_SENSOR_LEFT 	= 780;
const int MIN_SENSOR_RIGHT	= 600;
const int MAX_SENSOR_RIGHT 	= 880;

// Pin setup (must match hardware)
const byte leftSensorPin  = A0;
const byte rightSensorPin = A1;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor  = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

// Global variable setup (things that change each loop)
long lastActionTime;


int leftPower = 0, rightPower = 0; // range -255...255

// Declare and allocate robot pose and paths

Pose robotPose;

Path path1(200, false);
Path path2(100, true);
Path path3(600, false);
Path path4(150, true);
Path path5(200, false);
Path path6(100, true);
Path path7(100, false);

Path *paths[] = {&path1, &path2, &path3, &path4, &path5, &path6, &path7};
const byte numPaths = 7;
byte currentPathId = 0;

Path *currentPath = paths[currentPathId];

// Setup PID controller
double PIDerror=0, PIDsetpoint=0, PIDoutput;
double kp=1,ki=0,kd=0;

PID pid(&PIDerror, &PIDoutput, &PIDsetpoint, kp, ki, kd, DIRECT);

void setup()
{
	AFMS.begin();

	// Wait one second before running to allow the user to get their hand
	// out of the robot.
	stop();
	delay(1000);

	// Note the high baud rate to allow Serial.print() statements to run non-blocking within 10ms
	Serial.begin(57600);

	lastActionTime = millis();

	// Initialize PID controller parameters
	pid.SetMode(AUTOMATIC);
	pid.SetSampleTime(LOOP_DURATION - 2);
	pid.SetOutputLimits(-1, 1);

	// Reset robot pose for beginning of training run
	robotPose.reset();
}

// Tracking variables for IR averaging to allow maximally smooth data collection
// As many readings as possible over <10ms are averaged before any other processing is done
long totalLeft = 0;
long totalRight = 0;
int count = 0;

// Used to moderate debugging printouts with modulo.
int loopCount = 0;

void loop()
{
	// Check for newly-set PID parameters on serial
	handleIncomingSerial();

	// Update tracking variables of IR sensor
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

		// Update the estimated position of the robot based on currently commanded powers
		// In the future, this could be stateful and involve other sensors.
		robotPose.odometryUpdate(leftPower, rightPower, dt);


		// Main state machine
		if(state == STATE_MEMORIZE){
			//Serial.println(lineOffset(leftAvg, rightAvg));
			memorizeLine(leftAvg, rightAvg);
			
			if(state == STATE_REPLAY){
				// The memorizeLine function decided to change to STATE_REPLAY
				// Transition from STATE_MEMORIZE to STATE_REPLAY

				Serial.println("finished course, replaying.");

				stop();

				for(int i=0; i<numPaths; i++){
					paths[i]->smooth(SMOOTHING_LENGTH);

					Serial.println(i);
					paths[i]->writeOut();
				}

				// This is blocking code, but the Serial printing above is inherently blocking,
				// so there is nothing we can do to prevent it. This adds an aesthetically nice pause,
				// and makes any bugs caused by blocking behavior more obvious so they can be fixed.
				delay(3000);

				// Reset to the first path
				currentPathId = 0;
				currentPath = paths[0];

				// Reset the estimated robot position
				robotPose.reset();
			}
			
		}else if(state == STATE_REPLAY){

			replayLine(leftAvg, rightAvg, dt);
			
			if(loopCount % 50 == 0){
				writePoseSerial();
			}

		}else{
			stop();
		}

		// Functions above write requested powers to global variables, 
		// driveMotors() reads, processes, and outputs those.
		driveMotors();


		// Check to see wheter the loop is running as fast as desired, and throw warnings if it isn't.
		// This code intentionally refuses to run every 10th loop to prevent feeding badly where
		// printing the warning causes excessive loop time in a self-perpetuating loop.
		if(dt > LOOP_DURATION * 2 && loopCount % 10){
		    Serial.print("WARNING: Main loop running too slow: ");
		    Serial.println(dt);
		}

		// Reset sensor tracking variables
		totalRight = totalLeft = 0;
		count = 0;
		loopCount++;

		// This formulation attempts to ensure average loop duration is LOOP_DURATION,
		// without causing hyperactive behavior if the code is running slower than expected.

		// In particular, this won't drift at all unless a loop takes more than 10ms, 
		// and won't ever run multiple loops in a row with less than 10ms spacing.
		lastActionTime = lastActionTime + LOOP_DURATION*int(dt / LOOP_DURATION);

		// If something messed up such that this loop took ridiculously long, prevent
		// massive values of dt next time through the loop.
		// This happens during state transitions sometimes, especially if lots of serial data is being printed
		if (millis() - lastActionTime > 500) {
			lastActionTime = millis();
		}
	}
}

// Using a basic clipped PID controller, memorizes the shape of the course.
void memorizeLine(float leftAvg, float rightAvg)
{
	// Whether the current path curves right or left (preconfigured)
	bool useLeftSensor = currentPath->useLeftSensor;
	
	// Step 1: compute the error from a simple PID line follower
	float error = lineOffset(leftAvg, rightAvg, useLeftSensor);
	PIDerror = error;

	pid.Compute();

	// whether the robot will turn right or left (positive is right)
	float turnFactor = PIDoutput;

	if (!useLeftSensor){
		turnFactor *= -1;
	}

	// Step 2: constrain that error to make sure the robot doesn't turn (much) toward the line
	if(useLeftSensor){
	    turnFactor = min(turnFactor, OUTER_TURN_LIMIT);
	} else {
	    turnFactor = max(turnFactor, -OUTER_TURN_LIMIT);
	}

	leftPower	= FORWARD_POWER_INITIAL + turnFactor * TURN_POWER_INITIAL;
	rightPower	= FORWARD_POWER_INITIAL - turnFactor * TURN_POWER_INITIAL;


	// Step 3: Record the current motion into the path.
	currentPath->attemptUpdate( &robotPose );


	// Step 4: determine whether the robot has ended the current segment

	float offReading = lineOffset(leftAvg, rightAvg, !useLeftSensor);

	// Note guards to prevent segments from finishing immediately (less then 5cm)
	// or outlasting their allocated memory.
	if ((offReading > 0.5 && robotPose.distAlong > 5) || robotPose.distAlong > currentPath->allocatedPoints){
		// The robot's off-line sensor has seen a line
		Serial.println("segment end detected");


		stop();
		// delay(500);
		robotPose.reset();

		if(currentPathId < numPaths - 1){
		    currentPathId++;
		    currentPath = paths[currentPathId];
		}else{
			// trigger state transition
			state = STATE_REPLAY;
		}

	}

	// Print debug information
	if(loopCount % 50 == 0){
		writePoseSerial();
	}
}

// Sets motor values for replaying a recorded path with input from sensor readings leftAvg and rightAvg
void replayLine(float leftAvg, float rightAvg, int dt) {

	// Step 0: Handle exception for control parameters on the 5th segment (id=4)
	double awayAdjustAmount = LINE_ANGLE_ADJUSTMENT_RATE_AWAY;
	if(currentPathId == 4){
	    awayAdjustAmount *= 1.8;
	}

	bool useLeftSensor = currentPath->useLeftSensor;

	// Step 1: adjust current odometry estimate on the basis of sensor readings.
	// Note that this is clipped such that it will adjust dramatically to avoid
	// a line and gradually to find it again.
	// This prevents errors caused by the asymmetry of one-sensor following.

	double lineError = lineOffset(leftAvg, rightAvg, useLeftSensor);

	double lineCorrection = max(awayAdjustAmount, LINE_ANGLE_ADJUSTMENT_RATE_TOWARD)
							 * dt * lineError;

	lineCorrection = constrain(lineCorrection, -LINE_ANGLE_ADJUSTMENT_RATE_TOWARD, awayAdjustAmount);

	robotPose.angleFrom += lineCorrection;


	// Step 2: Drive the robot to follow the recorded path

	PathPoint *target = currentPath->getPoint(robotPose.distAlong);

	// error is positive if the path is left of the robot
	double pathError = target->wrappedAngle - robotPose.angleFrom;

	// whether the robot will turn right or left (positive is right)
	double turnFactor = -pathError * PATH_STEERING_RATE;

	leftPower	= POWER_REPLAY * (1 + turnFactor);
	rightPower	= POWER_REPLAY * (1 - turnFactor);

	// Step 3: Determine whether this segment of path is finished

	float offReading = lineOffset(leftAvg, rightAvg, !useLeftSensor);

	if (offReading > 0.5 && robotPose.distAlong > currentPath->usedPoints*0.7){
		// The robot's off-line sensor has seen a line
		Serial.println("segment end detected");

		stop();
		// delay(200);
		robotPose.reset();

		if(currentPathId < numPaths - 1){
		    currentPathId++;
		    currentPath = paths[currentPathId];
		}else{
			state = STATE_STOP;
		}

	}

	// Print debug information
	if(loopCount % 50 == 0){
	    Serial.print("pathError = ");
	    Serial.print(pathError);
	    Serial.print("\t:\t");
	    Serial.println(turnFactor);
	}
}

// normalizePowers ensures that 
// abs(*left) < limit and abs(*right) < limit
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
// Note that the output is not strictly gaurunteed to be in this range.
float lineOffset(float leftAvg, float rightAvg, bool useLeftSensor)
{
	if(useLeftSensor){
		return map(leftAvg, MIN_SENSOR_LEFT, MAX_SENSOR_LEFT, -100, 100) / 100.0;
	} else {
		return map(rightAvg, MIN_SENSOR_RIGHT, MAX_SENSOR_RIGHT, -100, 100) / 100.0;
	}
}

// If there is any data in the serial buffer, attempts to read in that data as new P, I, and D values.
void handleIncomingSerial()
{
	if(Serial.available() > 0){

		Serial.setTimeout(100);
		// Read first number from serial stream
		kp = Serial.parseFloat();
		Serial.read();
		// Read second number from serial stream
		ki = Serial.parseFloat();
		Serial.read();
		// Read third number form serial stream
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

	normalizePowers(&leftPower, &rightPower, 255);

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