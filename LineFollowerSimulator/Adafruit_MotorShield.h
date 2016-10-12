#ifndef ADAFRUIT_MOTORSHIELD_H
#define ADAFRUIT_MOTORSHIELD_H

#include "utils.h"

#include "arduino.h"
#include "robot.h"

#define FORWARD 1

#define BACKWARD 2

class Adafruit_DCMotor{
private:
    int pin;
    int speed;
public:
    Adafruit_DCMotor(int pin);
    void setSpeed(int);
    void run(byte&);
};

class Adafruit_MotorShield
{
private:
    Adafruit_DCMotor *motor_left, *motor_right;
public:
    Adafruit_MotorShield();
    ~Adafruit_MotorShield();
    void begin();
    Adafruit_DCMotor* getMotor(int);
};

#endif // ADAFRUIT_MOTORSHIELD_H
