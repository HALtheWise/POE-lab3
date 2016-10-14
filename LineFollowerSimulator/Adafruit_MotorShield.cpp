#include "Adafruit_MotorShield.h"

// function definitions for the emulation of the Adafruit Motor Shield V2
// just to have it interface with the "virtual arduino"

Adafruit_DCMotor::Adafruit_DCMotor(int pin):pin(pin){

}

void Adafruit_DCMotor::run(byte & direction){
    float dir;
    if(direction == FORWARD){
        dir = 1;
    }else if (direction == BACKWARD){
        dir = -1;
    }

    if(robot){
        switch(pin){
        case 1: //left
            robot->setPowerL(dir * speed);
            break;
        case 2:
            robot->setPowerR(dir * speed);
            break;
        }
    }

}

void Adafruit_DCMotor::setSpeed(int speed){
    this->speed = speed;
}

Adafruit_MotorShield::Adafruit_MotorShield()
{
    this->motor_left = new Adafruit_DCMotor(1);
    this->motor_right = new Adafruit_DCMotor(2);
}

Adafruit_MotorShield::~Adafruit_MotorShield(){
    if(motor_left){
        delete motor_left;
        motor_left = nullptr;
    }
    if(motor_right){
        delete motor_right;
        motor_right = nullptr;
    }
}

void Adafruit_MotorShield::begin(){
    // don't need to do anything here
}

Adafruit_DCMotor* Adafruit_MotorShield::getMotor(int id){
    switch(id){
    case 1:
        return this->motor_left;
        break;
    case 2:
        return this->motor_right;
        break;
    default:
        return nullptr;
    }
}
