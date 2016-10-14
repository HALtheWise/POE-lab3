#include "arduino.h"
#include <QTime>
#include <QThread>

_Serial Serial; // global Serial object to use everywhere else

long long millis(){
    return QTime(0,0,0).msecsTo(QTime::currentTime());
    // TODO : account for simulation acceleration ... somehow.
}

void delay(int duration){
    //QThread::sleep(duration);
}


//duplicate information
const byte leftSensorPin  = A1;
const byte rightSensorPin = A0;


int analogRead(int pin){
    if(!robot){
        return -1;
    }else{
        if(pin == leftSensorPin){
            return map(robot->ir_val_l,0.0,1.0,800,300);
        }else{
            return map(robot->ir_val_r,0.0,1.0,800,300);
            //return robot->ir_val_r;
        }

    }
}
