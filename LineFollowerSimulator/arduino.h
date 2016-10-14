#ifndef ARDUINO_H
#define ARDUINO_H

// arduino.h essentially enables native arduino code to run seamlessly with the application.

#define BIN (2)

const int A0 = 0xA0;
const int A1 = 0xA1;

#include <iostream>
#include <bitset>

#include "utils.h"
#include "mainwindow.h"
#include "robot.h"

class _Serial{

public:
    void begin(int){
        // ignore baud rate
    }

    // all prints are redirected to stdout
    template<typename T>
    void print(T val){
        std::cout << val;
    }

    template<typename T>
    void print(T val, int flag){
        if(flag == BIN){
            std::cout << std::bitset<8>(val);
        }
        // ... not handling other flags yet
    }

    template<typename T>
    void println(T val){
        std::cout << val << std::endl;
    }

    void println(){
        std::cout << std::endl;
    }
};

extern _Serial Serial;
typedef unsigned char byte;

// Arduino APIs

extern long long millis();
extern int analogRead(const int pin);
extern void setup();
extern void loop();
extern void delay(int);

#endif // ARDUINO_H
