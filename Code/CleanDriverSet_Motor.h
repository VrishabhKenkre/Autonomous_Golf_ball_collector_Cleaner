#ifndef CLEANDRIVERSET_MOTOR_H
#define CLEANDRIVERSET_MOTOR_H

#include <Arduino.h>

class CleanDriverSet_Motor {
public:
    // Constructor: Initialize motor control pins
    CleanDriverSet_Motor();

    // Start the motor: Fixed forward rotation at speed 250
    void start();

    // Stop the motor
    void stop();

private:
    // Motor control pin definitions
    const int enA = 44; // PWM control pin
    const int in1 = 42; // Direction control pin 1
    const int in2 = 40; // Direction control pin 2

    const int Speed = 250; // Fixed maximum speed in PMW 
};

#endif
