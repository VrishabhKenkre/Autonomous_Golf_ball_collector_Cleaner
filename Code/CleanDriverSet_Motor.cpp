#include "CleanDriverSet_Motor.h"

// Constructor: Set pin modes and initialize motor to stop
CleanDriverSet_Motor::CleanDriverSet_Motor() {
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    // Initial state: Turn off the motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
}

// Start the motor: Forward rotation at maximum speed (250)
void CleanDriverSet_Motor::start() {
    analogWrite(enA, Speed);  // Set motor speed
    digitalWrite(in1, LOW);     // Set forward direction
    digitalWrite(in2, HIGH);
}

// Stop the motor: Turn off PWM and motor direction pins
void CleanDriverSet_Motor::stop() {
    analogWrite(enA, 0);         // Stop PWM output
    digitalWrite(in1, LOW);      // Set pins to stop the motor
    digitalWrite(in2, LOW);
}
