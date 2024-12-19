#ifndef _DeviceDriverSet_Motor_H_
#define _DeviceDriverSet_Motor_H_

#include <Arduino.h>

/* Motor Driver */
class DeviceDriverSet_Motor {
public:
    static const int direction_void = 2; // Direction identifier for stop state
    void DeviceDriverSet_Motor_Init(void); // Initialize motor pins
    void DeviceDriverSet_Motor_control(bool direction_A, uint8_t speed_A,  // Motor A (Left front) parameters
                                       bool direction_B, uint8_t speed_B,  // Motor B (Right front) parameters
                                       bool direction_C, uint8_t speed_C,  // Motor C (Left back) parameters
                                       bool direction_D, uint8_t speed_D,  // Motor D (Right back) parameters
                                       bool controlED);                   // Enable control
private:
    // Motor 1 (Left Front) Pin Setup Correct
    const int R_PWM_1 = 11;   // Mega PWM Pin 4
    const int L_PWM_1 = 10;   // Mega PWM Pin 5
    const int R_EN_1 = 53;   // Mega Digital Pin 46
    const int L_EN_1 = 52;   // Mega Digital Pin 47

    // Motor 2 (Right Front) Pin Setup Correct
    const int R_PWM_2 = 6;   // Mega PWM Pin 7
    const int L_PWM_2 = 7;   // Mega PWM Pin 6
    const int R_EN_2 = 47;   // Mega Digital Pin 48
    const int L_EN_2 = 46;   // Mega Digital Pin 49

    // Motor 3 Pin Setup (Left Back) (Not using when only using 2 motor driver)
    const int R_PWM_3 = 9; //Mega PWM Pin 8
    const int L_PWM_3 = 8; //Mega PWM Pin 9
    const int R_EN_3 = 51; //Mega Digital Pin 50
    const int L_EN_3 = 50; //Mega Digital Pin 51

    // Motor 4 Pin Setup (Right Back) (Not using when only using 2 motor driver)
    // Pin set need to be opposite from left motor setup
    const int R_PWM_4 = 5; //Mega PWM Pin 11
    const int L_PWM_4 = 4; //Mega PWM Pin 10
    const int R_EN_4 = 49; //Mega Digital Pin 52
    const int L_EN_4 = 48; //Mega Digital Pin 53
        

public:
    static const uint8_t speed_Max = 250; // Maximum speed value
    static const bool direction_just = true;  // Forward direction
    static const bool direction_back = false; // Reverse direction
    static const bool control_enable = true;  // Enable control
    static const bool control_disable = false; // Disable control
};

#endif

