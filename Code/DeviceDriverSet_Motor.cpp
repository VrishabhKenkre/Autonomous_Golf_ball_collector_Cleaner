#include "DeviceDriverSet_Motor.h"

/* Motor Control Initialization */
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Init(void) {
    // Motor 1 (Left Front)
    pinMode(R_PWM_1, OUTPUT);
    pinMode(L_PWM_1, OUTPUT);
    pinMode(R_EN_1, OUTPUT);
    pinMode(L_EN_1, OUTPUT);
    digitalWrite(R_EN_1, HIGH);
    digitalWrite(L_EN_1, HIGH);

    // Motor 2 (Right Front)
    pinMode(R_PWM_2, OUTPUT);
    pinMode(L_PWM_2, OUTPUT);
    pinMode(R_EN_2, OUTPUT);
    pinMode(L_EN_2, OUTPUT);
    digitalWrite(R_EN_2, HIGH);
    digitalWrite(L_EN_2, HIGH);

    // Motor 3 (Left Rear)
    pinMode(R_PWM_3, OUTPUT);
    pinMode(L_PWM_3, OUTPUT);
    pinMode(R_EN_3, OUTPUT);
    pinMode(L_EN_3, OUTPUT);
    digitalWrite(R_EN_3, HIGH);
    digitalWrite(L_EN_3, HIGH);

    // Motor 4 (Right Rear)
    pinMode(R_PWM_4, OUTPUT);
    pinMode(L_PWM_4, OUTPUT);
    pinMode(R_EN_4, OUTPUT);
    pinMode(L_EN_4, OUTPUT);
    digitalWrite(R_EN_4, HIGH);
    digitalWrite(L_EN_4, HIGH);
}

/* Motor Control Function */
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_control(bool direction_A, uint8_t speed_A,  // Motor A (Left front) parameters
                                                          bool direction_B, uint8_t speed_B,  // Motor B (Left front) parameters
                                                          bool direction_C, uint8_t speed_C,  // Motor C (Left back) parameters
                                                          bool direction_D, uint8_t speed_D,  // Motor D (Right back) parameters
                                                          bool controlED) {
    if (controlED == control_enable) {
        // Motor 1 (Left Front)
        if (direction_A == direction_just) {
            analogWrite(R_PWM_1, 0);
            analogWrite(L_PWM_1, speed_A); // Stop reverse direction PWM
        } else if (direction_A == direction_back) {
            analogWrite(R_PWM_1, speed_A); // Stop forward direction PWM
            analogWrite(L_PWM_1, 0);
        } else if (direction_A == direction_void) {
            analogWrite(R_PWM_1, 0);
            analogWrite(L_PWM_1, 0); // Stop all PWM outputs
        }

        // Motor 2 (Right Front)
        if (direction_B == direction_just) {
            analogWrite(R_PWM_2, speed_B);
            analogWrite(L_PWM_2, 0); // Stop reverse direction PWM
        } else if (direction_B == direction_back) {
            analogWrite(R_PWM_2, 0); // Stop forward direction PWM
            analogWrite(L_PWM_2, speed_B);
        } else if (direction_B == direction_void) {
            analogWrite(R_PWM_2, 0);
            analogWrite(L_PWM_2, 0); // Stop all PWM outputs
        }

        // Motor 3 (Left Back)
        if (direction_C == direction_just) {
            analogWrite(R_PWM_3, 0);
            analogWrite(L_PWM_3, speed_C); // Stop reverse direction PWM
        } else if (direction_C == direction_back) {
            analogWrite(R_PWM_3, speed_C); // Stop forward direction PWM
            analogWrite(L_PWM_3, 0);
        } else if (direction_C == direction_void) {
            analogWrite(R_PWM_3, 0);
            analogWrite(L_PWM_3, 0); // Stop all PWM outputs
        }

        // Motor 4 (Right Back)
        if (direction_D == direction_just) {
            analogWrite(R_PWM_4, speed_D);
            analogWrite(L_PWM_4, 0); // Stop reverse direction PWM
        } else if (direction_D == direction_back) {
            analogWrite(R_PWM_4, 0); // Stop forward direction PWM
            analogWrite(L_PWM_4, speed_D);
        } else if (direction_D == direction_void) {
            analogWrite(R_PWM_4, 0);
            analogWrite(L_PWM_4, 0); // Stop all PWM outputs
        }
    } else {
        // Disable control, stop all PWM outputs
        analogWrite(R_PWM_1, 0);
        analogWrite(L_PWM_1, 0);
        analogWrite(R_PWM_2, 0);
        analogWrite(L_PWM_2, 0);
        analogWrite(R_PWM_3, 0);
        analogWrite(L_PWM_3, 0);
        analogWrite(R_PWM_4, 0);
        analogWrite(L_PWM_4, 0);
    }
}


