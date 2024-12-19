#ifndef MPU6050_DMP_H
#define MPU6050_DMP_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

#define INTERRUPT_PIN 2  // Pin for MPU6050 interrupt
#define LED_PIN 13       // Pin for LED indicator

class MPU6050_DMP {
public:
    MPU6050_DMP();                    // Constructor
    bool initialize();                 // Initializes MPU6050 and DMP
    bool testConnection();             // Tests MPU6050 connection
    bool dmpReady = false;             // Flag for DMP readiness
    void update();                     // Reads and updates data from FIFO
    float getYaw();                    // Returns current yaw
    float getPitch();                  // Returns current pitch
    float getRoll();                   // Returns current roll
    void setInterruptPin(int pin);     // Sets custom interrupt pin if needed

private:
    MPU6050 mpu;
    uint8_t mpuIntStatus;              // MPU interrupt status byte
    uint8_t devStatus;                 // DMP initialization status
    uint16_t packetSize;               // DMP packet size
    uint8_t fifoBuffer[64];            // FIFO storage buffer
    Quaternion q;                      // Quaternion data
    VectorFloat gravity;               // Gravity vector
    float ypr[3];                      // Yaw, pitch, roll angles

    static void dmpDataReady();        // Interrupt callback for DMP data ready
    void calibrateOffsets();           // Calibrates sensor offsets
};

#endif
