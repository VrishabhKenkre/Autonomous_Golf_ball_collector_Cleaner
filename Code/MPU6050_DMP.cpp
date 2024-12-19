#include "MPU6050_DMP.h"

volatile bool mpuInterrupt = false;  // Interrupt flag

// Constructor
MPU6050_DMP::MPU6050_DMP() {
    mpu = MPU6050();
}

// Initialize MPU6050 and DMP
bool MPU6050_DMP::initialize() {
    Wire.begin();
    Serial.begin(115200);

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        return false;
    }

    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        calibrateOffsets();  // Generate offsets and calibrate

        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        return true;
    } else {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
        return false;
    }
}

// Connection test
bool MPU6050_DMP::testConnection() {
    return mpu.testConnection();
}

// Calibrate sensor offsets
void MPU6050_DMP::calibrateOffsets() {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
}

// Update Yaw, Pitch, Roll values
void MPU6050_DMP::update() {
    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
}

// Get Yaw value
float MPU6050_DMP::getYaw() {
    return ypr[0] * 180 / M_PI;  // Convert to degrees
}

// Get Pitch value
float MPU6050_DMP::getPitch() {
    return ypr[1] * 180 / M_PI;  // Convert to degrees
}

// Get Roll value
float MPU6050_DMP::getRoll() {
    return ypr[2] * 180 / M_PI;  // Convert to degrees
}

// Set custom interrupt pin
void MPU6050_DMP::setInterruptPin(int pin) {
    pinMode(pin, INPUT);
}

// Static interrupt callback function
void MPU6050_DMP::dmpDataReady() {
    mpuInterrupt = true;
}
