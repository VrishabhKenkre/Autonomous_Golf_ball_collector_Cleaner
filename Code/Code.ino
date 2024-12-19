#include "DeviceDriverSet_Motor.h"
#include "MPU6050_DMP.h" 
#include "SparkFun_u-blox_GNSS_Arduino_Library.h" // GPS library

DeviceDriverSet_Motor motorDriver;
MPU6050_DMP mpuDMP;
SFE_UBLOX_GNSS myGNSS;

// Constants
const float EARTH_RADIUS = 6371000.0; // Earth's radius in meters
const float MAX_SPEED = 2.0;          // Maximum speed in meters/second
const float yawTolerance = 2.0;       // Tolerance range to control turning precision

// Variables
float Yaw = 0.0;
bool startPointCaptured = false;
float startLat = 0, startLon = 0;
float midLat = 0, midLon = 0;
float finalLat = 0, finalLon = 0;

// Function to calculate yaw to target GPS point
float calculateYawToPoint(float startLat, float startLon, float endLat, float endLon) {
    float deltaX = endLon - startLon;
    float deltaY = endLat - startLat;
    return atan2(deltaY, deltaX) * 180.0 / M_PI; // Convert radians to degrees
}

// Function to calculate distance between two GPS points
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);
    float a = sin(dLat / 2) * sin(dLat / 2) +
              cos(radians(lat1)) * cos(radians(lat2)) *
              sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS * c; // Distance in meters
}

void moveForward(int base_speed, float duration) {
    const int LowerLimit = 30; // Maximum speed limit
    const int UpperLimit = 230; // Maximum speed limit

    // PID controller parameters setup & initialize
    unsigned long lastTime = millis();
    double PsiError = 0.0;
    double LastPsiError = 0.0;
    double dPsiError = 0.0;
    double dt = 0.0;

    const double Kp = 10; // P controller
    const double Kd = 0.05; // D controller

    mpuDMP.update();
    double Yaw_Desired = mpuDMP.getYaw(); // Get forward initial direction

    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
        mpuDMP.update();
        Yaw = mpuDMP.getYaw();

        // PID calculations
        unsigned long now = millis();
        dt = (double)(now - lastTime) / 1000.0; // Convert to seconds
        PsiError = Yaw_Desired - Yaw;
        dPsiError = (PsiError - LastPsiError) / dt;

        // Normalize yaw error
        if (PsiError > 180.0) PsiError -= 360.0;
        if (PsiError < -180.0) PsiError += 360.0;

        double Output = Kp * PsiError + Kd * dPsiError;

        // Adjust motor speeds
        float L_speed = base_speed + Output;
        float R_speed = base_speed - Output;
        L_speed = constrain(L_speed, LowerLimit, UpperLimit);
        R_speed = constrain(R_speed, LowerLimit, UpperLimit);

        motorDriver.DeviceDriverSet_Motor_control(DeviceDriverSet_Motor::direction_just, L_speed,
                                                  DeviceDriverSet_Motor::direction_just, R_speed,
                                                  DeviceDriverSet_Motor::direction_just, L_speed,
                                                  DeviceDriverSet_Motor::direction_just, R_speed,
                                                  DeviceDriverSet_Motor::control_enable);

        LastPsiError = PsiError;
        lastTime = now;

        Serial.print("TargetYaw: ");
        Serial.print(Yaw_Desired);
        Serial.print(" | CurrentYaw: ");
        Serial.print(Yaw);
        Serial.print(" | Yaw Error: ");
        Serial.print(PsiError);
        Serial.print(" | L_speed: ");
        Serial.print(L_speed);
        Serial.print(" | R_speed: ");
        Serial.println(R_speed);

        delay(50); // Avoid refreshing control signals too frequently
    }

    // Stop motors
    motorDriver.DeviceDriverSet_Motor_control(DeviceDriverSet_Motor::direction_void, 0,
                                              DeviceDriverSet_Motor::direction_void, 0,
                                              DeviceDriverSet_Motor::direction_void, 0,
                                              DeviceDriverSet_Motor::direction_void, 0,
                                              DeviceDriverSet_Motor::control_disable);

    Serial.println("Move Forward completed.");
}


void ArcTurn(float targetYaw, float Inner_speed, float Outter_speed) {
    float yawError;

    do {
        mpuDMP.update();
        Yaw = mpuDMP.getYaw();
        yawError = targetYaw - Yaw;

        if (yawError > 180.0) yawError -= 360.0;
        if (yawError < -180.0) yawError += 360.0;

        // Adjust speeds proportionally
        float scaleFactor = constrain(abs(yawError) / 90.0, 0.5, 1.0);
        float FO_Speed = Outter_speed * scaleFactor;
        float FI_Speed = Inner_speed * scaleFactor;

        if (yawError > yawTolerance) { // Turn Right
            motorDriver.DeviceDriverSet_Motor_control(DeviceDriverSet_Motor::direction_just, FO_Speed,
                                                      DeviceDriverSet_Motor::direction_just, FI_Speed,
                                                      DeviceDriverSet_Motor::direction_just, FO_Speed,
                                                      DeviceDriverSet_Motor::direction_just, FI_Speed,
                                                      DeviceDriverSet_Motor::control_enable);
        } else if (yawError < -yawTolerance) { // Turn Left
            motorDriver.DeviceDriverSet_Motor_control(DeviceDriverSet_Motor::direction_just, FI_Speed,
                                                      DeviceDriverSet_Motor::direction_just, FO_Speed,
                                                      DeviceDriverSet_Motor::direction_just, FI_Speed,
                                                      DeviceDriverSet_Motor::direction_just, FO_Speed,
                                                      DeviceDriverSet_Motor::control_enable);
        }

        Serial.print("Target Yaw: ");
        Serial.print(targetYaw);
        Serial.print(" | Current Yaw: ");
        Serial.print(Yaw);
        Serial.print(" | Yaw Error: ");
        Serial.println(yawError);

        delay(50);
    } while (abs(yawError) > yawTolerance);

    motorDriver.DeviceDriverSet_Motor_control(DeviceDriverSet_Motor::direction_void, 0,
                                              DeviceDriverSet_Motor::direction_void, 0,
                                              DeviceDriverSet_Motor::direction_void, 0,
                                              DeviceDriverSet_Motor::direction_void, 0,
                                              DeviceDriverSet_Motor::control_disable);

    Serial.println("Turn completed.");
}


// GPS-based forward movement
void moveForwardGPS(float baseSpeed, float targetLat, float targetLon) {
    float currentLat, currentLon;
    bool targetReached = false;

    while (!targetReached) {
        // Get current GPS and update yaw
        myGNSS.checkUblox();
        currentLat = myGNSS.getLatitude() / 10000000.0;
        currentLon = myGNSS.getLongitude() / 10000000.0;
        mpuDMP.update();
        Yaw = mpuDMP.getYaw();

        // Calculate distance and yaw error
        float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
        float desiredYaw = calculateYawToPoint(currentLat, currentLon, targetLat, targetLon);
        float yawError = desiredYaw - Yaw;

        // Normalize yaw error to [-180, 180]
        if (yawError > 180.0) yawError -= 360.0;
        if (yawError < -180.0) yawError += 360.0;

        // Check if the target is reached
        if (distance < 0.5) { // Within 0.5 meters
            targetReached = true;
            break;
        }

        // Calculate motor speeds
        float output = 10 * yawError; // Proportional control for yaw correction
        float L_speed = baseSpeed + output;
        float R_speed = baseSpeed - output;

        // Constrain motor speeds
        L_speed = constrain(L_speed, 30, 230);
        R_speed = constrain(R_speed, 30, 230);

        // Drive motors
        motorDriver.DeviceDriverSet_Motor_control(
            DeviceDriverSet_Motor::direction_just, L_speed,
            DeviceDriverSet_Motor::direction_just, R_speed,
            DeviceDriverSet_Motor::direction_just, L_speed,
            DeviceDriverSet_Motor::direction_just, R_speed,
            DeviceDriverSet_Motor::control_enable
        );

        // Debug info
        Serial.print("Current Lat: "); Serial.print(currentLat, 6);
        Serial.print(" | Current Lon: "); Serial.print(currentLon, 6);
        Serial.print(" | Distance: "); Serial.print(distance);
        Serial.print(" | Desired Yaw: "); Serial.print(desiredYaw);
        Serial.print(" | Current Yaw: "); Serial.println(Yaw);

        delay(50); // Control loop delay
    }

    // Stop motors
    motorDriver.DeviceDriverSet_Motor_control(
        DeviceDriverSet_Motor::direction_void, 0,
        DeviceDriverSet_Motor::direction_void, 0,
        DeviceDriverSet_Motor::direction_void, 0,
        DeviceDriverSet_Motor::direction_void, 0,
        DeviceDriverSet_Motor::control_disable
    );

    Serial.println("Target reached.");
}

// Cover Path Planning
void coverPathPlanning() {
    float Forward_RPM = 150;
    float Inner_RPM = 90;
    float Outter_RPM = 200;

    int loopcount = 3; // Adjust as needed
    for (int i = 0; i < loopcount; ++i) {
        moveForwardGPS(150, midLat, midLon); // Ensure the robot moves forward using GPS
        moveForward(Forward_RPM, 3000); // Fixed motion for path planning
        ArcTurn(90.0, Inner_RPM, Outter_RPM);
        moveForward(Forward_RPM, 700);
        ArcTurn(180.0, Inner_RPM, Outter_RPM);
        moveForward(Forward_RPM, 3000);
        ArcTurn(-90.0, Inner_RPM, Outter_RPM);
        ArcTurn(0.0, Inner_RPM, Outter_RPM);
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize Motor Driver
    motorDriver.DeviceDriverSet_Motor_Init();
    Serial.println("Motor driver initialized!");

    // Initialize IMU
    if (mpuDMP.initialize()) {
        Serial.println("MPU6050 initialized successfully!");
        mpuDMP.update();
        Yaw = mpuDMP.getYaw();
        Serial.print("Initial Yaw: ");
        Serial.println(Yaw);
    } else {
        Serial.println("MPU6050 initialization failed.");
        while (1);
    }

    // Initialize GPS
    if (!myGNSS.begin(Wire, 0x42)) {
        Serial.println("GPS module not detected. Check wiring!");
        while (1);
    }
    Serial.println("GPS initialized!");

    // Capture the starting point
    while (!startPointCaptured) {
        myGNSS.checkUblox();
        startLat = myGNSS.getLatitude() / 10000000.0;
        startLon = myGNSS.getLongitude() / 10000000.0;

        if (startLat != 0 && startLon != 0) {
            startPointCaptured = true;
            Serial.print("Start Point Captured: Lat=");
            Serial.print(startLat, 6);
            Serial.print(", Lon=");
            Serial.println(startLon, 6);
        }
        delay(1000);
    }
}

void loop() {
    // Step 1: Calculate middle point (e.g., 10m north and 2m east)
    midLat = startLat + 0.0001; // Approx. 10m north
    midLon = startLon + 0.00002; // Approx. 2m east

    // Step 2: Move to the middle point
    Serial.println("Moving to Middle Point...");
    moveForwardGPS(150, midLat, midLon);

    // Step 3: Perform cover path planning at the middle point
    Serial.println("Performing Cover Path Planning...");
    coverPathPlanning();

    // Step 4: Capture final GPS point after path planning
    myGNSS.checkUblox();
    finalLat = myGNSS.getLatitude() / 10000000.0;
    finalLon = myGNSS.getLongitude() / 10000000.0;
    Serial.print("Final Point Captured: Lat=");
    Serial.print(finalLat, 6);
    Serial.print(", Lon=");
    Serial.println(finalLon, 6);

    // Step 5: Return to the starting point
    Serial.println("Returning to Start Point...");
    moveForwardGPS(150, startLat, startLon);

    while (1); // Stop execution
}

