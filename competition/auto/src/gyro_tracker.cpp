#include "gyro_tracker.h"
// BMI160Gen.h is included via GyroTracker.h

GyroTracker::GyroTracker()
    : yaw(0.0)
    , lastTime(0)
    , gyroOffsetZ(0.0) {}

bool GyroTracker::begin() {
    // 1. Initialize BMI160
    if (!BMI160.begin(BMI160GenClass::I2C_MODE)) {
        return false;
    }

    // 2. Set sensor parameters (matching the original sketch)
    BMI160.setGyroRate(800);   // 800 Hz sample rate
    BMI160.setGyroRange(250);  // ±250 deg/s range (corresponds to LSB_PER_DEG_S = 131.0)

    // 3. Set initial timestamp for the first update calculation
    lastTime = micros();

    return true;
}

void GyroTracker::calibrate(int samples) {
    // Reset offset before calculation
    gyroOffsetZ = 0.0;

    long gzSum = 0;

    if (samples <= 0) samples = 500;

    Serial.println("Calibrating gyro (keep stationary for a few seconds)...");

    for (int i = 0; i < samples; i++) {
        int gx, gy, gz;
        BMI160.readGyro(gx, gy, gz);
        gzSum += gz;
        // Small delay between samples
        delay(2);
    }

    // Calculate the average stationary reading (the drift offset) in raw LSB units
    gyroOffsetZ = (float)gzSum / samples;

    Serial.print("Calibration complete. Gyro offset Z (LSB): ");
    Serial.println(gyroOffsetZ);

    // Reset yaw after calibration to start from 0 degrees
    yaw = 0.0;
}

void GyroTracker::update() {
    int gx, gy, gz;
    BMI160.readGyro(gx, gy, gz);

    // 1. Subtract the calculated offset from the raw reading (LSB)
    float gz_raw_compensated = (float)gz - gyroOffsetZ;

    // 2. Convert the compensated raw reading to angular velocity (deg/s)
    float angularVelocityZ = gz_raw_compensated / LSB_PER_DEG_S;

    // 3. Calculate time delta (dt) in seconds since the last update
    unsigned long now = micros();

    // Basic check for micros() overflow (which happens approx every 71 minutes)
    if (now < lastTime) {
        lastTime = now;
        return;
    }

    float dt = (now - lastTime) / 1000000.0;  // Convert microseconds to seconds
    lastTime = now;

    // 4. Integrate (multiply angular velocity by time) to get the angle change
    yaw += angularVelocityZ * dt;

    // 5. Keep yaw constrained within the 0–360 degree range
    if (yaw >= 360.0) {
        yaw -= 360.0;
    } else if (yaw < 0.0) {
        yaw += 360.0;
    }
}

float GyroTracker::getYaw() const {
    return yaw;
}

float GyroTracker::getOffset() const {
    return gyroOffsetZ;
}
