#include <Arduino.h>

#include <BMI160Gen.h>

float yaw = 0.0;
unsigned long lastTime = 0;

// Calibration offset for gyro drift (in deg/s)
float gyroOffsetZ = 0.0;

void setup() {
    Serial.begin(115200);

    // Initialize BMI160
    BMI160.begin(BMI160GenClass::I2C_MODE);
    BMI160.setGyroRate(800);   // 800 Hz
    BMI160.setGyroRange(250);  // ±250 deg/s

    // Calibrate gyro offset (average of stationary readings)
    Serial.println("Calibrating gyro...");
    const int samples = 500;
    long gzSum = 0;
    for (int i = 0; i < samples; i++) {
        int gx, gy, gz;
        BMI160.readGyro(gx, gy, gz);
        gzSum += gz;
        delay(2);
    }
    gyroOffsetZ = (float)gzSum / samples;
    Serial.print("Gyro offset Z: ");
    Serial.println(gyroOffsetZ);

    lastTime = micros();
}

void loop() {
    int gx, gy, gz;
    BMI160.readGyro(gx, gy, gz);

    // Subtract offset
    gz -= gyroOffsetZ;

    // Convert to deg/s
    // Conversion factor depends on range setting:
    // ±250°/s → 131.0 LSB/°/s
    float gzf = gz / 131.0;

    // Time delta in seconds
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;

    // Integrate yaw
    yaw += gzf * dt;

    // Keep yaw in 0–360 range
    if (yaw >= 360.0)
        yaw -= 360.0;
    else if (yaw < 0.0)
        yaw += 360.0;

    Serial.print("Yaw: ");
    Serial.println(yaw);
}
