#pragma once

#include <Arduino.h>
#include <BMI160Gen.h>

/**
 * @brief Handles initialization, calibration, and dead-reckoning (integration)
 * of yaw angle using the Z-axis of the BMI160 gyroscope.
 */
class GyroTracker
{
public:
    // Constructor
    GyroTracker();

    /**
     * @brief Initializes the BMI160 sensor with I2C mode, sets the gyro rate (800 Hz)
     * and range (±250 deg/s).
     * @return true if initialization was successful, false otherwise.
     */
    bool begin();

    /**
     * @brief Performs the gyro calibration routine by averaging stationary Z-axis readings.
     * This calculates the gyro drift offset in raw LSB units.
     * @param samples The number of samples to take for the average. Defaults to 500.
     */
    void calibrate(int samples = 500);

    /**
     * @brief Reads the gyro, subtracts the offset, calculates angular velocity,
     * integrates the rotation over the time delta, and updates the yaw angle.
     */
    void update();

    /**
     * @brief Returns the current yaw angle.
     * @return The yaw angle in degrees (0.0 to 360.0).
     */
    float getYaw() const;

    /**
     * @brief Returns the calculated raw Z-axis offset (in LSB units).
     * @return The Z-axis drift offset.
     */
    float getOffset() const;

private:
    float yaw;
    unsigned long lastTime;
    float gyroOffsetZ;  // Stored in raw LSB units

    // Conversion factor for ±250°/s range (131.0 LSB/°/s)
    static constexpr float LSB_PER_DEG_S = 131.0;
};
