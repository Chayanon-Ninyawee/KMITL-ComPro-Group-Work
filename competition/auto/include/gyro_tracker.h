#pragma once

#include <Arduino.h>
#include <BMI160Gen.h>

/**
 * @brief Handles initialization, calibration, and dead-reckoning (integration)
 * of yaw angle using the Z-axis of the BMI160 gyroscope.
 *
 * This class is designed to be used with the BMI160's "Data Ready" interrupt.
 * It uses a static wrapper to interface the C-style interrupt with the class instance.
 *
 * @note This class assumes it is a singleton (only one instance exists).
 * You should only create one GyroTracker object globally.
 */
class GyroTracker
{
public:
    /**
     * @brief Constructor. Initializes volatile flags and registers the instance.
     */
    GyroTracker();

    /**
     * @brief Initializes the BMI160 sensor, sets gyro parameters,
     * enables the Data Ready (DRDY) interrupt, and attaches the ISR.
     * @return true if initialization was successful, false otherwise.
     */
    bool begin();

    /**
     * @brief Performs the gyro calibration routine by polling stationary Z-axis readings.
     * This calculates the gyro drift offset in raw LSB units.
     * @param samples The number of samples to take for the average. Defaults to 500.
     */
    void calibrate(int samples = 500);

    /**
     * @brief This is the Interrupt Service Routine (ISR) callback.
     * It just sets a flag for the main loop to process.
     * It should be called by the static C-style wrapper.
     * @warning Do not call this directly from your main loop!
     */
    void isr();

    /**
     * @brief Processes new data if available (i.e., if the ISR has run).
     * This function should be called repeatedly from the main loop().
     * It now handles reading the gyro data via I2C.
     * It performs the integration and updates the yaw angle.
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
    float gyroOffsetZ;  // Stored in raw LSB units

    // --- Volatile variables ---
    // These are modified by the ISR (isr()) and read by the main loop (update())
    volatile unsigned long interruptTime;
    volatile bool newDataAvailable;

    // Timekeeping for integration, only accessed in update()
    unsigned long lastInterruptTime;

    // Conversion factor for ±250°/s range (131.0 LSB/°/s)
    static constexpr float LSB_PER_DEG_S = 131.0;
};
