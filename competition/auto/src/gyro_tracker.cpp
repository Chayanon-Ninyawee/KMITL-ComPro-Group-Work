#include "gyro_tracker.h"
// BMI160Gen.h is included via GyroTracker.h

// --- Static ISR Wrapper ---
// This C++ class needs to interface with a C-style interrupt.
// We create a global pointer to the single GyroTracker instance.

static GyroTracker *globalGyroTrackerInstance = nullptr;

/**
 * @brief C-style static function to wrap the class-based ISR.
 * This is the function address passed to BMI160.attachInterrupt().
 */
static void static_isr_wrapper() {
    if (globalGyroTrackerInstance) {
        globalGyroTrackerInstance->isr();
    }
}
// --- End Static ISR Wrapper ---

GyroTracker::GyroTracker()
    : yaw(0.0)
    , gyroOffsetZ(0.0)
    , interruptTime(0)
    , newDataAvailable(false)
    , lastInterruptTime(0) {
    // Register this instance as the global one for the ISR wrapper
    // This assumes you only create ONE GyroTracker object.
    globalGyroTrackerInstance = this;
}

bool GyroTracker::begin() {
    // 1. Initialize BMI160
    if (!BMI160.begin(BMI160GenClass::I2C_MODE)) {
        Serial.println("Failed to initialize BMI160");
        return false;
    }

    // 2. Set sensor parameters
    BMI160.setGyroRate(800);   // 800 Hz sample rate
    BMI160.setGyroRange(250);  // ±250 deg/s range

    // 3. Configure the BMI160 to generate an interrupt on "Data Ready"
    // This tells the chip to pulse the INT1 pin when new gyro data is available.
    BMI160.setIntDataReadyEnabled(true);

    // 4. Attach our static wrapper function to the interrupt.
    // The BMI160Gen library handles which pin (INT1 or INT2) is used.
    BMI160.attachInterrupt(static_isr_wrapper);
    Serial.println("BMI160 interrupt attached.");

    // 5. Reset timekeeping
    lastInterruptTime = 0;
    newDataAvailable = false;

    return true;
}

void GyroTracker::calibrate(int samples) {
    // Reset offset before calculation
    gyroOffsetZ = 0.0;
    long gzSum = 0;

    if (samples <= 0) samples = 500;

    Serial.println("Calibrating gyro (keep stationary for a few seconds)...");

    // We use the interrupt flag for calibration as well.
    for (int i = 0; i < samples; i++) {
        // Wait for the ISR to set the flag
        while (!newDataAvailable) {
            // Yield or sleep to be nice to the processor
            delayMicroseconds(100);
        }

        // Flag is set, so new data is ready.
        // Read the data from the sensor (this also clears the interrupt)
        int gx_temp, gy_temp, gz_local;
        BMI160.readGyro(gx_temp, gy_temp, gz_local);

        // Consume the flag *after* reading
        newDataAvailable = false;

        gzSum += gz_local;
    }

    // Calculate the average stationary reading (the drift offset) in raw LSB units
    gyroOffsetZ = (float)gzSum / samples;

    Serial.print("Calibration complete. Gyro offset Z (LSB): ");
    Serial.println(gyroOffsetZ);

    // Reset yaw and timekeeping to start fresh
    yaw = 0.0;
    lastInterruptTime = 0;
    newDataAvailable = false;
}

/**
 * @brief Interrupt Service Routine.
 * This function is called by the hardware interrupt (via the static wrapper)
 * It should be as fast as possible.
 * It reads the raw data and sets a flag for the main loop to process.
 */
void GyroTracker::isr() {
    // This is all an ISR should do:
    // Set a flag and record the time.
    // DO NOT do I2C/SPI reads/writes here, it will cause a deadlock!
    // The readGyro() in update() will clear the interrupt source on the chip.
    interruptTime = micros();
    newDataAvailable = true;
}

/**
 * @brief Main loop processing function.
 * Checks if the ISR has provided new data, then processes it.
 * This should be called from your main loop().
 */
void GyroTracker::update() {
    // Check if the ISR has set the flag
    if (!newDataAvailable) {
        return;  // No new data to process
    }

    // New data is available. Read it now from the main loop.
    // This reads the data via I2C and also clears the interrupt flag on the BMI160.
    int gx_temp, gy_temp, gz_local;
    BMI160.readGyro(gx_temp, gy_temp, gz_local);

    // --- Critical Section ---
    // Safely copy the volatile data (interruptTime)
    // and reset the flag.
    unsigned long now;

    noInterrupts();  // Disable interrupts
    now = interruptTime;
    newDataAvailable = false;  // Reset the flag
    interrupts();              // Re-enable interrupts
    // --- End Critical Section ---

    // 1. Check for first run or micros() overflow
    // If this is the first interrupt, we can't calculate dt yet.
    if (lastInterruptTime == 0 || now < lastInterruptTime) {
        lastInterruptTime = now;
        return;  // Skip first calculation to get a valid dt
    }

    // 2. Calculate time delta (dt) in seconds since the *last interrupt*
    float dt = (now - lastInterruptTime) / 1000000.0;
    lastInterruptTime = now;

    // 3. Subtract the calculated offset from the raw reading (LSB)
    float gz_raw_compensated = (float)gz_local - gyroOffsetZ;

    // 4. Convert the compensated raw reading to angular velocity (deg/s)
    float angularVelocityZ = gz_raw_compensated / LSB_PER_DEG_S;

    // 5. Integrate (multiply angular velocity by time) to get the angle change
    yaw += angularVelocityZ * dt * 1.0035;

    // 6. Keep yaw constrained within the 0–360 degree range
    if (yaw >= 360.0) {
        yaw -= 360.0;
    } else if (yaw < 0.0) {
        yaw += 360.0;
    }
}

float GyroTracker::getHeading() const {
    // It's good practice to disable interrupts when reading multi-byte
    // variables that can be written by an ISR, but 'yaw' (a float)
    // is only written by update(), which is in the main loop.
    // If 'yaw' were written by the ISR, we would need to protect this read.
    // Since it's only written by update(), this is safe.
    return yaw;
}

float GyroTracker::getOffset() const {
    return gyroOffsetZ;
}
