#include <Arduino.h>

// --- Constants ---
const int TRIG_PIN = 11;
const int ECHO_PIN = 3;  // This MUST be an interrupt pin (e.g., 2 or 3 on Uno/Nano)

// --- Volatile variables ---
// These are shared between the main loop and the Interrupt Service Routine (ISR)
// 'volatile' tells the compiler that these variables can change unexpectedly.
volatile unsigned long pulseStartTime = 0;
volatile unsigned long pulseDuration = 0;
volatile boolean newReadingAvailable = false;

// --- Other global variables ---
unsigned long lastTriggerTime = 0;  // Tracks when we last sent a trigger pulse
const int TRIGGER_INTERVAL = 60;    // How often to trigger a new reading (ms)
                                    // Min 60ms is safe for HC-SR04

// --- Interrupt Service Routine (ISR) ---
// This function is called automatically whenever the ECHO_PIN changes state
void handleEcho() {
    // Check if the pin just went HIGH
    if (digitalRead(ECHO_PIN) == HIGH) {
        pulseStartTime = micros();  // Record the start time
    }
    // Otherwise, the pin must have just gone LOW
    else
    {
        pulseDuration = micros() - pulseStartTime;  // Calculate the pulse duration
        newReadingAvailable = true;                 // Set the flag for the main loop
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Attach the interrupt.
    // We trigger the 'handleEcho' function on ANY state change (RISING or FALLING)
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), handleEcho, CHANGE);
}

// --- Main Loop ---
void loop() {
    unsigned long currentMillis = millis();

    // --- 1. Trigger a New Sensor Reading (Non-Blocking) ---
    // Check if it's time to send a new trigger pulse
    if (currentMillis - lastTriggerTime >= TRIGGER_INTERVAL) {
        lastTriggerTime = currentMillis;  // Update the last trigger time

        // Send the 10-microsecond trigger pulse
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);
    }

    // --- 2. Check for a New Reading from the ISR ---
    if (newReadingAvailable) {
        unsigned long duration;  // Local variable to hold the duration

        // --- Critical Section ---
        // Disable interrupts temporarily to safely copy the volatile variable
        noInterrupts();
        duration = pulseDuration;
        newReadingAvailable = false;  // Clear the flag
        interrupts();
        // -----------------------

        // Calculate the distance in centimeters
        // Sound speed = 343 m/s = 0.0343 cm/us
        // Distance = (Duration * Speed of Sound) / 2 (for round trip)
        float distance = (duration * 0.0343) / 2.0;

        // Print the result
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    }

    // --- 3. Do Other Non-Blocking Things ---
    // You can add any other code here.
    // As long as it doesn't use 'delay()', your loop will run
    // quickly and be responsive.
    //
    // Example:
    // checkButtons();
    // updateLEDs();
    // myOtherTask();
}
