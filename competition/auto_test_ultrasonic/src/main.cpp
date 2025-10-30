#include <Arduino.h>

#include <HCSR04.h>

const byte TRIGGER_PIN = 11;
const byte ECHO_COUNT = 1;
byte *ECHO_PINS = new byte[ECHO_COUNT]{12};

void setup() {
    HCSR04.begin(TRIGGER_PIN, ECHO_PINS, ECHO_COUNT);

    Serial.begin(115200);
}

void loop() {
    double *distances = HCSR04.measureDistanceCm();

    for (int i = 0; i < ECHO_COUNT; i++) {
        if (i > 0) Serial.print(" | ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(distances[i]);
        Serial.print(" cm");
    }

    Serial.println("");
    delay(500);
}
