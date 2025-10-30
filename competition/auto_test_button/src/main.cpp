#include <Arduino.h>

#define BUTTON_PIN 10

void setup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    Serial.begin(115200);
}

void loop() {
    int buttonValue = digitalRead(BUTTON_PIN);

    Serial.println(buttonValue);
}
