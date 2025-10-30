#include <Arduino.h>
#include <Servo.h>

#define SERVO_PIN A0
#define SERVO_MIN 20
#define SERVO_MAX 140

Servo armServo;

void setup() {
    armServo.attach(SERVO_PIN);
}

void loop() {
    armServo.write(SERVO_MIN);
    delay(2000);

    armServo.write(SERVO_MAX);
    delay(2000);
}
