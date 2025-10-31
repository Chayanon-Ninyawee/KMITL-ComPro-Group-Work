#include <Arduino.h>

#define MOTOR_LEFT_1 7
#define MOTOR_LEFT_2 8
#define MOTOR_LEFT_EN 6

#define MOTOR_RIGHT_1 4
#define MOTOR_RIGHT_2 5
#define MOTOR_RIGHT_EN 3

void setup() {
    pinMode(MOTOR_LEFT_1, OUTPUT);
    pinMode(MOTOR_LEFT_2, OUTPUT);
    pinMode(MOTOR_LEFT_EN, OUTPUT);

    pinMode(MOTOR_RIGHT_1, OUTPUT);
    pinMode(MOTOR_RIGHT_2, OUTPUT);
    pinMode(MOTOR_RIGHT_EN, OUTPUT);
}

void loop() {
    digitalWrite(MOTOR_LEFT_1, HIGH);
    digitalWrite(MOTOR_LEFT_2, LOW);
    analogWrite(MOTOR_LEFT_EN, 255);

    digitalWrite(MOTOR_RIGHT_1, HIGH);
    digitalWrite(MOTOR_RIGHT_2, LOW);
    analogWrite(MOTOR_RIGHT_EN, 255);

    delay(2000);

    digitalWrite(MOTOR_LEFT_1, LOW);
    digitalWrite(MOTOR_LEFT_2, HIGH);
    analogWrite(MOTOR_LEFT_EN, 255);

    digitalWrite(MOTOR_RIGHT_1, LOW);
    digitalWrite(MOTOR_RIGHT_2, HIGH);
    analogWrite(MOTOR_RIGHT_EN, 255);

    delay(2000);

    digitalWrite(MOTOR_LEFT_1, HIGH);
    digitalWrite(MOTOR_LEFT_2, LOW);
    analogWrite(MOTOR_LEFT_EN, 255);

    digitalWrite(MOTOR_RIGHT_1, LOW);
    digitalWrite(MOTOR_RIGHT_2, HIGH);
    analogWrite(MOTOR_RIGHT_EN, 255);

    delay(2000);

    digitalWrite(MOTOR_LEFT_1, LOW);
    digitalWrite(MOTOR_LEFT_2, HIGH);
    analogWrite(MOTOR_LEFT_EN, 255);

    digitalWrite(MOTOR_RIGHT_1, HIGH);
    digitalWrite(MOTOR_RIGHT_2, LOW);
    analogWrite(MOTOR_RIGHT_EN, 255);

    delay(2000);
}
