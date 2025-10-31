#include "motor_controller.h"

// --- Constructor ---
MotorController::MotorController(int left1Pin, int left2Pin, int leftEnablePin, int right1Pin, int right2Pin, int rightEnablePin) {
    // Store all pin numbers in private member variables
    _l1 = left1Pin;
    _l2 = left2Pin;
    _len = leftEnablePin;

    _r1 = right1Pin;
    _r2 = right2Pin;
    _ren = rightEnablePin;
}

// --- Initialization ---
void MotorController::begin() {
    // Set all six motor pins as outputs
    pinMode(_l1, OUTPUT);
    pinMode(_l2, OUTPUT);
    pinMode(_len, OUTPUT);

    pinMode(_r1, OUTPUT);
    pinMode(_r2, OUTPUT);
    pinMode(_ren, OUTPUT);

    // Ensure motors are stopped on initialization
    stop();
}

// --- Private Motor Control Helper ---
void MotorController::setMotorSpeed(int in1, int in2, int en, int speed) {
    // Clamp speed to valid range [-255, 255]
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    // Determine Direction
    if (speed > 0) {  // Forward
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (speed < 0) {  // Reverse
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {  // Stop (0 speed)
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }

    // Set Speed (PWM signal on the Enable pin)
    // The absolute value of speed is used for analogWrite
    analogWrite(en, abs(speed));
}

// --- Public Move Function ---
void MotorController::move(int leftSpeed, int rightSpeed) {
    setMotorSpeed(_l1, _l2, _len, leftSpeed);
    setMotorSpeed(_r1, _r2, _ren, rightSpeed);
}

// --- Public Stop Function ---
void MotorController::stop() {
    // Setting speed to 0 automatically handles the direction pins inside setMotorSpeed
    move(0, 0);
}
