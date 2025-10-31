#pragma once

#include <Arduino.h>

class MotorController
{
public:
    /**
     * @brief Constructor for the MotorController class.
     * * @param left1Pin Digital pin for left motor direction 1 (IN1).
     * @param left2Pin Digital pin for left motor direction 2 (IN2).
     * @param leftEnablePin PWM pin for left motor speed (ENA).
     * @param right1Pin Digital pin for right motor direction 1 (IN3).
     * @param right2Pin Digital pin for right motor direction 2 (IN4).
     * @param rightEnablePin PWM pin for right motor speed (ENB).
     */
    MotorController(int left1Pin, int left2Pin, int leftEnablePin, int right1Pin, int right2Pin, int rightEnablePin);

    /**
     * @brief Initializes the motor pins as outputs.
     */
    void begin();

    /**
     * @brief Sets the speed and direction for both motors.
     * * Speed ranges from -255 (full reverse) to 255 (full forward).
     * 0 is stop.
     * * @param leftSpeed Target speed for the left motor (-255 to 255).
     * @param rightSpeed Target speed for the right motor (-255 to 255).
     */
    void move(int leftSpeed, int rightSpeed);

    /**
     * @brief Stops both motors instantly.
     */
    void stop();

private:
    // Pin storage for the left motor
    int _l1, _l2, _len;
    // Pin storage for the right motor
    int _r1, _r2, _ren;

    /**
     * @brief Private helper function to control a single motor.
     * * @param in1 Direction pin 1.
     * @param in2 Direction pin 2.
     * @param en Enable/PWM pin.
     * @param speed Speed value (-255 to 255).
     */
    void setMotorSpeed(int in1, int in2, int en, int speed);
};
