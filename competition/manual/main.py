import time

import cyberpi
import gamepad


def mbot2_drive(power, steering_percent):
    power = max(-100, min(100, power)) * 1.5
    steering_percent = max(-100, min(100, steering_percent)) * 1.2

    power_left = power + steering_percent
    power_right = power - steering_percent

    # Clamp the results to the -200 to 200 range
    power_left = max(-200, min(200, power_left))
    power_right = max(-200, min(200, power_right))

    cyberpi.mbot2.EM_set_speed(power_left, "EM1")
    cyberpi.mbot2.EM_set_speed(-power_right, "EM2")  # Negative to move forward


cyberpi.mbot2.servo_add(0, "S1")
cyberpi.mbot2.servo_add(0, "S3")

cyberpi.mbot2.EM_set_power(0, "EM1")
cyberpi.mbot2.EM_set_power(0, "EM2")

time.sleep(1)

manual_mode = True

while True:
    # Read the value of the left joystick on the x-axis
    # The value ranges from -100 to 100
    lx_value = gamepad.get_joystick("Lx")
    ly_value = gamepad.get_joystick("Ly")
    rx_value = gamepad.get_joystick("Rx")
    ry_value = gamepad.get_joystick("Ry")

    up_pressed = gamepad.is_key_pressed("Up")
    down_pressed = gamepad.is_key_pressed("Down")
    left_pressed = gamepad.is_key_pressed("Left")
    right_pressed = gamepad.is_key_pressed("Right")

    n1_pressed = gamepad.is_key_pressed("N1")
    n2_pressed = gamepad.is_key_pressed("N2")
    n3_pressed = gamepad.is_key_pressed("N3")
    n4_pressed = gamepad.is_key_pressed("N4")

    select_pressed = gamepad.is_key_pressed("Select")
    start_pressed = gamepad.is_key_pressed("Start")

    l1_pressed = gamepad.is_key_pressed("L1")
    l2_pressed = gamepad.is_key_pressed("L2")
    r1_pressed = gamepad.is_key_pressed("R1")
    r2_pressed = gamepad.is_key_pressed("R2")

    # cyberpi.mbot2.servo_set(80, "S3")
    # cyberpi.mbot2.servo_set(170, "S1")
    # time.sleep(4)
    # cyberpi.mbot2.servo_set(0, "S3")
    # time.sleep(1)
    # cyberpi.mbot2.servo_set(30, "S1")
    # time.sleep(0.35)
    # cyberpi.mbot2.servo_set(80, "S3")
    # time.sleep(4)

    # cyberpi.mbot2.motor_set(50, "M1")
    # cyberpi.mbot2.motor_set(50, "M2")

    # display_text = (
    #     "Lx: "
    #     + str(lx_value)
    #     + "\nLy: "
    #     + str(ly_value)
    #     + "\nUp: "
    #     + str(up_pressed)
    #     + "\nDown: "
    #     + str(down_pressed)
    #     + "\nLeft: "
    #     + str(left_pressed)
    #     + "\nRight: "
    #     + str(right_pressed)
    # )

    # cyberpi.display.show_label(display_text, 16, 0, 0)

    # Move arm to pickup
    if l1_pressed:
        cyberpi.mbot2.servo_set(80, "S3")
        cyberpi.mbot2.servo_set(170, "S1")

    # Move arm to pickup on the middle
    if l2_pressed:
        cyberpi.mbot2.servo_set(80, "S3")
        cyberpi.mbot2.servo_set(150, "S1")

    # Hold the gripper
    if r2_pressed:
        cyberpi.mbot2.servo_set(0, "S3")

    # Release the gripper
    if n1_pressed:
        cyberpi.mbot2.servo_set(80, "S3")

    if n3_pressed:
        current_angle = cyberpi.mbot2.servo_get("S1")
        for angle in range(current_angle, 80, -1):
            cyberpi.mbot2.servo_set(angle, "S1")
            time.sleep(0.008)

    if n4_pressed:
        current_angle = cyberpi.mbot2.servo_get("S1")
        for angle in range(current_angle, 170, 1):
            cyberpi.mbot2.servo_set(angle, "S1")
            time.sleep(0.008)

    # Throw the ball
    if r1_pressed:
        cyberpi.mbot2.servo_set(30, "S1")
        time.sleep(0.35)
        cyberpi.mbot2.servo_set(80, "S3")
        time.sleep(0.8)

    if n2_pressed:
        manual_mode = not manual_mode

    if manual_mode:
        mbot2_drive(ly_value, rx_value)
    else:
        mbot2_drive(0, 0)

    time.sleep(0.0167)
