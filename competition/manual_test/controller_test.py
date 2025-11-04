import time

import cyberpi
import gamepad

# This script continuously reads the joystick's values and button presses,
# then displays them on the CyberPi's screen using the new gamepad module.

while True:
    # Clear the console on the screen to show the new values
    cyberpi.console.clear()

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

    display_text = (
        "Lx: "
        + str(lx_value)
        + "\nLy: "
        + str(ly_value)
        + "\nUp: "
        + str(up_pressed)
        + "\nDown: "
        + str(down_pressed)
        + "\nLeft: "
        + str(left_pressed)
        + "\nRight: "
        + str(right_pressed)
    )

    cyberpi.display.show_label(display_text, 16, 0, 0)

    # This prevents the screen from flickering too quickly
    time.sleep(0.1)
