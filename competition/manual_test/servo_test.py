import time

import cyberpi

cyberpi.mbot2.servo_add(0, "S1")

while True:
    cyberpi.mbot2.servo_set(180, "S1")
    time.sleep(2)
    cyberpi.mbot2.servo_set(0, "S1")
    time.sleep(2)
