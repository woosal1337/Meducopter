from adafruit_servokit import *
import time
import keyboard
import autolib

drone = autolib.Movements()
time.sleep(1)


drone.setArmed(True)
time.sleep(1)


drone.setArmed(False)