from adafruit_servokit import *
import time
import keyboard
import autolib

drone = autolib.Movements()

drone.setArmed(False)