from adafruit_servokit import *
import time
import keyboard
import smoothlib



drone = smoothlib.Movements()

time.sleep(1) 
drone.setArmed(True)
drone.goUp(3)
drone.altHold()
time.sleep(0.5)
drone.land()
drone.setArmed(False)

#test 2 take off, go forward, land
# drone.setArmed(True)
# drone.goUp(3)
# 
# drone.altHold()
# 
# drone.forward(3)
# 
# drone.altHold()
# 
# drone.goDown(5)
# drone.setArmed(False)