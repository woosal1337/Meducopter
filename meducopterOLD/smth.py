from adafruit_servokit import ServoKit
import time
import meducopter2
kit = ServoKit(channels=16)
drone= meducopter2.Movements()

drone.stabilize()
time.sleep(1)

drone.setArmed(True)

drone.stabilize()
time.sleep(1)

drone.move(0, 20)
time.sleep(5)

drone.stabilize()
time.sleep(1)

drone.setArmed(False)

