from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)
kit.servo[0].set_pulse_width_range(1000, 2200)
kit.servo[1].set_pulse_width_range(1000, 2200)
kit.servo[2].set_pulse_width_range(1000, 2200)
kit.servo[3].set_pulse_width_range(1000, 2200)
kit.servo[0].angle = 165
kit.servo[1].angle = 0
kit.servo[2].angle = 165
kit.servo[3].angle = 0

