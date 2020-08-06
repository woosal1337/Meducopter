from adafruit_servokit import ServoKit
import time
import keyboard
import urllib
import meducopter2

kit = ServoKit(channels=16)
kit.servo[0].set_pulse_width_range(1000, 2200)
kit.servo[1].set_pulse_width_range(1000, 2200)
kit.servo[2].set_pulse_width_range(1000, 2200)
kit.servo[3].set_pulse_width_range(1000, 2200)


class Movements:
    def __init__(self):
        self.maxValue0 = 163.9
        self.minValue0 = 27.5
        self.medValue0 = 95.5

        self.maxValue1 = 163.5
        self.minValue1 = 30
        self.medValue1 = 98

        self.maxValue2 = 163.5
        self.minValue2 = 16.5
        self.medValue2 = 101

        self.maxValue3 = 163.5
        self.minValue3 = 29
        self.medValue3 = 95.5

        kit.servo[0].angle = self.medValue0  # X AXIS MOVING

        kit.servo[1].angle = self.medValue1  # Y AXIS MOVING
        kit.servo[2].angle = self.medValue2  # THROTTLE
        kit.servo[3].angle = self.medValue3  # TURNING


    def stabilize(self):  # DRONE WILL GO DOWN
        kit.servo[0].angle = self.medValue0
        kit.servo[1].angle = self.medValue1
        kit.servo[2].angle = self.medValue2
        kit.servo[3].angle = self.medValue3

    def setArmed(self, armed):  # ARM DISARM FUNCTION
        if armed == True:
            kit.servo[2].angle = self.minValue2
            kit.servo[3].angle = self.maxValue3
            time.sleep(5)

            #NEW
            set.stabilize()
            time.sleep(1)

        if armed == False:
            kit.servo[2].angle = self.minValue2
            time.sleep(1)
            kit.servo[3].angle = self.minValue3
            time.sleep(5)

            #NEW
            set.stabilize()
            time.sleep(1)

    def setAltitude(self, altitudeValue, time1):
        kit.servo[2].angle = self.medValue2 + altitudeValue
        time.sleep(time1)
        kit.servo[2].angle = self.medValue2

    def move(self, xValue=0, yValue=0):  # CONTROL THE X,Y AXIS IN SAME ALTITUDE
        if yValue != 0:
            if xValue != 0:  # CHANGE X+Y POSITIONS
                kit.servo[1].angle = self.medValue1 - yValue
                kit.servo[0].angle = self.medValue0 + xValue
            else:  # CHANGE ONLY Y POSITION
                kit.servo[1].angle = self.medValue1 - yValue
        else:
            if xValue != 0:  # CHANGE ONLY X POSITION
                kit.servo[0].angle = self.medValue0 + xValue

    def turn(self, turnValue):
        kit.servo[3].angle = self.medValue3 + turnValue

drone = Movements()

while True:
        if keyboard.is_pressed('a'):  # if key 'q' is pressed
            print('left')
            drone.move(-5, 0)
        elif keyboard.is_pressed('s'):  # if key 'a' is pressed
            print('backward')
            drone.move(0, -5)
        elif keyboard.is_pressed('d'):  # if key 'a' is pressed
            print('right')
            drone.move(5, 0)
        elif keyboard.is_pressed('w'):  # if key 'a' is pressed
            print('forward')
            drone.move(0, 5)
        elif keyboard.is_pressed('space'):  # if key 'a' is pressed
            print('go down')
            setAltitude(-5,0)
        elif keyboard.is_pressed('v'):  # if key 'a' is pressed
            print('go up')
            setAltitude(5, 0)
        else:
            print('stabilize')
            drone.stabilize()