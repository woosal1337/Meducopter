from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)
kit.servo[0].set_pulse_width_range(1000, 2200)
kit.servo[1].set_pulse_width_range(1000, 2200)
kit.servo[2].set_pulse_width_range(1000, 2200)
kit.servo[3].set_pulse_width_range(1000, 2200)
class Movements:
    def __init__(self,maxValue, minValue, medValue,armed=False):
        self.maxValue = maxValue
        self.minValue = minValue
        self.medValue = medValue
        self.armed = armed
        kit.servo[0].angle = medValue#X AXIS MOVING
        kit.servo[1].angle = medValue#Y AXIS MOVING
        kit.servo[2].angle = medValue#THROTTLE
        kit.servo[3].angle = medValue#TURNING
        kit.servo[4].angle = medValue#CHANGE MODE
        kit.servo[5].angle = medValue
        kit.servo[6].angle = medValue
        kit.servo[7].angle = medValue
    def setArmed(self,armed): #ARM DISARM FUNCTION
        if armed==True:
            kit.servo[2].angle = self.minValue
            kit.servo[3].angle = self.maxValue
            time.sleep(5)
        if armed==False:
            kit.servo[2].angle = self.minValue
            time.sleep(1)
            kit.servo[3].angle = self.minValue
            time.sleep(5)
    def setAltitude(self,altitudeValue,time1):
        kit.servo[2].angle = self.medValue+altitudeValue
        time.sleep(time1)
        kit.servo[2].angle = self.medValue
    def move(self,xValue=0,yValue=0): #CONTROL THE X,Y AXIS IN SAME ALTITUDE
        if yValue!=0:
            if xValue!=0: #CHANGE X+Y POSITIONS
                kit.servo[1].angle = self.medValue - yValue
                kit.servo[0].angle = self.medValue + xValue
            else: #CHANGE ONLY Y POSITION
                kit.servo[1].angle = self.medValue - yValue
        else:
            if xValue!=0:#CHANGE ONLY X POSITION
                kit.servo[0].angle = self.medValue + xValue
    def turn(self,turnValue): #DRONE WILL GO DOWN
        kit.servo[3].angle = self.medValue + turnValue
    def turn(self,turnValue): #DRONE WILL GO DOWN
        kit.servo[3].angle = self.medValue + turnValue
    def stabilize(self): #DRONE WILL GO DOWN
        kit.servo[0].angle = self.medValue
        kit.servo[1].angle = self.medValue
        kit.servo[2].angle = self.medValue
        kit.servo[3].angle = self.medValue
        kit.servo[4].angle = self.medValue
        kit.servo[5].angle = self.medValue
        kit.servo[6].angle = self.medValue
        kit.servo[7].angle = self.medValue