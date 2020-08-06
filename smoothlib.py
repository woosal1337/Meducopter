from adafruit_servokit import *
import time
import keyboard

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
        kit.servo[2].angle = 16.5  # THROTTLE
        kit.servo[3].angle = self.medValue3  # TURNING


    def altHold(self): 
        kit.servo[0].angle = self.medValue0
        kit.servo[1].angle = self.medValue1
        kit.servo[2].angle = self.medValue2
        kit.servo[3].angle = self.medValue3

    def setArmed(self, armed):  # ARM DISARM FUNCTION
        if armed == True:
            kit.servo[2].angle = self.minValue2
            kit.servo[3].angle = self.maxValue3
            time.sleep(5)
            kit.servo[3].angle = self.medValue3
            kit.servo[2].angle = self.medValue2
            i=self.minValue2
            while i<self.medValue2:
                kit.servo[2].angle = i
                i+=0.5
                time.sleep(0.02)
                
        if armed == False:
            kit.servo[2].angle = self.minValue2
            time.sleep(1)
            kit.servo[3].angle = self.minValue3
            time.sleep(5)
  

    def goUp(self, time1):
        kit.servo[2].angle = self.medValue2 + 4
        time.sleep(time1)
        if time1!=0:
            kit.servo[2].angle = self.medValue2
    
    def goDown(self, time1):
        kit.servo[2].angle = self.medValue2 - 2
        time.sleep(time1)
        if time1!=0:
            kit.servo[2].angle = self.medValue2
        
    def forward(self,time1):
        kit.servo[1].angle = self.medValue1 - 7
        time.sleep(time1)
        if time1!=0:
            kit.servo[1].angle = self.medValue1
        
    def backward(self,time1):
        kit.servo[1].angle = self.medValue1 + 11.25
        time.sleep(time1)
        if time1!=0:
            kit.servo[1].angle = self.medValue1
        
    def right(self,time1):
        kit.servo[0].angle = self.medValue0 + 6
        time.sleep(time1)
        if time1!=0:
            kit.servo[0].angle = self.medValue0
        
    def left(self,time1):
        kit.servo[0].angle = self.medValue0 - 6
        time.sleep(time1)
        if time1!=0:
            kit.servo[0].angle = self.medValue0

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
    def fail(self):
        kit.servo[0].angle = self.minValue0
        kit.servo[1].angle = self.minValue1
        kit.servo[2].angle = self.minValue2
        kit.servo[3].angle = self.minValue3
        setArmed(False)
    def land(self):
        i=self.medValue2
        while i>self.minValue2:
                kit.servo[2].angle = i
                i-=0.5
                time.sleep(0.04)