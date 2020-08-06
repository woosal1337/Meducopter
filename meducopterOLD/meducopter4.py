from adafruit_servokit import *
import time
import keyboard
import tkinter as tk

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
        
        self.val0= 27.5
        self.val1= 30
        self.val2= 16.5
        self.val3= 29

        kit.servo[0].angle = self.medValue0  # X AXIS MOVING
        kit.servo[1].angle = self.medValue1  # Y AXIS MOVING
        kit.servo[2].angle = self.val2 # THROTTLE
        kit.servo[3].angle = self.medValue3  # TURNING


    def stabilize(self): 
        kit.servo[0].angle = self.medValue0
        kit.servo[1].angle = self.medValue1
        kit.servo[2].angle = self.medValue2
        kit.servo[3].angle = self.medValue3

    def setArmed(self, armed):  # ARM DISARM FUNCTION
        if armed == True:
            kit.servo[2].angle = self.minValue2
            kit.servo[3].angle = self.maxValue3
            time.sleep(5)
            kit.servo[2].angle = self.val2

        if armed == False:
            kit.servo[2].angle = self.minValue2
            kit.servo[3].angle = self.minValue3
            time.sleep(5)
            kit.servo[2].angle = self.val2
  

    def setAltitude(self, altitudeValue, time1):
        kit.servo[2].angle = self.medValue2 + altitudeValue

    def move(self, xValue=0, yValue=0):  # CONTROL THE X,Y AXIS IN SAME ALTITUDE
        if yValue != 0:
            if xValue != 0:
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
    
def key(event):
    if event.char == event.keysym: #-- standard keys
        if event.keysym == 'k':
            print("Armed")
            drone.setArmed(True)

        elif event.keysym == 'v':
            drone.setAltitude(5, 0)
            print("Yukari")
        elif event.keysym == 'b':
            drone.setAltitude(-5,0)
            print("Asagi")
        elif event.keysym == 'l':
            drone.setArmed(False)
            print("Disarming")
        elif event.keysym == 'c':
            drone.fail()
            drone.setArmed(False)
            
            print("GG")
            

    else: #-- non standard keys
        if event.keysym == 'Up':
            drone.move(0,15)
            print("Yukari")
        elif event.keysym == 'Down':
            drone.move(0, -15)
            print("Asagi")
        elif event.keysym == 'Left':
            drone.move(-15, 0)
            print("sol")
        elif event.keysym == 'Right':
            drone.move(15, 0)
            print("sag")
    

   


# while True:
#         if keyboard.is_pressed('a'):
#             print('left')
#             drone.move(-15, 0)
#         elif keyboard.is_pressed('s'):
#             print('backward')
#             drone.move(0, -15)
#         elif keyboard.is_pressed('d'):
#             print('right')
#             drone.move(15, 0)
#         elif keyboard.is_pressed('w'):
#             print('forward')
#             drone.move(0, 15)
#         elif keyboard.is_pressed('b'):
#             print('go down')
#             drone.setAltitude(-2,0)
#         elif keyboard.is_pressed('v'):
#             print('go up')
#             drone.setAltitude(2, 0)
#         elif keyboard.is_pressed('k'):
#             print('arming')
#             drone.setArmed(True)
#         elif keyboard.is_pressed('l'):
#             print('disarming')
#             drone.fail()
#             drone.setArmed(False)
#         elif keyboard.is_pressed('space'):
#             print('fail')
#             drone.fail()
#             drone.setArmed(False)
#             break
#         else:
#             drone.setAltitude(0,0)
# 
# 
drone = Movements()
root = tk.Tk()

root.bind_all('<Key>', key)

root.mainloop()