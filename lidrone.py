import serial
import time
#import smoothlib

def start_Serial():
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout = 1,bytesize=8, parity='N', stopbits=1)
    ser.write(bytes(b'B'))
    ser.write(bytes(b'W'))
    ser.write(bytes(2))
    ser.write(bytes(0))
    ser.write(bytes(0))
    ser.write(bytes(0))
    ser.write(bytes(1))
    ser.write(bytes(6))
    ser.write(bytes(b'B'))
    ser.write(bytes(b'W'))
    ser.write(bytes(2))
    ser.write(bytes(0))
    ser.write(0xEE)
    ser.write(0xFF)
    ser.write(bytes(1))          
    ser.write(0x19)
    return ser
def stop_Serial(ser):
    ser.flush()
    ser.close()

ser = start_Serial()
def getAltitude():
    if ser.inWaiting() >= 9:
        rcv = ser.read(9)
        if len(rcv) != 9:
            stop_Serial(ser)
            ser = start_Serial()
        if rcv[0] == 0x59  and rcv[1] == 0x59 :
            Dist_L = rcv[2]
            Dist_H = rcv[3]
            Dist_Total = (Dist_H * 256) + Dist_L
            print(Dist_Total)  
        else:
            stop_Serial(ser)
            ser = start_Serial()
    else:
        stop_Serial(ser)
        ser = start_Serial()