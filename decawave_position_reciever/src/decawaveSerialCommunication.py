import time
import rospy
import serial

ser =  serial.Serial('/dev/ttyACM0',115200)
if ser.isOpen:
    print("port opened")
    ser.flushInput()
    ser.flushOutput()
    time.sleep(0.1)
    ser.write('\n'.encode())
    time.sleep(0.1)
    ser.write('\n'.encode())
    time.sleep(0.1)
    ser.write('lec\n'.encode())
    time.sleep(0.1)
    waiting=ser.inWaiting()
    line = ser.read(waiting)
  
    print(line.decode())
    ser.close()
