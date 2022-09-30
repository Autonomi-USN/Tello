#DWM Serial Parser by Brian H. | Updated 3/6/19
import serial
import time
import datetime

DWM=serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print("Connected to " +DWM.name)
DWM.flushInput()
DWM.flushOutput()
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)
while True:
    try:
        DWM.flushInput()
        DWM.flushOutput()
        line=DWM.readline()
        if(line):
            if len(line)>=32:
                parse=line.decode().split(",")
                #print(parse)
                dwm_name = parse[2]
                if dwm_name == '5C14':
                    x_pos=parse[3]
                    y_pos=parse[4]
                    z_pos=parse[5]
                    #val = (x_pos,y_pos)
                    print(dwm_name, ": ", datetime.datetime.now().strftime("%H:%M:%S"),"(",x_pos,", ",y_pos,", ",z_pos,")")
            #else:
                #print("Position not calculated: ",line.decode())
    except Exception as ex:
        print(ex)
        break
DWM.write("\r".encode())
DWM.close()