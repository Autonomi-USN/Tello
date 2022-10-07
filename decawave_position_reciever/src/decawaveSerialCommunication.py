from typing import Any
import serial
import time
import datetime
import rospy
from decawave_position_reciever.msg import Decawave_position, Decawave_list
from collections import Counter

class Decawave_Bridge_Node:
    def __init__(self):
        rospy.init_node("decawave_bridge_node")
        rospy.loginfo("Starting Decawave_Bridge_Node.")
        self.decawave_data_pub = rospy.Publisher("decawave_data", Decawave_list, queue_size=10)
        self.data_list = Decawave_list()
        self.list =[]

        self.DWM=serial.Serial(port="/dev/ttyACM0", baudrate=115200)
        print("Connected to " + self.DWM.name)
        self.DWM.flushInput()
        self.DWM.flushOutput()
        self.DWM.write("\r\r".encode())
        time.sleep(1)
        self.DWM.write("lec\r".encode())
        time.sleep(1)
        self.publish_decawave_data()

    def decawave_shutdown_sequence(self):
        print("Shutting down")
        self.DWM.write("\r".encode())
        self.DWM.close()

    def publish_decawave_data(self):
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.DWM.flushInput()
                self.DWM.flushOutput()
                line=self.DWM.readline()
                if(line):
                    if len(line)>=32:
                        parse=line.decode().split(",")
                        #print(parse)
                        dwm_name = parse[2]
                        x_pos=parse[3]
                        y_pos=parse[4]
                        z_pos=parse[5]
                        #val = (x_pos,y_pos)
                        #print(dwm_name, ": ", datetime.datetime.now().strftime("%H:%M:%S"),"(",x_pos,", ",y_pos,", ",z_pos,")")
                        tag_data = Decawave_position()
                        tag_data.name = dwm_name
                        tag_data.stamp = rospy.rostime.Time().now()
                        tag_data.x = float(x_pos)
                        tag_data.y = float(y_pos)
                        tag_data.z = float(z_pos)
                        tag_name_exists = False
                        exist_index = None
                        for i in range(len(self.list)):
                            if self.list[i].name == dwm_name:
                                tag_name_exists = True
                                exist_index = i
                        if tag_name_exists:
                            self.list[exist_index] = tag_data
                        else:
                            self.list.append(tag_data)
                        self.data_list.amount = len(self.list)
                        self.data_list.stamp = rospy.rostime.Time().now()
                        self.data_list.decawave_positions = self.list
                        self.decawave_data_pub.publish(self.data_list)
                        

                    #else:
                        #print("Position not calculated: ",line.decode())
            except Exception as ex:
                print(ex)
                break
        rate.sleep()


if __name__ == "__main__":
    decawave_bridge_node = Decawave_Bridge_Node()
    rospy.on_shutdown(decawave_bridge_node.decawave_shutdown_sequence)
    
    rospy.spin()