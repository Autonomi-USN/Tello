#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import threading
import av
import traceback
import time
import sys
from subprocess import Popen, PIPE
from tellopy import tello
from tellopy import error
import actionlib
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from tello_bridge.msg import Tello_data
from tello_bridge.msg import TakeoffAction
from tello_bridge.msg import TakeoffResult
from tello_bridge.msg import TakeoffFeedback
from tello_bridge.msg import LandAction
from tello_bridge.msg import LandResult
from tello_bridge.msg import LandFeedback

class Tello_Bridge_Node:
    def __init__(self):
        rospy.init_node("tello_bridge_node")
        rospy.loginfo("Starting Tello_Bridge_Node.")

        self.prev_flight_data = None
        self.flight_data = None
        self.log_data = None
        self.connection_quit = None
        self.landed = True

        self.tello = tello.Tello()

        self.data_msg = Tello_data()
        
        self.tello_data_pub = rospy.Publisher("tello_data", Tello_data, queue_size=10)
        self.connection_quit = True
        self.tello.connect()
        rospy.loginfo("Connecting to Tello drone")
        try:
            self.tello.wait_for_connection(10)
        except error.TelloError as err:
            rospy.logerr(str(err))
            rospy.signal_shutdown(str(err))
            self.tello.quit()
            return
        rospy.loginfo("Connected to Tello drone")
        self.connection_quit = False
        self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self.event_handler)
        self.tello.subscribe(self.tello.EVENT_LOG_DATA, self.event_handler)

            

    def event_handler(self, event, sender, data, **args):
        drone = sender
        if event is drone.EVENT_FLIGHT_DATA:
            self.flight_data = data
            self.tello_flight_data_publish()
        elif event is drone.EVENT_LOG_DATA:
            self.log_data = data
            self.tello_log_data_publish()
        else:
            print('event="%s" data=%s' % (event.getname(), str(data)))


    def tello_log_data_publish(self):
        self.data_msg.vel_x = self.log_data.mvo.vel_x
        self.data_msg.vel_y = self.log_data.mvo.vel_y
        self.data_msg.vel_z = self.log_data.mvo.vel_z
        self.data_msg.pos_x = self.log_data.mvo.pos_x
        self.data_msg.pos_y = self.log_data.mvo.pos_y
        self.data_msg.pos_z = self.log_data.mvo.pos_z
        self.data_msg.acc_x = self.log_data.imu.acc_x
        self.data_msg.acc_y = self.log_data.imu.acc_y
        self.data_msg.acc_z = self.log_data.imu.acc_z
        self.data_msg.gyro_x = self.log_data.imu.gyro_x
        self.data_msg.gyro_y = self.log_data.imu.gyro_y
        self.data_msg.gyro_z = self.log_data.imu.gyro_z
        self.data_msg.vg_x = self.log_data.imu.vg_x
        self.data_msg.vg_y = self.log_data.imu.vg_y
        self.data_msg.vg_z = self.log_data.imu.vg_z
        self.data_msg.q0 = self.log_data.imu.q0
        self.data_msg.q1 = self.log_data.imu.q1
        self.data_msg.q2 = self.log_data.imu.q2
        self.data_msg.q3 = self.log_data.imu.q3
        self.data_msg.landed = self.landed
        self.tello_data_pub.publish(self.data_msg)

    def tello_flight_data_publish(self):
        self.data_msg.battery_percentage = self.flight_data.battery_percentage
        self.data_msg.east_speed = self.flight_data.east_speed/10.0
        self.data_msg.north_speed = self.flight_data.north_speed/10.0
        self.data_msg.ground_speed = self.flight_data.ground_speed
        self.data_msg.height = self.flight_data.height/10.0
        self.data_msg.landed = self.landed
        self.tello_data_pub.publish(self.data_msg)

        
def clamp(n, minn, maxn):
    """
    Clamps a value

    Args:
        n (int/float): The number to clamp
        minn (int/float): Minimum value
        maxn (int/float): Maximum value

    Returns:
        int/float: Clamped value
    """
    return max(min(maxn, n), minn)

if __name__ == "__main__":
    tello_bridge_node = Tello_Bridge_Node()
    rospy.spin()
