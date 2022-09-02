#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from djitellopy import Tello
from geometry_msgs.msg import Twist
from tello_bridge.msg import Tello_data
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Tello_Bridge_Node:
    def __init__(self):
        rospy.init_node("tello_bridge_node")
        rospy.loginfo("Starting Tello_Bridge_Node.")

        self.cvBridge = CvBridge()
        self.tello = Tello()

        #a: left/right -100 = 100% left, +100 = 100% right
        #b: forward/backward -100 = 100% back, +100 = 100% forward
        #c: up/down -100 = 100% down, +100 = 100% up
        #d: yaw -100 = 100% CCW, +100 = 100% CW
        self.rc = {'a': 0, 'b': 0, 'c' : 0, 'd' : 0}
        self.speed = 50
        self.data_msg = Tello_data()
        
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.pub = rospy.Publisher("tello_data", Tello_data, queue_size=10)
        self.image_pub = rospy.Publisher("image_topic_tello", Image)
        self.command_frequency = 1.0/10.0

        self.tello.connect()
        self.tello.set_speed(self.speed)
        self.tello.streamoff()
        self.tello.set_video_resolution(self.tello.RESOLUTION_480P)
        self.tello_get_data()
        self.tello.streamon()
        self.frame_read = self.tello.get_frame_read()
        #self.tello.takeoff()
        self.frame = self.frame_read.frame
        #Calls communication 
        rospy.Timer(rospy.Duration(self.command_frequency),self.send_to_tello)

        rospy.spin()

    def callback(self, Twist):
        self.rc['a'] = int(clamp(Twist.linear.y * -self.speed, -100, 100))
        self.rc['b'] = int(clamp(Twist.linear.x * self.speed, -100, 100))
        self.rc['c'] = int(clamp(Twist.linear.z * self.speed, -100, 100))
        self.rc['d'] = int(clamp(Twist.angular.z * -self.speed, -100, 100))

    def send_to_tello(self, event=None):
        """
        Sends remote controll data to the drone

        Args:
            event (_type_, optional): _description_. Defaults to None.
        """
        #self.tello.send_rc_control(self.rc['a'], self.rc['b'], self.rc['c'], self.rc['d'],)
        self.tello_get_data()
        self.tello_get_video()

    def tello_shutdown_sequence(self):
        print('Attempting to land drone')
        self.tello.end()


    def tello_get_data(self):
        """
        Gets data from the drone and publishes it on the tello_data topic
        """
        self.data_msg.pitch = int(self.tello.get_pitch())
        self.data_msg.roll = int(self.tello.get_roll())
        self.data_msg.yaw = int(self.tello.get_yaw())
        self.data_msg.speed_x = int(self.tello.get_speed_x())
        self.data_msg.speed_y = int(self.tello.get_speed_y())
        self.data_msg.speed_z = int(self.tello.get_speed_z())
        self.data_msg.acceleration_x = float(self.tello.get_acceleration_x())
        self.data_msg.acceleration_y = float(self.tello.get_acceleration_y())
        self.data_msg.acceleration_z = float(self.tello.get_acceleration_z())
        self.data_msg.rel_height = int(self.tello.get_height())
        self.data_msg.abs_height = int(self.tello.get_barometer())
        self.data_msg.battery = int(self.tello.get_battery())
        self.pub.publish(self.data_msg)

    def tello_get_video(self):
        self.frame = self.frame_read.frame
        #self.frame = cv2.resize(self.frame, (320,240))
        self.frame = self.frame[0:480, 0:640]
        #self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("result", self.frame)
        #cv2.waitKey(1)
        #self.frame = cv2.cvtColor(self.frame, cv2.COLORBGR2RGB)
        #self.frame = np.rot90(self.frame)
        #self.frame = np.flipud(self.frame)
        self.image_message = self.cvBridge.cv2_to_imgmsg(self.frame, encoding="passthrough")
        try:
            self.image_pub.publish(self.image_message)
        except CvBridgeError as e:
            print(e)




        
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
    rospy.on_shutdown(tello_bridge_node.tello_shutdown_sequence)
    
    rospy.spin()
