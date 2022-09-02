#!/usr/bin/env python
from ast import If
import rospy 
from geometry_msgs.msg import Twist
from djitellopy import Tello
from tello_bridge.msg import Tello_data

class Tello_Bridge_Node:
    def __init__(self):
        rospy.init_node("tello_bridge_node")
        rospy.loginfo("Starting Tello_Bridge_Node.")
        #a: left/right -100 = 100% left, +100 = 100% right
        #b: forward/backward -100 = 100% back, +100 = 100% forward
        #c: up/down -100 = 100% down, +100 = 100% up
        #d: yaw -100 = 100% CCW, +100 = 100% CW
        self.rc = {'a': 0, 'b': 0, 'c' : 0, 'd' : 0}
        self.speed = 50
        self.data_msg = Tello_data()
        self.tello = Tello()
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.pub = rospy.Publisher("tello_data", Tello_data, queue_size=10)
        

        self.tello.connect()
        self.tello.set_speed(self.speed)
        self.tello.streamoff()
        self.tello_get_data()
        self.tello.takeoff()
        rospy.Timer(rospy.Duration(1.0/10.0),self.send_to_tello)

        rospy.spin()

    def callback(self, Twist):
        self.rc['a'] = int(clamp(Twist.linear.y * -self.speed, -100, 100))
        self.rc['b'] = int(clamp(Twist.linear.x * self.speed, -100, 100))
        self.rc['c'] = int(clamp(Twist.linear.z * self.speed, -100, 100))
        self.rc['d'] = int(clamp(Twist.angular.z * -self.speed, -100, 100))

    def send_to_tello(self, event=None):
        self.tello.send_rc_control(self.rc['a'], self.rc['b'], self.rc['c'], self.rc['d'],)
        self.tello_get_data()

    def tello_shutdown_sequence(self):
        print('Attempting to land drone')
        self.tello.end()


    def tello_get_data(self):
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


        
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

if __name__ == "__main__":
    tello_bridge_node = Tello_Bridge_Node()
    rospy.on_shutdown(tello_bridge_node.tello_shutdown_sequence)
    
    rospy.spin()