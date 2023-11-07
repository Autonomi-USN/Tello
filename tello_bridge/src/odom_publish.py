#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from tello_bridge.msg import Tello_data
import std_msgs, geometry_msgs

FRAME = "base_link"
ODOM_FRAME = "tello/odom"

class ImuNode:
    def __init__(self):
        rospy.init_node("tello_imu_publisher")
        self.seq = 0
        self.drone_data_sub = rospy.Subscriber("tello_data", Tello_data, self.drone_data_cb)
        self.imu_pub = rospy.Publisher(f"tello/odom", Odometry, queue_size=10)

    def drone_data_cb(self, tello_msg):
        msg = Odometry()
        msg.header = std_msgs.msg.Header(self.seq, rospy.get_rostime(), ODOM_FRAME)
        msg.child_frame_id = FRAME
        msg.pose.pose.position = geometry_msgs.msg.Point(tello_msg.pos_x, tello_msg.pos_y, tello_msg.pos_z)
        msg.twist.twist.linear = geometry_msgs.msg.Vector3(tello_msg.vel_x, tello_msg.vel_y, tello_msg.vel_z)
        self.imu_pub.publish(msg)
        self.seq += 1

if __name__ == "__main__":
    tf_node = ImuNode()
    rospy.spin()