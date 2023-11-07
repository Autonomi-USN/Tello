#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Imu
from tello_bridge.msg import Tello_data
import std_msgs, geometry_msgs

FRAME = "imu_link"

class ImuNode:
    def __init__(self):
        rospy.init_node("tello_imu_publisher")
        self.seq = 0
        self.drone_data_sub = rospy.Subscriber("tello_data", Tello_data, self.drone_data_cb)
        self.imu_pub = rospy.Publisher(f"tello/imu/raw", Imu, queue_size=10)

    def drone_data_cb(self, msg):
        rot = (msg.q1, -msg.q2, -msg.q3, msg.q0)
        imu_msg = Imu()
        imu_msg.header = std_msgs.msg.Header(self.seq, rospy.get_rostime(), f"{FRAME}")
        imu_msg.orientation = geometry_msgs.msg.Quaternion(rot[0], rot[1], rot[2], rot[3])
        imu_msg.linear_acceleration = geometry_msgs.msg.Vector3(msg.acc_x, msg.acc_y, msg.acc_z)
        imu_msg.angular_velocity = geometry_msgs.msg.Vector3(msg.gyro_x, msg.gyro_y, msg.gyro_z)
        self.imu_pub.publish(imu_msg)
        self.seq += 1

if __name__ == "__main__":
    tf_node = ImuNode()
    rospy.spin()