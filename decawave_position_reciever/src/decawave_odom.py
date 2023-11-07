#!/usr/bin/python3

#! This node only works with one decawave tag/drone active at a time.

import rospy
from tf import TransformBroadcaster
from nav_msgs.msg import Odometry
from decawave_position_reciever.msg import Decawave_position, Decawave_list
from tello_bridge.msg import Tello_data

ORIGIN_FRAME = "decawave/odom"
TAG_PREFIX = "decawave/tag/"
DRONE_TAG = "5C14"

class TfNode:
    def __init__(self):
        rospy.init_node("decawave_odom_publisher")
        
        self.odometry_pub = rospy.Publisher(f"{TAG_PREFIX}{DRONE_TAG}/odometry", Odometry, queue_size=10)
        self.decawave_data_sub = rospy.Subscriber("decawave_data", Decawave_list, self.beacon_data_cb)

        self.drone_rot = (0, 0, 0, 1)
        self.data_list = Decawave_list()
        self.odom_seq = 0

    def beacon_data_cb(self, msg):
        for data in msg.decawave_positions:
            odom_msg = Odometry()
            odom_msg.header.seq = self.odom_seq
            self.odom_seq += 1
            odom_msg.header.stamp = rospy.get_rostime()
            odom_msg.header.frame_id = ORIGIN_FRAME
            odom_msg.child_frame_id = TAG_PREFIX + DRONE_TAG
            odom_msg.pose.pose.position.x = data.x
            odom_msg.pose.pose.position.y = data.y
            odom_msg.pose.pose.position.z = data.z
            self.odometry_pub.publish(odom_msg)
            rospy.loginfo(f"published tf and odometry ({data.name})")



if __name__ == "__main__":
    tf_node = TfNode()
    rospy.spin()