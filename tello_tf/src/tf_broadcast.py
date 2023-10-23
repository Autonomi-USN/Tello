#!/usr/bin/python3
import rospy
from tf import TransformBroadcaster
from decawave_position_reciever.msg import Decawave_position, Decawave_list
from tello_bridge.msg import Tello_data

ORIGIN_FRAME = "decawave/origin"
TAG_PREFIX = "decawave/tag/"
DRONE_TAG = "06B5"

class TfNode:
    def __init__(self):
        rospy.init_node("tello_tf_broadcaster")
        self.decawave_data_sub = rospy.Subscriber("decawave_data", Decawave_list, self.data_cb)
        self.drone_data_sub = rospy.Subscriber("tello_data", Tello_data, self.drone_data_cb)
        self.drone_rot = (0, 0, 0, 1)
        self.data_list = Decawave_list()
        self.br = TransformBroadcaster()
        self.drone_frame = None
        self.pos = None

    def data_cb(self, msg):
        for data in msg.decawave_positions:
            self.drone_frame = TAG_PREFIX + data.name
            self.pos = (data.x, data.y, data.z)
            self.rot = (0, 0, 0, 1)
            if self.drone_frame == TAG_PREFIX + DRONE_TAG:
                self.rot = self.drone_rot
            
            self.br.sendTransform(self.pos,
                self.rot,
                rospy.Time.now(),
                self.drone_frame,
                ORIGIN_FRAME)
            print('---')
    
    def drone_data_cb(self, msg):
        rot = (msg.q1, -msg.q2, -msg.q3, msg.q0)
        if self.pos is None:
            return
        self.br.sendTransform(self.pos,
            rot,
            rospy.Time.now(),
            self.drone_frame,
            ORIGIN_FRAME)




if __name__ == "__main__":
    tf_node = TfNode()
    rospy.spin()