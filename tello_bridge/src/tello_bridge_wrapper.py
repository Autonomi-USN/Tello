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
        self.run_recv_thread = True
        self.new_image = None
        self.current_image = None
        self.flight_data = None
        self.log_data = None
        self.connection_quit = None
        self.landed = True

        self.cvBridge = CvBridge()
        self.tello = tello.Tello()

        self.data_msg = Tello_data()
        self.land_feedback = LandFeedback()
        self.land_result = LandResult()
        self.takeoff_feedback = TakeoffFeedback()
        self.takeoff_result = TakeoffResult()
        
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        self.as_takeoff = actionlib.SimpleActionServer("tello_takeoff", TakeoffAction, execute_cb=self.takeoff_execute_cb, auto_start=False)
        self.as_land = actionlib.SimpleActionServer("tello_land", LandAction, execute_cb=self.land_execute_cb, auto_start=False)
        self.as_takeoff.start()
        rospy.loginfo("Starting takeoff actionserver")
        self.as_land.start()
        rospy.loginfo("Starting land actionserver")

        self.tello_data_pub = rospy.Publisher("tello_data", Tello_data, queue_size=10)
        self.image_pub = rospy.Publisher("tello_image", Image, queue_size=10)
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
        threading.Thread(target=self.recv_thread, args=[self.tello]).start()
        
        

    def takeoff_execute_cb(self, goal):
        rospy.loginfo("Attempting to takeoff drone")
        self.tello.takeoff()
        count = 0
        timer = time.time()
        self.takeoff_result.success = True
        while self.flight_data.em_sky == 0:
            if time.time() - timer > 1:
                timer = time.time()
                count+=1
                self.takeoff_feedback.seconds_taken=count
                self.as_takeoff.publish_feedback(self.takeoff_feedback)
            if count > 20:
                self.takeoff_result.success = False
                rospy.logerr("Failed to takeoff drone")
                break
        if self.takeoff_result.success:
            self.landed = False
            rospy.loginfo("Takeoff successfull")
        self.as_takeoff.set_succeeded(self.takeoff_result)            

    def land_execute_cb(self, goal):
        rospy.loginfo("Attempting to land drone")
        self.tello.land()
        count = 0
        timer = time.time()
        self.land_result.success = True
        while self.flight_data.em_sky != 0:
            if time.time() - timer > 1:
                timer = time.time()
                count+=1
                self.land_feedback.seconds_taken=count
                self.as_land.publish_feedback(self.land_feedback)
            if count > 10:
                self.land_result.success = False
                rospy.logerr("Failed to land drone")
                break
        if self.land_result.success:
            self.landed = True
            rospy.loginfo("Landing successfull")
        self.as_land.set_succeeded(self.land_result)



    def cmd_vel_cb(self, msg):
        if self.flight_data.em_sky != 0:
            self.tello.set_pitch(clamp(msg.linear.x, -1.0, 1.0))
            self.tello.set_roll(clamp(-msg.linear.y, -1.0, 1.0))
            self.tello.set_yaw(clamp(-msg.angular.z, -1.0, 1.0))
            self.tello.set_throttle(clamp(msg.linear.z, -1.0, 1.0))

    def event_handler(self, event, sender, data, **args):
        drone = sender
        if event is drone.EVENT_FLIGHT_DATA:
            self.flight_data = data
        elif event is drone.EVENT_LOG_DATA:
            self.log_data = data
        else:
            print('event="%s" data=%s' % (event.getname(), str(data)))

    def recv_thread(self, drone):
        rospy.loginfo("Tello recieve data thread started")
        print('start recv_thread()')
        try:
            container = av.open(drone.get_video_stream())
            # skip first 30 frames
            frame_skip = 30
            while True:
                for frame in container.decode(video=0):
                    if 0 < frame_skip:
                        frame_skip = frame_skip - 1
                        continue
                    start_time = time.time()
                    image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                    self.frame = image
                    self.image_message = self.cvBridge.cv2_to_imgmsg(self.frame, encoding="passthrough")
                    try:
                        self.image_pub.publish(self.image_message)
                    except CvBridgeError as e:
                        print(e)

                    if self.flight_data:
                        self.tello_flight_data_publish()
                        
                    if self.log_data:
                        self.tello_log_data_publish()
                        
                    self.new_image = image
                    if frame.time_base < 1.0/60:
                        time_base = 1.0/60
                    else:
                        time_base = frame.time_base
                    frame_skip = int((time.time() - start_time)/time_base)
        except Exception as ex:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            print(ex)


    def tello_shutdown_sequence(self):
        if not self.connection_quit:
            print('Attempting to land drone')
            rospy.loginfo("Attempting to land drone")
            self.tello.land()
            while self.flight_data.em_sky != 0:
                continue
        self.run_recv_thread = False
        self.tello.quit()
        


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
    rospy.on_shutdown(tello_bridge_node.tello_shutdown_sequence)
    rospy.spin()
