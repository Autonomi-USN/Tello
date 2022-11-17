#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from tello_bridge.msg import Tello_data
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tellopy import tello
import time
import sys
from subprocess import Popen, PIPE
import threading
import av
import traceback

#TODO: Subscriber for land and takeoff, implement cmd_vel in new library

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

        self.cvBridge = CvBridge()
        self.tello = tello.Tello()

        self.data_msg = Tello_data()
        
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.tello_data_pub = rospy.Publisher("tello_data", Tello_data, queue_size=10)
        self.image_pub = rospy.Publisher("image_tello", Image, queue_size=10)
        self.command_frequency = 1.0/10.0
        self.tello.connect()
        self.tello.wait_for_connection(60)
        self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self.event_handler)
        self.tello.subscribe(self.tello.EVENT_LOG_DATA, self.event_handler)
                
        
        threading.Thread(target=self.recv_thread, args=[self.tello]).start()
        
        self.tello.takeoff()


    def cmd_vel_callback(self, msg):
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
        print('Attempting to land drone')
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
        self.tello_data_pub.publish(self.data_msg)

    def tello_flight_data_publish(self):
        self.data_msg.battery_percentage = self.flight_data.battery_percentage
        self.data_msg.east_speed = self.flight_data.east_speed/10.0
        self.data_msg.north_speed = self.flight_data.north_speed/10.0
        self.data_msg.ground_speed = self.flight_data.ground_speed
        self.data_msg.height = self.flight_data.height/10.0
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
