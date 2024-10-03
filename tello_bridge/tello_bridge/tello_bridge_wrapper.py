#!/usr/bin/env python3
from typing import Any
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import threading
import av
import traceback
import time
import sys
from tello_bridge.tellopy.event import Event
from subprocess import Popen, PIPE
import tello_bridge.tellopy.tello as tello
import tello_bridge.tellopy.error as error
from rclpy.action import ActionServer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from tello_interfaces.msg import TelloData
from tello_interfaces.action import Takeoff, Land

class TelloBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('tello_bridge_node')
        self.get_logger().info("Starting TelloBridgeNode.")

        self.prev_flight_data = None
        self.run_recv_thread = True
        self.new_image = None
        self.current_image = None
        self.flight_data = None
        self.log_data = None
        self.connection_quit = None
        self.landed = True
        self.handle_img = False

        self.cvBridge = CvBridge()
        self.tello = tello.Tello()

        self.data_msg = TelloData()
        self.land_feedback = Land.Feedback()
        self.land_result = Land.Result()
        self.takeoff_feedback = Takeoff.Feedback()
        self.takeoff_result = Takeoff.Result()

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.as_takeoff = ActionServer(self, Takeoff, '/tello_takeoff', self.takeoff_execute_cb)
        self.as_land = ActionServer(self, Land, '/tello_land', self.land_execute_cb)
        self.get_logger().info("Takeoff and land action servers started")

        self.tello_data_pub = self.create_publisher(TelloData, 'tello_data', 10)
        self.image_pub = self.create_publisher(Image, 'tello_image', 10)
        
        self.tello_connect()
        self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self.event_handler)
        self.tello.subscribe(self.tello.EVENT_LOG_DATA, self.event_handler)
        threading.Thread(target=self.recv_thread, args=[self.tello]).start()

    def tello_connect(self) -> None:
        self.connection_quit = True
        self.tello.connect()
        self.get_logger().info("Connecting to Tello drone")
        try:
            self.tello.wait_for_connection(10)
        except error.TelloError as err:
            self.get_logger().error(str(err))
            self.tello.quit()
            self.destroy_node()
            return
        self.get_logger().info("Connected to Tello drone")
        self.connection_quit = False
        

    def takeoff_execute_cb(self, goal_handle: Takeoff) -> None:
        self.get_logger().info("Attempting to takeoff drone")
        self.tello.takeoff()
        count = 0
        timer = time.time()
        self.takeoff_result.success = True
        while self.flight_data.em_sky == 0:
            if time.time() - timer > 1:
                timer = time.time()
                count += 1
                self.takeoff_feedback.seconds_taken = count
                goal_handle.publish_feedback(self.takeoff_feedback)
            if count > 20:
                self.takeoff_result.success = False
                self.get_logger().error("Failed to takeoff drone")
                break
        if self.takeoff_result.success:
            self.landed = False
            self.get_logger().info("Takeoff successful")
        goal_handle.succeed()
        goal_handle.set_result(self.takeoff_result)

    def land_execute_cb(self, goal_handle: Land) -> None:
        self.get_logger().info("Attempting to land drone")
        self.tello.land()
        count = 0
        timer = time.time()
        self.land_result.success = True
        while self.flight_data.em_sky != 0:
            if time.time() - timer > 1:
                timer = time.time()
                count += 1
                self.land_feedback.seconds_taken = count
                goal_handle.publish_feedback(self.land_feedback)
            if count > 10:
                self.land_result.success = False
                self.get_logger().error("Failed to land drone")
                break
        if self.land_result.success:
            self.landed = True
            self.get_logger().info("Landing successful")
        goal_handle.succeed()
        goal_handle.set_result(self.land_result)

    def cmd_vel_cb(self, msg: Twist) -> None:
        if self.flight_data.em_sky != 0:
            self.tello.set_pitch(self.clamp(msg.linear.x, -1.0, 1.0))
            self.tello.set_roll(self.clamp(-msg.linear.y, -1.0, 1.0))
            self.tello.set_yaw(self.clamp(-msg.angular.z, -1.0, 1.0))
            self.tello.set_throttle(self.clamp(msg.linear.z, -1.0, 1.0))

    def event_handler(self, event: Event, sender: tello.Tello, data: Any, **args) -> None:
        drone = sender
        if event is drone.EVENT_FLIGHT_DATA:
            self.flight_data = data
        elif event is drone.EVENT_LOG_DATA:
            self.log_data = data
        else:
            print(f'event="{event.getname()}" data={str(data)}')

    def recv_thread(self, drone: tello.Tello) -> None:
        self.get_logger().info("Tello receive data thread started")
        if self.handle_img:
            try:
                container = av.open(drone.get_video_stream())
                frame_skip = 30
                while True:
                    for frame in container.decode(video=0):
                        if frame_skip > 0:
                            frame_skip -= 1
                            continue
                        start_time = time.time()
                        image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                        self.frame = image
                        self.image_message = self.cvBridge.cv2_to_imgmsg(self.frame, encoding="passthrough")
                        try:
                            self.image_pub.publish(self.image_message)
                        except CvBridgeError as e:
                            self.get_logger().error(str(e))

                        if self.flight_data:
                            self.tello_flight_data_publish()

                        if self.log_data:
                            self.tello_log_data_publish()

                        self.new_image = image
                        time_base = frame.time_base if frame.time_base >= 1.0/60 else 1.0/60
                        frame_skip = int((time.time() - start_time) / time_base)
            except Exception as ex:
                exc_type, exc_value, exc_traceback = sys.exc_info()
                traceback.print_exception(exc_type, exc_value, exc_traceback)
                print(ex)
        else:
            try:
                while True:
                    start_time = time.time()
                    if self.flight_data:
                        self.tello_flight_data_publish()

                    if self.log_data:
                        self.tello_log_data_publish()
            except Exception as ex:
                exc_type, exc_value, exc_traceback = sys.exc_info()
                traceback.print_exception(exc_type, exc_value, exc_traceback)
                print(ex)

    def tello_shutdown_sequence(self) -> None:
        if not self.connection_quit:
            self.get_logger().info("Attempting to land drone")
            self.tello.land()
            while self.flight_data.em_sky != 0:
                continue
        self.run_recv_thread = False
        self.tello.quit()

    def tello_log_data_publish(self) -> None:
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

    def tello_flight_data_publish(self) -> None:
        self.data_msg.battery_percentage = self.flight_data.battery_percentage
        self.data_msg.east_speed = self.flight_data.east_speed / 10.0
        self.data_msg.north_speed = self.flight_data.north_speed / 10.0
        self.data_msg.ground_speed = float(self.flight_data.ground_speed)
        self.data_msg.height = self.flight_data.height / 10.0
        self.data_msg.landed = self.landed
        self.tello_data_pub.publish(self.data_msg)
        
    @staticmethod    
    def clamp(n: float, minn: float, maxn: float) -> None:
        return max(min(maxn, n), minn)
    

def main(args=None):
    rclpy.init(args=args)
    tello_bridge_node = TelloBridgeNode()
    rclpy.spin(tello_bridge_node)
    tello_bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
