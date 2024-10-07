#!/usr/bin/env python3
from typing import Any, Callable, Union
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
import tello_bridge.tellopy.tello as tello
import tello_bridge.tellopy.error as error
from rclpy.action import ActionServer
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tello_interfaces.msg import TelloData
from tello_interfaces.action import Takeoff, Land

class TelloBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('tello_bridge_node')
        self.get_logger().info("Starting TelloBridgeNode.")

        self.initialize_variables()
        
        self.setup_action_servers()
        self.setup_subscribers()
        self.setup_publishers()
        
        self.tello_connect()

    # ------ Initializing the system ------ #
    
    def initialize_variables(self) -> None:
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


    def setup_publishers(self) -> None:
        self.tello_data_pub = self.create_publisher(TelloData, '/tello_data', 10)
        self.tello_imu_pub = self.create_publisher(Imu, '/tello_imu', 10)
        self.tello_odom_pub = self.create_publisher(Odometry, '/tello_odom', 10)
        self.image_pub = self.create_publisher(Image, '/tello_image', 10)

    def setup_subscribers(self) -> None:
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

    def setup_action_servers(self) -> None:
        self.as_takeoff = ActionServer(self, Takeoff, '/tello_takeoff', self.takeoff_execute_cb)
        self.as_land = ActionServer(self, Land, '/tello_land', self.land_execute_cb)
        self.get_logger().info("Takeoff and land action servers started")


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
        
        self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self.event_handler)
        self.tello.subscribe(self.tello.EVENT_LOG_DATA, self.event_handler)
        threading.Thread(target=self.recv_thread, args=[self.tello]).start()
        
    # ------ Action Server Handling ------ #
    
    def takeoff_execute_cb(self, goal_handle: Takeoff.Goal) -> Takeoff.Result:
        self.get_logger().info("Attempting to takeoff drone")
        return self.execute_drone_action(self.tello.takeoff, goal_handle, "Takeoff", 20)

    def land_execute_cb(self, goal_handle: Land.Goal) -> Land.Result:
        self.get_logger().info("Attempting to land drone")
        return self.execute_drone_action(self.tello.land, goal_handle, "Landing", 10)


    def execute_drone_action(self, action_func: Callable, goal_handle: Union[Takeoff.Goal, Land.Goal], action_name: str, timeout: int) -> Union[Takeoff.Result, Land.Result]:
        action_func()
        feedback, result = (self.takeoff_feedback, self.takeoff_result) if action_name == "Takeoff" else (self.land_feedback, self.land_result)

        # Initialize result correctly
        result.success = True  # Assuming this is a boolean field

        count = 0
        timer = time.time()
        while self.flight_data.em_sky == (0 if action_name == "Takeoff" else 1):
            if time.time() - timer > 1:
                timer = time.time()
                count += 1
                feedback.seconds_taken = count
                goal_handle.publish_feedback(feedback)
            if count > timeout:
                result.success = False
                self.get_logger().error(f"Failed to {action_name.lower()} drone")
                break

        if result.success:
            self.landed = (action_name == "Landing")
            self.get_logger().info(f"{action_name} successful")
        
        # Mark the goal as succeeded
        goal_handle.succeed()

        # Return the result
        return result


    # ------ Subscriber Callback Handling ------ #

    def cmd_vel_cb(self, msg: Twist) -> None:
        if self.flight_data.em_sky != 0:
            self.tello.set_pitch(self.clamp(msg.linear.x, -1.0, 1.0))
            self.tello.set_roll(self.clamp(-msg.linear.y, -1.0, 1.0))
            self.tello.set_yaw(self.clamp(-msg.angular.z, -1.0, 1.0))
            self.tello.set_throttle(self.clamp(msg.linear.z, -1.0, 1.0))

    # ------ Tello Data Handling ------ #

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
            self.handle_video_stream(drone)
        else:
            self.process_flight_data_loop()

    def handle_video_stream(self, drone: tello.Tello) -> None:
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
                    self.publish_image(image)
                    self.new_image = image
                    frame_skip = int((time.time() - start_time) / max(frame.time_base, 1.0/60))
        except Exception as ex:
            self.handle_exception(ex)

    def process_flight_data_loop(self) -> None:
        try:
            while True:
                self.process_flight_data()
        except Exception as ex:
            self.handle_exception(ex)

    def process_flight_data(self) -> None:
        if self.flight_data:
            self.tello_flight_data_publish()

        if self.log_data:
            self.tello_log_data_publish()
            self.tello_imu_publish()
            self.tello_odom_publish()

    def shutdown(self) -> None:
        self.run_recv_thread = False
        self.get_logger().info("Shutting down TelloBridgeNode")
        self.tello.quit()
        rclpy.shutdown()

    def handle_exception(self, exception: Exception) -> None:
        self.get_logger().error(f"Exception occurred: {str(exception)}")
        self.get_logger().error(traceback.format_exc())

    # ------ Publisher Functions ------ #

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

    def tello_imu_publish(self) -> None:
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()

        imu_msg.orientation.x = self.log_data.imu.q0
        imu_msg.orientation.y = self.log_data.imu.q1
        imu_msg.orientation.z = self.log_data.imu.q2
        imu_msg.orientation.w = self.log_data.imu.q3

        imu_msg.angular_velocity.x = self.log_data.imu.gyro_x
        imu_msg.angular_velocity.y = self.log_data.imu.gyro_y
        imu_msg.angular_velocity.z = self.log_data.imu.gyro_z

        imu_msg.linear_acceleration.x = self.log_data.imu.acc_x
        imu_msg.linear_acceleration.y = self.log_data.imu.acc_x
        imu_msg.linear_acceleration.z = self.log_data.imu.acc_x

        self.tello_imu_pub.publish(imu_msg)

    def tello_odom_publish(self) -> None:
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        
        odom_msg.pose.pose.position.x = self.log_data.mvo.pos_x
        odom_msg.pose.pose.position.y = self.log_data.mvo.pos_y
        odom_msg.pose.pose.position.z = self.log_data.mvo.pos_z

        odom_msg.pose.pose.orientation.x = self.log_data.imu.q0
        odom_msg.pose.pose.orientation.y = self.log_data.imu.q1
        odom_msg.pose.pose.orientation.z = self.log_data.imu.q2
        odom_msg.pose.pose.orientation.w = self.log_data.imu.q3

        odom_msg.twist.twist.linear.x = self.log_data.mvo.pos_x
        odom_msg.twist.twist.linear.y = self.log_data.mvo.pos_y
        odom_msg.twist.twist.linear.z = self.log_data.mvo.pos_z

        odom_msg.twist.twist.angular.x = self.log_data.imu.gyro_x
        odom_msg.twist.twist.angular.y = self.log_data.imu.gyro_y
        odom_msg.twist.twist.angular.z = self.log_data.imu.gyro_z
        
        self.tello_odom_pub.publish(odom_msg)

    def tello_flight_data_publish(self) -> None:
        self.data_msg.battery_percentage = self.flight_data.battery_percentage
        self.data_msg.east_speed = self.flight_data.east_speed / 10.0
        self.data_msg.north_speed = self.flight_data.north_speed / 10.0
        self.data_msg.ground_speed = float(self.flight_data.ground_speed)
        self.data_msg.height = self.flight_data.height / 10.0
        self.data_msg.landed = self.landed
        self.tello_data_pub.publish(self.data_msg)

    def publish_image(self, image: np.ndarray) -> None:
        try:
            img_msg = self.cvBridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.image_pub.publish(img_msg)
        except CvBridgeError as err:
            self.get_logger().error(str(err))
        
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
