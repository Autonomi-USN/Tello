#!/usr/bin/env python3

import rclpy
import time

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from tello_interfaces.action import Takeoff, Land
from tello_control.utils import clamp


class TelloControlNode(Node):
    def __init__(self):
        super().__init__('tello_move_drone')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Action clients for takeoff and land
        self.takeoff_client = ActionClient(self, Takeoff, '/tello_takeoff')
        self.land_client    = ActionClient(self, Land,    '/tello_land')

    def send_takeoff(self) -> bool:
        # wait for the server
        self.get_logger().info("Waiting for takeoff server…")
        self.takeoff_client.wait_for_server()

        goal = Takeoff.Goal()
        self.get_logger().info("Sending takeoff request")
        future = self.takeoff_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error("Takeoff goal rejected!")
            return False

        # wait result
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        success = result_future.result().result.success

        if success:
            self.get_logger().info("Takeoff succeeded")
        else:
            self.get_logger().error("Takeoff failed")
        return success
    
    def send_land(self) -> bool:
        self.get_logger().info("Waiting for land server…")
        self.land_client.wait_for_server()

        goal = Land.Goal()
        self.get_logger().info("Sending land request")
        future = self.land_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error("Land goal rejected!")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        success = result_future.result().result.success

        if success:
            self.get_logger().info("Land succeeded")
        else:
            self.get_logger().error("Land failed")
        return success

    def move(self, forward: float, lateral: float, up: float, turn: float, duration: float):

        msg = Twist()
        msg.linear.x = clamp(forward, -0.5, 0.5)
        msg.linear.y = clamp(lateral, -0.5, 0.5)
        msg.linear.z = clamp(up, -0.5, 0.5)
        msg.angular.z = clamp(turn, -0.5, 0.5)


        rate = self.create_rate(1)  # 1 Hz
        end_time = self.get_clock().now().nanoseconds + int(duration * 1e9)

        while rclpy.ok() and self.get_clock().now().nanoseconds < end_time:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=1.0)
        
        self.cmd_pub.publish(Twist())

    def run(self):
        self.get_logger().info("Runing commands")



        if not self.send_takeoff():
            return
        
        """
        Implement your flight logic here by calling move().
        Example (fly in a little square):

            self.move(0.2, 0.0, 0.0, 0.0, 1.5)  # forward
            self.move(0.0, 0.2, 0.0, 0.0, 1.5)  # right
            self.move(-0.2, 0.0, 0.0, 0.0, 1.5) # backward
            self.move(0.0, -0.2, 0.0, 0.0, 1.5) # left
        """
 
        self.send_land()

def main(args=None):
    rclpy.init(args=args)
    node = TelloControlNode()
    node.get_logger().info("Tello control initialized")

    try:
        node.run()
    finally:
        node.cmd_pub.publish(Twist())
        node.send_land()
        node.destroy_node()
        rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()