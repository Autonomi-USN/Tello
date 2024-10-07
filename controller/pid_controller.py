import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class PID:
    def __init__(self, kp, ki, kd, dt, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt  # Fixed time interval
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        # Calculate derivative
        derivative = (error - self.prev_error) / self.dt

        # Update integral with anti-windup
        self.integral += error * self.dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        return output

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Parameters
        self.dt = 0.1  # Time interval in seconds

        # Subscribe to the filtered odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10)

        # Publisher to send velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Setpoint positions in ENU frame
        self.setpoint_x = 2.0
        self.setpoint_y = 2.0
        self.setpoint_z = 1.0

        # PID controllers for x, y, z axes
        self.pid_x = PID(kp=0.3, ki=0.05, kd=0.01, dt=self.dt, integral_limit=0.5)
        self.pid_y = PID(kp=0.3, ki=0.05, kd=0.01, dt=self.dt, integral_limit=0.5)
        self.pid_z = PID(kp=0.3, ki=0.05, kd=0.01, dt=self.dt, integral_limit=0.5)

        # Initialize current state variables
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        # Create a timer to run the control loop at fixed intervals
        self.timer = self.create_timer(self.dt, self.control_loop)

    def odometry_callback(self, msg):
        # Extract current position from odometry message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q)

    def control_loop(self):
        # Ensure that we have received the odometry data
        if self.current_x is None or self.current_y is None or self.current_z is None or self.current_yaw is None:
            return  # Skip control loop until we have valid data

        # Compute position errors in ENU frame
        error_x = self.setpoint_x - self.current_x
        error_y = self.setpoint_y - self.current_y
        error_z = self.setpoint_z - self.current_z

        # Compute control outputs using PID controllers
        control_x = self.pid_x_function(error_x)
        control_y = self.pid_y_function(error_y)
        control_z = self.pid_z_function(error_z)

        # Convert control commands from ENU to body frame
        control_body_x, control_body_y = self.enu_to_body_frame(control_x, control_y, self.current_yaw)

        # Create Twist message to publish
        cmd_vel = Twist()
        cmd_vel.linear.x = control_body_x
        cmd_vel.linear.y = control_body_y
        cmd_vel.linear.z = control_z
        cmd_vel.angular.z = 0.0  # No angular control for now

        # Publish velocity command
        self.publisher.publish(cmd_vel)

    def pid_x_function(self, error_x):
        # PID control for x-axis with integral limit
        return self.pid_x.update(error_x)

    def pid_y_function(self, error_y):
        # PID control for y-axis with integral limit
        return self.pid_y.update(error_y)

    def pid_z_function(self, error_z):
        # PID control for z-axis with integral limit
        return self.pid_z.update(error_z)

    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw angle
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def enu_to_body_frame(self, x_enu, y_enu, yaw):
        # Rotate ENU control commands to body frame using yaw angle
        x_body = math.cos(-yaw) * x_enu - math.sin(-yaw) * y_enu
        y_body = math.sin(-yaw) * x_enu + math.cos(-yaw) * y_enu
        return x_body, y_body

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
