import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from math import atan2, sqrt, pi
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        """Calculate the PID output given the error and time delta."""
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float64, '/yaw', self.yaw_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Initialize waypoints
        self.waypoints = [(1.5, 0.0), (1.5, 1.4), (0.0, 1.4)]
        self.current_goal_index = 0

        # Movement parameters
        self.linear_velocity = 0.1  # Reduced to 0.1 m/s for slower movement
        self.angular_velocity = 2.4   # rad/s (you can adjust this if needed)
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        # PID controllers for linear and angular motion
        self.linear_pid = PIDController(kp=1.0, ki=0.0, kd=0.0)  # Tune these parameters
        self.angular_pid = PIDController(kp=3.0, ki=0.0, kd=0.5)  # Tune these parameters

        self.get_logger().info("Go To Goal Node initialized and starting to move to waypoints...")
        self.move_to_goal()

    def yaw_callback(self, yaw_msg):
        """Update the current yaw from the TF node."""
        self.current_yaw = yaw_msg.data

    def odom_callback(self, odom_msg):
        """Update current position from odometry data."""
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y

    def move_to_goal(self):
        """Main logic to move to the next waypoint."""
        while self.current_goal_index < len(self.waypoints):
            goal_x, goal_y = self.waypoints[self.current_goal_index]
            self.navigate_to_goal(goal_x, goal_y)

            # Proceed to the next waypoint after reaching the current one
            self.current_goal_index += 1

    def navigate_to_goal(self, goal_x, goal_y):
        """Navigate to the specific goal coordinates using PID control."""
        while True:
            # Calculate the distance and angle to the goal
            distance = sqrt((goal_x - self.current_x) ** 2 + (goal_y - self.current_y) ** 2)
            target_angle = atan2(goal_y - self.current_y, goal_x - self.current_x)
            angle_error = target_angle - self.current_yaw
            angle_error = (angle_error + pi) % (2 * pi) - pi  # Normalize angle error to [-pi, pi]

            # Calculate PID outputs
            dt = 0.1  # Time delta; you may adjust this based on your loop timing
            angular_velocity = self.angular_pid.update(angle_error, dt)
            linear_velocity = self.linear_pid.update(distance, dt)

            # Create a twist message for motion
            twist = Twist()
            twist.linear.x = max(0.0, min(self.linear_velocity, self.linear_velocity))  # Max speed limit
            twist.angular.z = angular_velocity

            self.publisher.publish(twist)

            # Check if close enough to the waypoint
            if distance < 0.1:  # Adjust threshold as necessary
                self.stop()
                self.get_logger().info(f"Reached waypoint: ({goal_x}, {goal_y})")
                time.sleep(2)  # Wait at the waypoint
                break

    def stop(self):
        """Stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = GoToGoalNode()
    rclpy.spin(go_to_goal_node)
    go_to_goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
