import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math

class Bug2Controller(Node):
    def __init__(self):
        super().__init__('bug2_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile_sensor_data)

        # Parameters
        self.dist_thresh_obs = 0.35
        self.wall_following_dist = 0.4
        self.goal_threshold = 0.15
        self.hysteresis_margin = 0.2
        self.dynamic_dist_thresh = self.dist_thresh_obs  # Used for dynamic adjustments

        # Waypoints and control
        self.current_waypoint = 0
        self.waypoints = [(1.5, 0), (1.5, 1.4), (1.4, 0)]
        self.mode = "go to goal"
        self.twist = Twist()

        # PID controller for smoother wall following
        self.kp = 0.5
        self.kd = 0.1
        self.last_error = 0.0

    def odom_callback(self, msg):
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.at_goal():
            self.switch_to_next_waypoint()
        elif self.mode == "go to goal" and self.obstacle_detected():
            self.mode = "wall following mode"
            self.dynamic_dist_thresh = self.dist_thresh_obs * 1.2  # Increase threshold for curves
        elif self.mode == "wall following mode":
            self.wall_following_control()

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        if self.mode == "wall following mode" and self.on_start_goal_line() and not self.obstacle_detected():
            self.mode = "go to goal"
            self.dynamic_dist_thresh = self.dist_thresh_obs  # Reset after wall-following

    def obstacle_detected(self):
        # Detect if obstacles are present within the threshold
        return any(distance < self.dynamic_dist_thresh for distance in self.laser_ranges if not math.isinf(distance))

    def on_start_goal_line(self):
        # Check if bot is on start-goal line based on waypoint slope
        start_x, start_y = self.waypoints[0]
        goal_x, goal_y = self.waypoints[-1]
        current_x, current_y = self.current_pos
        # Basic line equation check for "start-goal line" alignment
        return abs((goal_y - start_y) * (current_x - start_x) - (goal_x - start_x) * (current_y - start_y)) < 0.05

    def at_goal(self):
        # Check if the bot is near the current waypoint
        x, y = self.current_pos
        goal_x, goal_y = self.waypoints[self.current_waypoint]
        return math.sqrt((x - goal_x) ** 2 + (y - goal_y) ** 2) < self.goal_threshold

    def switch_to_next_waypoint(self):
        # Move to the next waypoint in list
        if self.current_waypoint < len(self.waypoints) - 1:
            self.current_waypoint += 1
            self.mode = "go to goal"

    def go_to_goal_control(self):
        goal_x, goal_y = self.waypoints[self.current_waypoint]
        x, y = self.current_pos
        distance = math.sqrt((goal_x - x) ** 2 + (goal_y - y) ** 2)
        angle_to_goal = math.atan2(goal_y - y, goal_x - x)
        self.twist.linear.x = min(0.18, distance)
        self.twist.angular.z = min(2.4, angle_to_goal)
        self.cmd_vel_pub.publish(self.twist)

    def wall_following_control(self):
        # Basic curvature check: adjust dynamic threshold based on detected obstacle shape
        curvature = self.estimate_curvature()
        if curvature > 0.3:  # If curvature is high, adjust wall-following threshold dynamically
            self.dynamic_dist_thresh = max(self.dist_thresh_obs, self.wall_following_dist * 1.1)

        # PID control for smooth wall-following
        error = self.wall_following_dist - min(self.laser_ranges)
        derivative = error - self.last_error
        self.twist.linear.x = 0.15
        self.twist.angular.z = self.kp * error + self.kd * derivative
        self.cmd_vel_pub.publish(self.twist)
        self.last_error = error

    def estimate_curvature(self):
        # Assess the curvature of the wall based on the laser scan readings
        scan_points = [r for r in self.laser_ranges if not math.isinf(r)]
        if len(scan_points) < 2:
            return 0
        curvature_sum = sum(abs(scan_points[i] - scan_points[i + 1]) for i in range(len(scan_points) - 1))
        return curvature_sum / len(scan_points)

def main():
    rclpy.init()
    bug2_controller = Bug2Controller()
    rclpy.spin(bug2_controller)
    bug2_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
