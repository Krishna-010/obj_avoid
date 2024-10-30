import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from std_msgs.msg import Bool

class Bug2Controller(Node):
    def __init__(self):
        super().__init__('bug_2_with_waypoints')

        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Point,
            '/fixed_odom', 
            self.odom_callback,
            10)
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.has_obstacle_sub = self.create_subscription(
            Bool, 
            '/has_obstacle', 
            self.obstacle_callback,
            10)

        # Publisher
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize variables
        self.current_x, self.current_y, self.current_yaw = 0.0, 0.0, 0.0
        self.goal_x, self.goal_y = None, None
        self.start_goal_line_calculated = False
        self.start_goal_slope = None
        self.start_goal_intercept = None
        self.waypoints = [(1.5, 0.0), (1.5, 1.4), (0.0, 1.4)]
        self.current_waypoint_index = 0
        self.has_obstacle = False
        self.robot_mode = "go to goal mode"
        
        # Parameters
        self.dist_thresh_obs = 0.3  # Obstacle detection threshold
        self.wall_following_dist = 0.4  # Wall-following distance
        self.goal_threshold = 0.1  # Waypoint goal threshold
        self.hysteresis_margin = 0.1  # Buffer margin for switching back to "go to goal"

    def odom_callback(self, msg):
        self.current_x, self.current_y, self.current_yaw = msg.x, msg.y, msg.z
        if not self.start_goal_line_calculated:
            self.calculate_start_goal_line()

    def calculate_start_goal_line(self):
        self.start_x, self.start_y = self.current_x, self.current_y
        self.goal_x, self.goal_y = self.waypoints[self.current_waypoint_index]
        if self.goal_x != self.start_x:
            self.start_goal_slope = (self.goal_y - self.start_y) / (self.goal_x - self.start_x)
            self.start_goal_intercept = self.start_y - self.start_goal_slope * self.start_x
        else:
            self.start_goal_slope = float('inf')
            self.start_goal_intercept = None
        self.start_goal_line_calculated = True

    def on_start_goal_line(self):
        if self.start_goal_slope is None:
            return False
        if self.start_goal_slope == float('inf'):
            return abs(self.current_x - self.start_x) < 0.05
        else:
            expected_y = self.start_goal_slope * self.current_x + self.start_goal_intercept
            return abs(self.current_y - expected_y) < 0.05

    def scan_callback(self, msg):
        scan_ranges = np.array(msg.ranges)
        scan_ranges = np.where(np.isnan(scan_ranges), float('inf'), scan_ranges)

        self.front_dist = scan_ranges[int(0/360 * len(msg.ranges))]
        self.rightfront_dist = scan_ranges[int(315/360 * len(msg.ranges))]
        self.leftfront_dist = scan_ranges[int(45/360 * len(msg.ranges))]

        if self.robot_mode == "go to goal mode" and self.obstacle_detected():
            self.robot_mode = "wall following mode"

        elif self.robot_mode == "wall following mode" and not self.obstacle_detected() and self.on_start_goal_line():
            self.robot_mode = "go to goal mode"

        if self.robot_mode == "go to goal mode":
            self.go_to_goal()
        elif self.robot_mode == "wall following mode":
            self.follow_wall()

    def obstacle_callback(self, msg):
        self.has_obstacle = msg.data

    def obstacle_detected(self):
        return self.has_obstacle or (self.front_dist < self.dist_thresh_obs or self.rightfront_dist < self.wall_following_dist or self.leftfront_dist < self.wall_following_dist)

    def go_to_goal(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached!")
            self.stop_robot()
            return

        self.goal_x, self.goal_y = self.waypoints[self.current_waypoint_index]
        distance_to_goal = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        yaw_error = angle_to_goal - self.current_yaw
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        msg = Twist()
        if distance_to_goal > self.goal_threshold:
            if abs(yaw_error) > 0.1:
                msg.angular.z = 1.0 if yaw_error > 0 else -1.0
            else:
                msg.linear.x = 0.1
        else:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached.")
            self.current_waypoint_index += 1
            self.start_goal_line_calculated = False

        self.velocity_publisher.publish(msg)

    def follow_wall(self):
        msg = Twist()

        if self.front_dist < self.wall_following_dist:
            msg.angular.z = 1.0
        elif self.rightfront_dist < self.wall_following_dist:
            if self.rightfront_dist < self.dist_thresh_obs - self.hysteresis_margin:
                msg.angular.z = 1.0
            else:
                msg.linear.x = 0.1
        else:
            msg.angular.z = -1.0

        self.velocity_publisher.publish(msg)

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    bug2_controller = Bug2Controller()
    rclpy.spin(bug2_controller)
    bug2_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
