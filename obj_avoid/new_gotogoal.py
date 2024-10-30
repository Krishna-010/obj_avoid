from rclpy.qos import qos_profile_sensor_data
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class Bug2Controller(Node):
    def __init__(self):
        super().__init__('bug2_controller')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Parameters
        self.dist_thresh_obs = 0.21  # Obstacle detection threshold
        self.wall_following_dist = 0.18  # Wall-following distance
        self.goal_threshold = 0.1  # Waypoint goal threshold
        self.hysteresis_margin = 0.5  # Buffer margin for switching back to "go to goal"
        
        # State variables
        self.mode = "go to goal"  # Modes: "go to goal", "wall following"
        self.current_goal_index = 0
        self.waypoints = [(1.5, 0), (1.5, 1.4), (1.4, 0)]
        self.laser_ranges = []
        self.position = (0.0, 0.0)

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        self.process_scan()

    def process_scan(self):
        if self.mode == "wall following" and self.obstacle_detected():
            self.follow_wall()
        elif self.mode == "go to goal" and not self.obstacle_detected():
            self.move_to_goal()

    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.mode == "go to goal":
            if self.reached_goal():
                self.current_goal_index += 1
                if self.current_goal_index >= len(self.waypoints):
                    self.get_logger().info("All goals reached!")
                    self.stop_bot()
                else:
                    self.set_mode("go to goal")  # Move to the next goal
        elif self.mode == "wall following":
            if not self.obstacle_detected():
                self.set_mode("go to goal")  # Switch back to goal mode if no obstacle

    def set_mode(self, mode):
        self.mode = mode
        self.get_logger().info(f"Switched to mode: {self.mode}")

    def obstacle_detected(self):
        return any(distance < self.dist_thresh_obs for distance in self.laser_ranges if not math.isinf(distance))

    def reached_goal(self):
        goal_x, goal_y = self.waypoints[self.current_goal_index]
        return math.sqrt((self.position[0] - goal_x) ** 2 + (self.position[1] - goal_y) ** 2) < self.goal_threshold

    def move_to_goal(self):
        goal_x, goal_y = self.waypoints[self.current_goal_index]
        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = 0.0  # Placeholder for actual angle calculation
        self.cmd_pub.publish(twist)

    def follow_wall(self):
        twist = Twist()
        twist.linear.x = 0.08  # Slower speed while following wall
        twist.angular.z = 0.5  # Turn to follow wall
        self.cmd_pub.publish(twist)

    def stop_bot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    bug2_controller = Bug2Controller()
    rclpy.spin(bug2_controller)
    bug2_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
