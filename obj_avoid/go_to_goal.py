import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.waypoints = [(1.5, 0), (1.5, 1.4), (0, 1.4)]  # waypoints
        self.current_waypoint_index = 0
        self.goal = Point()
        self.goal.x, self.goal.y = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f'Current waypoint: {self.goal.x}, {self.goal.y}')

        self.timer = self.create_timer(0.1, self.move_to_goal)  # Timer to call move_to_goal every 0.1 seconds

    def odom_callback(self, msg):
        # Update the current position and orientation of JARVIS
        self.current_pos = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation

    def move_to_goal(self):
        if self.current_waypoint_index < len(self.waypoints):
            # Calculate the distance to the goal
            distance = math.sqrt((self.goal.x - self.current_pos.x) ** 2 + (self.goal.y - self.current_pos.y) ** 2)

            if distance < 0.1:  # Close enough to the goal
                self.get_logger().info(f'Reached waypoint: {self.goal.x}, {self.goal.y}')
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.waypoints):
                    self.goal.x, self.goal.y = self.waypoints[self.current_waypoint_index]
                    self.get_logger().info(f'Moving to next waypoint: {self.goal.x}, {self.goal.y}')
                else:
                    self.get_logger().info('All waypoints reached.')
                    return

            # Calculate the angle to the goal
            goal_angle = math.atan2(self.goal.y - self.current_pos.y, self.goal.x - self.current_pos.x)
            current_angle = self.get_yaw_from_quaternion(self.current_orientation)

            # PID control for velocity and angular velocity
            linear_vel = min(0.18, distance)  # Limit linear velocity
            angular_vel = self.pid_control_angle(goal_angle, current_angle)

            # Publish velocity commands
            cmd_vel = Twist()
            cmd_vel.linear.x = linear_vel
            cmd_vel.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd_vel)

            self.get_logger().info(f'Published cmd_vel: linear={cmd_vel.linear.x}, angular={cmd_vel.angular.z}')

    def get_yaw_from_quaternion(self, quat):
        # Convert quaternion to yaw
        return math.atan2(2.0 * (quat.z * quat.w + quat.x * quat.y), 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))

    def pid_control_angle(self, goal_angle, current_angle):
        # Simple P controller for angular velocity
        angle_diff = goal_angle - current_angle
        # Normalize angle_diff to be within -pi to pi
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        return max(-2.4, min(2.4, angle_diff * 1.0))  # Limit to max angular velocity

def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = GoToGoal()
    rclpy.spin(go_to_goal_node)
    go_to_goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
