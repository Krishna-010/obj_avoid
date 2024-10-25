import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
import time
from math import atan2, sqrt

class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float64, '/yaw', self.yaw_callback, 10)
        
        # Initialize waypoints
        self.waypoints = [(1.5, 0.0), (1.5, 1.4), (0.0, 1.4)]
        self.current_goal_index = 0
        
        # Movement parameters
        self.linear_velocity = 0.18  # m/s
        self.angular_velocity = 2.4   # rad/s
        self.current_yaw = 0.0

        self.get_logger().info("Go To Goal Node initialized and starting to move to waypoints...")

        self.move_to_goal()

    def yaw_callback(self, yaw_msg):
        """Update the current yaw from the TF node."""
        self.current_yaw = yaw_msg.data

    def move_to_goal(self):
        """Main logic to move to the next waypoint."""
        for waypoint in self.waypoints:
            self.move_straight(waypoint[0], waypoint[1])
            self.turn_left()

    def move_straight(self, goal_x, goal_y):
        """Move straight to the given goal coordinates."""
        current_x, current_y = 0.0, 0.0  # Replace with actual odometry data

        while True:
            # Calculate the distance to the goal
            distance = sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)

            if distance < 0.1:  # If close enough to the waypoint
                self.stop()
                self.get_logger().info(f"Reached waypoint: ({goal_x}, {goal_y})")
                time.sleep(2)  # Wait at the waypoint
                break

            # Create a twist message for forward movement
            twist = Twist()
            twist.linear.x = self.linear_velocity
            twist.angular.z = 0.0  # No rotation while moving straight
            self.publisher.publish(twist)

    def turn_left(self):
        """Turn left 90 degrees."""
        target_yaw = self.current_yaw + 1.5708  # 90 degrees in radians

        while True:
            # Create a twist message for turning
            twist = Twist()
            twist.linear.x = 0.0  # No forward movement
            twist.angular.z = self.angular_velocity  # Rotate counter-clockwise
            self.publisher.publish(twist)

            # Check if the desired yaw is reached
            if abs(self.current_yaw - target_yaw) < 0.1:
                self.stop()
                self.get_logger().info("Turned left 90 degrees")
                time.sleep(1)  # Wait after turning
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
