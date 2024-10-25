import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float64, '/yaw', self.yaw_callback, 10)

        # Waypoints
        self.waypoints = [(1.5, 0), (1.5, 1.4), (1.4, 0)]
        self.current_goal_index = 0

        # Velocity constraints
        self.linear_velocity_constraint = 0.18
        self.angular_velocity_constraint = 2.4

        self.current_yaw = 0.0
        self.odom_reset()  # Reset odometry at the start

        self.get_logger().info("Go to Goal Node initialized and ready to move.")

    def odom_reset(self):
        """Reset the odometry to start at (0, 0)."""
        # Implement your odometry reset logic here if required by your system
        self.get_logger().info("Odometry reset to (0, 0).")

    def yaw_callback(self, yaw_msg):
        """Callback to receive the current yaw value."""
        self.current_yaw = yaw_msg.data
        self.move_to_goal()

    def move_to_goal(self):
        """Move the robot towards the current goal."""
        if self.current_goal_index < len(self.waypoints):
            goal_x, goal_y = self.waypoints[self.current_goal_index]
            vel_msg = Twist()
            # Calculate distance to the goal
            distance_to_goal = ((goal_x ** 2 + goal_y ** 2) ** 0.5)

            # Calculate angle to the goal
            angle_to_goal = np.arctan2(goal_y, goal_x)

            # Control linear and angular velocities
            vel_msg.linear.x = min(self.linear_velocity_constraint, distance_to_goal)
            vel_msg.angular.z = min(self.angular_velocity_constraint, angle_to_goal - self.current_yaw)

            self.publisher.publish(vel_msg)

            self.get_logger().info(f"Moving to goal: ({goal_x}, {goal_y}) with linear.x: {vel_msg.linear.x}, angular.z: {vel_msg.angular.z}")

            # Check if the goal is reached (you may want to add a threshold)
            if distance_to_goal < 0.1:  # Assuming a small threshold for reaching the goal
                self.current_goal_index += 1  # Move to the next goal
                self.get_logger().info("Goal reached, moving to the next waypoint.")

def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = GoToGoal()
    rclpy.spin(go_to_goal_node)
    go_to_goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
