import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt

class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('go_to_goal_node')
        self.goal_subscriber = self.create_subscription(Point, '/waypoint', self.goal_callback, 10)
        self.position_subscriber = self.create_subscription(Point, '/global_position', self.position_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal = None
        self.current_position = Point()
        self.get_logger().info("Go to goal node initialized.")

    def goal_callback(self, msg):
        self.goal = msg
        self.get_logger().info(f"Current goal: {self.goal}")

    def position_callback(self, msg):
        self.current_position = msg
        if self.goal:
            self.move_to_goal()

    def move_to_goal(self):
        distance = sqrt((self.goal.x - self.current_position.x)**2 + (self.goal.y - self.current_position.y)**2)
        if distance > 0.1:
            angle_to_goal = atan2(self.goal.y - self.current_position.y, self.goal.x - self.current_position.x)
            cmd_msg = Twist()
            cmd_msg.linear.x = min(0.18, distance)
            cmd_msg.angular.z = angle_to_goal
            self.cmd_vel_publisher.publish(cmd_msg)
            self.get_logger().info(f"Moving to goal with linear.x: {cmd_msg.linear.x}, angular.z: {cmd_msg.angular.z}")
        else:
            self.get_logger().info("Goal reached!")

def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = GoToGoalNode()
    rclpy.spin(go_to_goal_node)
    go_to_goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
