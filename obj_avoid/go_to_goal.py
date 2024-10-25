import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('go_to_goal_node')
        self.waypoint_sub = self.create_subscription(Point, 'waypoint', self.waypoint_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_goal = None

    def waypoint_callback(self, waypoint):
        self.current_goal = waypoint
        self.get_logger().info(f'Current goal: {self.current_goal}')

    def move_to_goal(self):
        if self.current_goal:
            vel_msg = Twist()
            distance = self.euclidean_distance((0, 0), (self.current_goal.x, self.current_goal.y))
            angle_to_goal = np.arctan2(self.current_goal.y, self.current_goal.x)

            vel_msg.linear.x = min(0.18, distance)
            vel_msg.angular.z = min(2.4, angle_to_goal)

            self.cmd_pub.publish(vel_msg)

    def euclidean_distance(self, p1, p2):
        """Returns the Euclidean distance between two points."""
        return np.linalg.norm(np.array(p1) - np.array(p2))

def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = GoToGoalNode()
    rclpy.spin(go_to_goal_node)
    go_to_goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
