import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.yaw_sub = self.create_subscription(Float64, '/yaw', self.yaw_callback, 10)

        # Waypoints initialization
        self.waypoints = [(1.5, 0), (1.5, 1.4), (1.4, 0)]
        self.current_waypoint_index = 0
        self.velocity_constraints = {'linear': 0.18, 'angular': 2.4}
        self.odom_position = (0.0, 0.0)

        self.get_logger().info("Go To Goal Node initialized, moving to waypoints...")

    def yaw_callback(self, msg):
        self.current_yaw = msg.data
        self.move_to_next_waypoint()

    def move_to_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            target = self.waypoints[self.current_waypoint_index]
            distance = math.sqrt((target[0] - self.odom_position[0]) ** 2 + (target[1] - self.odom_position[1]) ** 2)
            import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.yaw_sub = self.create_subscription(Float64, '/yaw', self.yaw_callback, 10)

        # Waypoints initialization
        self.waypoints = [(1.5, 0), (1.5, 1.4), (1.4, 0)]
        self.current_waypoint_index = 0
        self.velocity_constraints = {'linear': 0.18, 'angular': 2.4}
        self.odom_position = (0.0, 0.0)

        self.get_logger().info("Go To Goal Node initialized, moving to waypoints...")

    def yaw_callback(self, msg):
        self.current_yaw = msg.data
        self.move_to_next_waypoint()

    def move_to_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            target = self.waypoints[self.current_waypoint_index]
            distance = math.sqrt((target[0] - self.odom_position[0]) ** 2 + (target[1] - self.odom_position[1]) ** 2)
            
            if distance < 0.1:  # Considered reached if within 10cm
                self.current_waypoint_index += 1
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}")
            else:
                vel_msg = Twist()
                vel_msg.linear.x = min(self.velocity_constraints['linear'], distance)
                vel_msg.angular.z = self.calculate_yaw_correction(target)
                self.cmd_pub.publish(vel_msg)

    def calculate_yaw_correction(self, target):
        target_yaw = math.atan2(target[1] - self.odom_position[1], target[0] - self.odom_position[0])
        yaw_error = target_yaw - self.current_yaw
        return min(self.velocity_constraints['angular'], yaw_error)

def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = GoToGoalNode()
    rclpy.spin(go_to_goal_node)
    go_to_goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

            if distance < 0.1:  # Considered reached if within 10cm
                self.current_waypoint_index += 1
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}")
            else:
                vel_msg = Twist()
                vel_msg.linear.x = min(self.velocity_constraints['linear'], distance)
                vel_msg.angular.z = self.calculate_yaw_correction(target)
                self.cmd_pub.publish(vel_msg)

    def calculate_yaw_correction(self, target):
        target_yaw = math.atan2(target[1] - self.odom_position[1], target[0] - self.odom_position[0])
        yaw_error = target_yaw - self.current_yaw
        return min(self.velocity_constraints['angular'], yaw_error)

def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = GoToGoalNode()
    rclpy.spin(go_to_goal_node)
    go_to_goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
