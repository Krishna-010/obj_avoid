import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry

class TransformationNode(Node):
    def __init__(self):
        super().__init__('transformation_node')
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher_ = self.create_publisher(Point, '/global_position', 10)
        self.get_logger().info("Transformation node initialized.")

    def odom_callback(self, msg):
        global_position = Point()
        global_position.x = msg.pose.pose.position.x
        global_position.y = msg.pose.pose.position.y
        global_position.z = 0.0
        self.publisher_.publish(global_position)
        self.get_logger().info(f"Published global position: {global_position}")

def main(args=None):
    rclpy.init(args=args)
    transformation_node = TransformationNode()
    rclpy.spin(transformation_node)
    transformation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
