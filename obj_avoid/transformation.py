import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

class TransformationNode(Node):
    def __init__(self):
        super().__init__('transformation_node')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.global_position_pub = self.create_publisher(Point, 'global_position', 10)

    def odom_callback(self, odom_msg):
        # Extract position from Odometry message
        position = odom_msg.pose.pose.position
        global_position = Point(x=position.x, y=position.y, z=position.z)
        self.global_position_pub.publish(global_position)
        self.get_logger().info(f'Published global position: {global_position}')

def main(args=None):
    rclpy.init(args=args)
    transformation_node = TransformationNode()
    rclpy.spin(transformation_node)
    transformation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
