import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
import tf_transformations

class TFNode(Node):
    def __init__(self):
        super().__init__('tf')
        self.subscriber = self.create_subscription(
            Quaternion,
            '/quaternion',  # Adjust the topic name as needed
            self.quaternion_callback,
            10
        )
        self.publisher = self.create_publisher(Float64, '/yaw', 10)
        self.get_logger().info("TF Node initialized and waiting for quaternion data...")

    def quaternion_callback(self, quaternion_msg):
        """Convert quaternion to yaw and publish it."""
        quaternion = (
            quaternion_msg.x,
            quaternion_msg.y,
            quaternion_msg.z,
            quaternion_msg.w
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]  # Get the yaw value
        yaw_msg = Float64()
        yaw_msg.data = yaw
        self.publisher.publish(yaw_msg)
        self.get_logger().info(f"Published yaw: {yaw}")

def main(args=None):
    rclpy.init(args=args)
    tf_node = TFNode()
    rclpy.spin(tf_node)
    tf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
