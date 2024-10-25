import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class TFNode(Node):
    def __init__(self):
        super().__init__('tf')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )
        
        self.publisher = self.create_publisher(float, '/yaw', 10)
        self.get_logger().info("TF Node initialized and waiting for odometry data...")

    def odom_callback(self, odom_msg):
        """Callback to handle incoming odometry data and convert quaternion to yaw."""
        orientation_q = odom_msg.pose.pose.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]  # Yaw is the third element in the euler angles
        
        self.publisher.publish(yaw)
        self.get_logger().info(f"Published Yaw: {yaw}")

def main(args=None):
    rclpy.init(args=args)
    tf_node = TFNode()
    rclpy.spin(tf_node)
    tf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
