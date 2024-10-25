import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class RangeDetectionNode(Node):
    def __init__(self):
        super().__init__('range_detection_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        self.obstacle_detected = False

    def scan_callback(self, scan_msg):
        """Handle incoming LaserScan data to detect obstacles."""
        for distance in scan_msg.ranges:
            if scan_msg.range_min < distance < scan_msg.range_max:
                self.obstacle_detected = True
                break
        else:
            self.obstacle_detected = False

        self.get_logger().info(f'Obstacle detected: {self.obstacle_detected}')

def main(args=None):
    rclpy.init(args=args)
    range_detection_node = RangeDetectionNode()
    rclpy.spin(range_detection_node)
    range_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
