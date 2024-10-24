import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class RangeDetection(Node):
    def __init__(self):
        super().__init__('range_detection')
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 1)
        self.get_logger().info('Range detection node initialized.')

    def laser_callback(self, data):
        # Process range data
        front_distance = min(data.ranges[0:30] + data.ranges[-30:])  # Front distance
        self.get_logger().info(f'Front distance: {front_distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    range_detection_node = RangeDetection()
    rclpy.spin(range_detection_node)
    range_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
