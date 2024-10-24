import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObjectAvoidance(Node):
    def __init__(self):
        super().__init__('object_avoidance')

        # Updated QoS profile to match /scan publisher
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            liveliness=rclpy.qos.QoSLivelinessPolicy.AUTOMATIC
        )

        # Subscribe to /scan with updated QoS
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        # Publisher for controlling the robot's velocity
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.obstacle_detected = False

    def scan_callback(self, msg):
        # Process laser scan data for object avoidance
        self.get_logger().info('Processing /scan data for obstacle avoidance')
        # Object avoidance logic here (keeping tangential path)

    def timer_callback(self):
        twist_msg = Twist()
        if self.obstacle_detected:
            # Adjust for obstacle
            self.get_logger().info('Obstacle detected, adjusting path...')
            # Set twist_msg for tangential path
        else:
            self.get_logger().info('No obstacle, continuing to goal...')
            # Set normal twist_msg to move toward the goal

        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
