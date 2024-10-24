import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan

class ObjectAvoidance(Node):
    def __init__(self):
        super().__init__('object_avoidance')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 1)
        self.get_logger().info('Object avoidance node initialized.')

    def laser_callback(self, data):
        # Check for obstacles in front of JARVIS
        front_distance = min(data.ranges[0:30] + data.ranges[-30:])  # Get minimum distance in front
        if front_distance < 0.5:  # Threshold for obstacle detection
            self.avoid_obstacle()

    def avoid_obstacle(self):
        self.get_logger().info('Obstacle detected! Avoiding...')
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 2.4  # Turn right to avoid the obstacle
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info(f'Published avoidance cmd_vel: linear={cmd_vel.linear.x}, angular={cmd_vel.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    object_avoidance_node = ObjectAvoidance()
    rclpy.spin(object_avoidance_node)
    object_avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
