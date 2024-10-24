import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class Transformation(Node):
    def __init__(self):
        super().__init__('transformation')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.get_logger().info('Transformation node initialized.')
        self.global_pos = np.array([0.0, 0.0])
        self.initial_position = np.array([0.0, 0.0])
        self.initial_orientation = 0.0
        self.first_update = True

    def odom_callback(self, odom):
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        yaw = self.get_yaw_from_quaternion(orientation)

        if self.first_update:
            self.initial_position = np.array([position.x, position.y])
            self.initial_orientation = yaw
            self.first_update = False

        # Update global position
        self.global_pos = np.array([position.x, position.y]) - self.initial_position
        self.global_pos[0] = np.cos(self.initial_orientation) * (position.x - self.initial_position[0]) + \
                             np.sin(self.initial_orientation) * (position.y - self.initial_position[1])
        self.global_pos[1] = -np.sin(self.initial_orientation) * (position.x - self.initial_position[0]) + \
                             np.cos(self.initial_orientation) * (position.y - self.initial_position[1])

        self.get_logger().info(f'Global position: x={self.global_pos[0]}, y={self.global_pos[1]}, orientation={yaw - self.initial_orientation:.2f}')

    def get_yaw_from_quaternion(self, quat):
        return np.arctan2(2.0 * (quat.z * quat.w + quat.x * quat.y), 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))

def main(args=None):
    rclpy.init(args=args)
    transformation_node = Transformation()
    rclpy.spin(transformation_node)
    transformation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
