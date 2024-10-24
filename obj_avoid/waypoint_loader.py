import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class WaypointLoader(Node):
    def __init__(self):
        super().__init__('waypoint_loader')
        self.waypoints = [(1.5, 0), (1.5, 1.4), (0, 1.4)]  # Predefined waypoints
        self.get_logger().info(f'Waypoints loaded: {self.waypoints}')

    def get_waypoints(self):
        return self.waypoints

def main(args=None):
    rclpy.init(args=args)
    waypoint_loader_node = WaypointLoader()
    rclpy.spin(waypoint_loader_node)
    waypoint_loader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
