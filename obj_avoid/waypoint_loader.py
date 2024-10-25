import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class WaypointLoader(Node):
    def __init__(self):
        super().__init__('waypoint_loader')  # Properly initializes the Node
        self.waypoints = [(0.0, 0.0), (1.5, 0.0), (1.5, 1.4), (0.0, 1.4)]
        self.current_index = 0
        self.publisher = self.create_publisher(Point, 'waypoint', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoint)  # Timer setup with the correct callback

    def publish_waypoint(self):
        if self.current_index < len(self.waypoints):
            waypoint = Point()
            waypoint.x, waypoint.y = self.waypoints[self.current_index]
            waypoint.z = 0.0
            self.publisher.publish(waypoint)
            self.get_logger().info(f'Published waypoint: ({waypoint.x}, {waypoint.y})')
            self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    waypoint_loader = WaypointLoader()
    rclpy.spin(waypoint_loader)
    waypoint_loader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
