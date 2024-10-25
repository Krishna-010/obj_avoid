import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class WaypointLoader(Node):
    def __init__(self):
        super().__init__('waypoint_loader')
        self.publisher_ = self.create_publisher(Point, '/waypoint', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoint)
        self.waypoints = [(0.0, 0.0), (1.5, 0.0), (1.5, 1.4), (0.0, 1.4)]
        self.index = 0
        self.get_logger().info("Waypoint loader initialized.")

    def publish_waypoint(self):
        if self.index < len(self.waypoints):
            x, y = float(self.waypoints[self.index][0]), float(self.waypoints[self.index][1])
            waypoint = Point(x=x, y=y, z=0.0)
            self.publisher_.publish(waypoint)
            self.get_logger().info(f"Published waypoint: ({x}, {y})")
            self.index += 1

def main(args=None):
    rclpy.init(args=args)
    waypoint_loader = WaypointLoader()
    rclpy.spin(waypoint_loader)
    waypoint_loader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
