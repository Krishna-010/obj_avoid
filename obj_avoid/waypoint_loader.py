import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class WaypointLoader(Node):
    def __init__(self):
        # Predefined waypoints
        self.waypoints = [
            Point(x=1.5, y=0.0, z=0.0),
            Point(x=1.5, y=1.4, z=0.0),
            Point(x=0.0, y=1.4, z=0.0),
            # Add more waypoints as needed
        ]
        self.current_index = 0

        self.timer = self.create_timer(1.0, self.publish_waypoint)

    def publish_waypoint(self):
        if self.current_index < len(self.waypoints):
            self.waypoint_pub.publish(self.waypoints[self.current_index])
            self.get_logger().info(f'Published waypoint: {self.waypoints[self.current_index]}')
            self.current_index += 1
        else:
            self.current_index = 0  # Reset to first waypoint

def main(args=None):
    rclpy.init(args=args)
    waypoint_loader = WaypointLoader()
    rclpy.spin(waypoint_loader)
    waypoint_loader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
