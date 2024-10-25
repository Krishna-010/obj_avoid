import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
import random
from scipy.interpolate import CubicSpline
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class RRTObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('rrt_obstacle_avoidance')
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
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.update_cmd_vel)

        self.rrt_path = []  # Store the RRT-generated path
        self.smooth_path = []  # Smoothed version of the RRT path
        self.current_goal = (2.0, 2.0)  # Set your desired goal here
        self.goal_threshold = 0.1  # Threshold to consider the goal reached
        self.velocity_constraints = {'linear': 0.18, 'angular': 2.4}

        # Define map boundaries (for RRT sampling space)
        self.map_bounds = {'x_min': 0, 'x_max': 3, 'y_min': 0, 'y_max': 2}
        self.obstacles = []  # Known and detected obstacles
        self.state = 'GO_TO_GOAL'  # Initial state

    def scan_callback(self, scan_msg):
        """Callback to handle incoming LaserScan data and detect obstacles."""
        obstacle_detected = False
        self.obstacles = []
        for i, distance in enumerate(scan_msg.ranges):
            if scan_msg.range_min < distance < scan_msg.range_max:
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x_obstacle = distance * math.cos(angle)
                y_obstacle = distance * math.sin(angle)
                self.obstacles.append((x_obstacle, y_obstacle))
                obstacle_detected = True
        
        # Switch states based on obstacle detection
        if obstacle_detected:
            self.state = 'AVOID_OBSTACLE'
            self.get_logger().info("Obstacle detected, running RRT...")
            self.rrt_path = self.run_rrt(self.current_goal)

            if self.rrt_path:
                self.smooth_path = self.smooth_rrt_path(self.rrt_path)
        else:
            self.state = 'GO_TO_GOAL'
            self.get_logger().info("No obstacle detected, moving to goal...")
            self.move_to_goal()

    def run_rrt(self, goal):
        """Runs the RRT algorithm to generate a path around the obstacles."""
        max_iterations = 1000
        step_size = 0.2

        start = (0.0, 0.0)  # Start at the current position
        goal = goal or (2.0, 2.0)  # Default goal if none provided

        tree = {start: None}
        for _ in range(max_iterations):
            rand_point = (random.uniform(self.map_bounds['x_min'], self.map_bounds['x_max']),
                          random.uniform(self.map_bounds['y_min'], self.map_bounds['y_max']))
            
            nearest_point = self.get_nearest_point(tree, rand_point)
            new_point = self.steer(nearest_point, rand_point, step_size)

            if self.is_collision_free(nearest_point, new_point):
                tree[new_point] = nearest_point
                if self.euclidean_distance(new_point, goal) < step_size:
                    return self.retrace_path(tree, new_point)

        return []

    def steer(self, from_point, to_point, step_size):
        """Steer towards a point within the specified step size."""
        angle = math.atan2(to_point[1] - from_point[1], to_point[0] - from_point[0])
        return (from_point[0] + step_size * math.cos(angle), from_point[1] + step_size * math.sin(angle))

    def is_collision_free(self, point_a, point_b):
        """Check for collision between two points, given known obstacles."""
        for obs in self.obstacles:
            # Implement collision detection logic here (e.g., bounding box, distance check)
            # Example: Check distance to the obstacle
            if self.euclidean_distance(obs, point_a) < 0.1 or self.euclidean_distance(obs, point_b) < 0.1:
                return False
        return True

    def get_nearest_point(self, tree, point):
        """Find the nearest point in the tree to the given point."""
        return min(tree.keys(), key=lambda p: self.euclidean_distance(p, point))

    def retrace_path(self, tree, new_point):
        """Retrace the path from the new point to the start point."""
        path = []
        while new_point is not None:
            path.append(new_point)
            new_point = tree[new_point]
        return path[::-1]  # Return reversed path

    def smooth_rrt_path(self, path):
        """Smooth the path using cubic splines."""
        if len(path) < 2:
            return path

        xs, ys = zip(*path)
        cs = CubicSpline(xs, ys)
        xs_smooth = np.linspace(min(xs), max(xs), num=100)
        ys_smooth = cs(xs_smooth)

        return list(zip(xs_smooth, ys_smooth))

    def move_to_goal(self):
        """Moves the bot towards the current goal or RRT path."""
        if self.state == 'GO_TO_GOAL':
            vel_msg = Twist()
            distance = self.euclidean_distance((0, 0), self.current_goal)
            angle_to_goal = math.atan2(self.current_goal[1], self.current_goal[0])

            vel_msg.linear.x = min(self.velocity_constraints['linear'], distance)
            vel_msg.angular.z = min(self.velocity_constraints['angular'], angle_to_goal)

            self.cmd_pub.publish(vel_msg)
        elif self.state == 'AVOID_OBSTACLE' and self.smooth_path:
            # Implement following the smoothed RRT path
            vel_msg = Twist()
            next_point = self.smooth_path[0]
            distance = self.euclidean_distance((0, 0), next_point)

            vel_msg.linear.x = min(self.velocity_constraints['linear'], distance)
            angle_to_path = math.atan2(next_point[1], next_point[0])
            vel_msg.angular.z = min(self.velocity_constraints['angular'], angle_to_path)

            self.cmd_pub.publish(vel_msg)

    def euclidean_distance(self, p1, p2):
        """Returns the Euclidean distance between two points."""
        return np.linalg.norm(np.array(p1) - np.array(p2))

def main(args=None):
    rclpy.init(args=args)
    rrt_obstacle_avoidance_node = RRTObstacleAvoidance()
    rclpy.spin(rrt_obstacle_avoidance_node)
    rrt_obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
