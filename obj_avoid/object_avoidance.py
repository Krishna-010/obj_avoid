#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np
import random
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ObstacleAvoidance:
    def __init__(self):
        super().__init__('obstacle_avoidance')
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,depth=10)
        # Publishers and Subscribers
        self.path_pub = self.create_publisher(PoseArray, '/planned_path', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to sensor data here for obstacles
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_profile)
    def laser_callback(self, msg):
        # Process laser data to update obstacle list
        self.obstacle_list = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        for i, range in enumerate(msg.ranges):
            if range < 1.0:  # Consider only close obstacles
                angle = angle_min + i * angle_increment
                obstacle_x = range * np.cos(angle)
                obstacle_y = range * np.sin(angle)
                self.obstacle_list.append((obstacle_x, obstacle_y, 0.1))  # 0.1 is radius estimate

    def move_along_path(self, path):
        for point in path:
            velocity_msg = Twist()
            direction = np.array(point) - np.array(self.get_current_position())
            distance = np.linalg.norm(direction)
            
            if distance < 0.1:  # Tolerance to stop at each waypoint
                continue
            
            velocity_msg.linear.x = min(0.18, 0.18 * distance)  # Cap at linear speed
            velocity_msg.angular.z = 2.4 * np.arctan2(direction[1], direction[0])  # Adjust angle
            
            self.velocity_publisher.publish(velocity_msg)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

    def get_current_position(self):
        # Placeholder: Replace with current position data from odometry or localization
        return [0.0, 0.0]

    def run(self):
        rate = self.create_rate(1)  # Hz
        while rclpy.ok():
            planner = RRTPlanner(start=self.get_current_position(), 
                                 goal=self.goal_position, 
                                 obstacle_list=self.obstacle_list, 
                                 step_size=self.step_size, 
                                 max_iter=self.max_iter)
            
            path = planner.plan()
            
            if path:
                self.get_logger().info("Path found, moving along path.")
                self.move_along_path(path)
            else:
                self.get_logger().warn("Path not found. Re-planning...")
            
            rate.sleep()
            
def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    try:
        obstacle_avoidance.run()
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        obstacle_avoidance.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()
    
