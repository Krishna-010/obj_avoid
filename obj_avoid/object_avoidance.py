#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np
import random
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)
        
        # Publishers and Subscribers
        self.path_pub = rospy.Publisher('/planned_path', PoseArray, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=10)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10

        # Subscribe to sensor data here for obstacles
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

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
            rospy.sleep(0.1)

    def get_current_position(self):
        # Placeholder: Replace with current position data from odometry or localization
        return [0.0, 0.0]

    def run(self):
        rate = rospy.Rate(1)  # Hz
        while not rospy.is_shutdown():
            planner = RRTPlanner(start=self.get_current_position(), 
                                 goal=self.goal_position, 
                                 obstacle_list=self.obstacle_list, 
                                 step_size=self.step_size, 
                                 max_iter=self.max_iter)
            
            path = planner.plan()
            
            if path:
                rospy.loginfo("Path found, moving along path.")
                self.move_along_path(path)
            else:
                rospy.logwarn("Path not found. Re-planning...")
            
            rate.sleep()

if __name__ == "__main__":
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass
