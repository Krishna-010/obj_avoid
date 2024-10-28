#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class RangeDetection:
    def __init__(self):
        rospy.init_node('range_detection', anonymous=True)
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,depth=10)
        # Publishers and Subscribers
        self.obstacle_pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback,qos_profile)

    def scan_callback(self, msg):
        # Check if there are obstacles close to the robot
        obstacle_detected = any(distance < 0.5 for distance in msg.ranges)
        self.obstacle_pub.publish(obstacle_detected)

if __name__ == "__main__":
    try:
        node = RangeDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
