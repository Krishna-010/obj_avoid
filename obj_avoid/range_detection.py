#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class RangeDetection(Node):
    def __init__(self):
        super().__init__('range_detection')
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,depth=10)
        # Publishers and Subscribers
        self.obstacle_pub = self.create_publisher(Bool,'/obstacle_detected',10)
        self.scan_sub = self.create_subscription(LaserScan,'/scan',self.scan_callback,qos_profile)

    def scan_callback(self, msg):
        # Check if there are obstacles close to the robot
        obstacle_detected = any(distance < 0.5 for distance in msg.ranges)
        self.obstacle_pub.publish(obstacle_detected)

def main(args=None):
    rclpy.init(args=args)
    node = RangeDetection()
    try:
        rclpy.spin(node)  # Spin the node
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        node.destroy_node()  # Clean up the node
        rclpy.shutdown()     # Shutdown the rclpy library
        
if __name__ == "__main__":
    main()
