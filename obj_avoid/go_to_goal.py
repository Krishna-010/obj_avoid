#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class GoToGoal:
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)
        
        # Goal waypoints (x, y)
        self.waypoints = [(1.5, 0), (1.5, 1.4), (1.4, 0)]
        self.current_waypoint_index = 0
        
        # Velocity constraints
        self.max_linear_vel = 0.18
        self.max_angular_vel = 2.4

        # PID Controller variables
        self.kp_linear = 0.5
        self.kp_angular = 1.2
        self.ki_linear = 0.0
        self.ki_angular = 0.0
        self.kd_linear = 0.1
        self.kd_angular = 0.3
        self.previous_error_linear = 0
        self.previous_error_angular = 0
        self.integral_linear = 0
        self.integral_angular = 0

        # Kalman Filter variables
        self.position_estimate = np.array([0.0, 0.0])
        self.velocity_estimate = np.array([0.0, 0.0])
        self.covariance = np.identity(2)
        self.kalman_filter = KalmanFilter()
        self.previous_time = rospy.Time.now()
        # Initialize Kalman filter variables
        # State vector [x_position, y_position, x_velocity, y_velocity]
        self.state_estimate = np.array([0.0, 0.0, 0.0, 0.0])
        # State covariance matrix (initial uncertainty)
        self.covariance_estimate = np.eye(4)
        # Define process noise covariance matrix (Q)
        self.process_noise = np.diag([0.1, 0.1, 0.01, 0.01])
        # Define measurement noise covariance matrix (R)
        self.measurement_noise = np.diag([0.2, 0.2])
        # Define measurement matrix (H)
        self.measurement_matrix = np.array([[1, 0, 0, 0],
                                            [0, 1, 0, 0]])
        # Define transition matrix (F)
        self.transition_matrix = np.eye(4)

        # ROS Publishers and Subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.obstacle_detected_sub = rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_callback)

        self.reached_goal = False
        self.obstacle_detected = False
    def predict(self, dt):
        """
        Prediction step of the Kalman Filter.
        Updates the state estimate and covariance matrix.
        """
        # Update transition matrix to account for time delta
        self.transition_matrix[0, 2] = dt
        self.transition_matrix[1, 3] = dt

        # Predict the next state
        self.state_estimate = np.dot(self.transition_matrix, self.state_estimate)

        # Update the covariance matrix
        self.covariance_estimate = np.dot(self.transition_matrix, 
                                          np.dot(self.covariance_estimate, 
                                                 self.transition_matrix.T)) + self.process_noise

    def update(self, measurement):
        """
        Update step of the Kalman Filter.
        Combines the predicted state with the new measurement.
        """
        # Measurement residual
        measurement_residual = measurement - np.dot(self.measurement_matrix, self.state_estimate)

        # Calculate the residual covariance
        residual_covariance = np.dot(self.measurement_matrix, 
                                     np.dot(self.covariance_estimate, 
                                            self.measurement_matrix.T)) + self.measurement_noise

        # Kalman gain
        kalman_gain = np.dot(self.covariance_estimate, 
                             np.dot(self.measurement_matrix.T, 
                                    np.linalg.inv(residual_covariance)))

        # Update the state estimate
        self.state_estimate += np.dot(kalman_gain, measurement_residual)

        # Update the covariance estimate
        identity = np.eye(self.covariance_estimate.shape[0])
        self.covariance_estimate = np.dot((identity - np.dot(kalman_gain, self.measurement_matrix)), 
                                          self.covariance_estimate)

    def get_position_estimate(self):
        return self.state_estimate[0], self.state_estimate[1]

    def get_velocity_estimate(self):
        return self.state_estimate[2], self.state_estimate[3]
        
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        current_measurement = np.array([x, y])
        yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

         # Predict phase with time delta (dt)
        current_time = rospy.Time.now()
        dt = (current_time - self.previous_time).to_sec()
        self.kalman_filter.predict(dt)

        # Update phase with current measurement
        self.kalman_filter.update(current_measurement)
        # Kalman filter estimate update here based on new odometry data
        self.position_estimate = np.array(self.kalman_filter.get_position_estimate())
        self.velocity_estimate = np.array(self.kalman_filter.get_velocity_estimate())
        self.previous_time = current_time

    def get_yaw_from_quaternion(self, orientation):
        # Conversion from quaternion to yaw
        # [Similar to tf transformations]
        pass

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data

    def pid_control(self, distance_error, angle_error):
        # Proportional
        proportional_linear = self.kp_linear * distance_error
        proportional_angular = self.kp_angular * angle_error

        # Integral
        self.integral_linear += distance_error
        self.integral_angular += angle_error
        integral_linear = self.ki_linear * self.integral_linear
        integral_angular = self.ki_angular * self.integral_angular

        # Derivative
        derivative_linear = self.kd_linear * (distance_error - self.previous_error_linear)
        derivative_angular = self.kd_angular * (angle_error - self.previous_error_angular)

        # Save previous errors
        self.previous_error_linear = distance_error
        self.previous_error_angular = angle_error

        # Total control signals
        linear_vel = proportional_linear + integral_linear + derivative_linear
        angular_vel = proportional_angular + integral_angular + derivative_angular

        # Constrain velocities
        linear_vel = min(self.max_linear_vel, linear_vel)
        angular_vel = min(self.max_angular_vel, angular_vel)

        return linear_vel, angular_vel

    def go_to_goal(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.current_waypoint_index < len(self.waypoints):
            if self.reached_goal:
                rospy.loginfo("Reached final goal!")
                break

            goal_x, goal_y = self.waypoints[self.current_waypoint_index]
            distance_to_goal = math.sqrt((goal_x - self.position_estimate[0])**2 + (goal_y - self.position_estimate[1])**2)
            angle_to_goal = math.atan2(goal_y - self.position_estimate[1], goal_x - self.position_estimate[0])
            yaw_error = angle_to_goal  # Adjust yaw using Kalman or odom heading

            if distance_to_goal < 0.1:
                self.current_waypoint_index += 1
                rospy.loginfo("Waypoint reached, moving to next waypoint.")

            else:
                # Apply PID control to compute velocities
                linear_vel, angular_vel = self.pid_control(distance_to_goal, yaw_error)
                cmd = Twist()
                cmd.linear.x = linear_vel
                cmd.angular.z = angular_vel
                self.cmd_pub.publish(cmd)

            rate.sleep()

if __name__ == "__main__":
    try:
        node = GoToGoal()
        node.go_to_goal()
    except rospy.ROSInterruptException:
        pass
