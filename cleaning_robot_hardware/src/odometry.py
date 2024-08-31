#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Int32, Int64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist

class OdometryCalculator:
    def __init__(self):
        rospy.init_node('odometry_node', anonymous=True)
        
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        
        self.wheel_distance = 0.288  # cm
        self.wheel_radius = 0.055
        self.ticks_per_revolution = 3133
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        rospy.Subscriber('/motor/feedback', Int64MultiArray, self.motor_feedback_callback)
        
    def motor_feedback_callback(self, msg):
        self.left_ticks = msg.data[0]
        self.right_ticks = msg.data[1]
    
    def calculate_odometry(self):
        rate = rospy.Rate(10)  # Update rate (10 Hz)
        self.dt = 1.0 / 10.0
        
        while not rospy.is_shutdown():
            delta_left = self.left_ticks - self.last_left_ticks
            delta_right = self.right_ticks - self.last_right_ticks
            
            self.last_left_ticks = self.left_ticks
            self.last_right_ticks = self.right_ticks
            
            distance_left = (2 * 3.14159 * self.wheel_radius * delta_left) / self.ticks_per_revolution
            distance_right = (2 * 3.14159 * self.wheel_radius * delta_right) / self.ticks_per_revolution
            
            delta_distance = (distance_left + distance_right) / 2
            delta_theta = (distance_right - distance_left) / self.wheel_distance
            
            self.x += delta_distance * math.cos(self.theta)
            self.y += delta_distance * math.sin(self.theta)
            self.theta += delta_theta
            
            linear_velocity = delta_distance / self.dt  # Calculate linear velocity
            angular_velocity = delta_theta / self.dt  # Calculate angular velocity
            
            self.publish_odometry(linear_velocity, angular_velocity)
            
            rate.sleep()
    
    def publish_odometry(self, linear_velocity, angular_velocity):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set position
        odom.pose.pose.position = Point(self.x, self.y, 0)
        odom.pose.pose.orientation = Quaternion(0, 0, math.sin(self.theta/2), math.cos(self.theta/2))
        
        # Set velocity (assuming it's zero in this example)
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        
        # Publish the odometry message
        self.odom_pub.publish(odom)


if __name__ == '__main__':
    try:
        odometry_calculator = OdometryCalculator()
        odometry_calculator.calculate_odometry()
    except rospy.ROSInterruptException:
        pass