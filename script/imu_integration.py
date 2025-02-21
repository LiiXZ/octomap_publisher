#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class ImuIntegrationNode:
    def __init__(self):
        rospy.init_node('imu_integration_node', anonymous=True)
        
        # Initialize variables
        self.initial_time = None
        self.initial_velocity = 0.0
        
        # ROS subscribers and publishers
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.velocity_pub = rospy.Publisher('/integrated_velocity', Twist, queue_size=10)
        
        rospy.spin()

    def imu_callback(self, msg):
        if self.initial_time is None:
            self.initial_time = msg.header.stamp
        
        # Calculate time difference since initial time
        dt = (msg.header.stamp - self.initial_time).to_sec()
        self.initial_time = msg.header.stamp
        # Filter acceleration data (ignore values less than 0.1)
        linear_acceleration = msg.linear_acceleration.x
        if abs(linear_acceleration) < 0.1:
            return  # Skip processing for small accelerations
        
        # Integrate linear acceleration to compute velocity (assuming along x-axis)
        new_velocity = self.initial_velocity + linear_acceleration * dt
        self.initial_velocity = new_velocity
        
        # Publish integrated velocity
        vel_msg = Twist()
        vel_msg.linear.x = new_velocity
        vel_msg.linear.y = 0.0  # Assuming no velocity in y and z directions
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        
        self.velocity_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        ImuIntegrationNode()
    except rospy.ROSInterruptException:
        pass

