#!/usr/bin/env python3

import rospy
import math
import random
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

# Noise parameters
noise_mu = 0.0    # Mean of the Gaussian noise
noise_sigma = 0.001 # Standard deviation of the noise

# Kalman filter parameters
Q = 0.0000001  # Process noise covariance
R = 0.000001   # Measurement noise covariance

# Kalman filter state variables
x_est = 0.0  # Initial estimate
P_est = 1.0  # Initial covariance

# Kalman filter update step
def kalman_filter(z):
    global x_est, P_est
    # Prediction step
    x_pred = x_est
    P_pred = P_est + Q

    # Update step
    K = P_pred / (P_pred + R)  # Kalman gain
    x_est = x_pred + K * (z - x_pred)  # Update estimate
    P_est = (1 - K) * P_pred  # Update covariance

    return x_est

# Add noise to the input data
def add_noise(value):
    return value + random.gauss(noise_mu, noise_sigma)

# Callback for odometry data
def odom_callback(msg):
    noisy_x = add_noise(msg.pose.pose.position.x)
    noisy_y = add_noise(msg.pose.pose.position.y)
    filtered_x = kalman_filter(noisy_x)
    filtered_y = kalman_filter(noisy_y)
    rospy.loginfo(f"Filtered Position: x={filtered_x:.2f}, y={filtered_y:.2f}")
    filtered_odom = Odometry()
    filtered_odom.pose.pose.position.x = filtered_x
    filtered_odom.pose.pose.position.y = filtered_y
    filtered_odom_pub.publish(filtered_odom)

# Callback for speed data
def twist_callback(msg):
    noisy_speed = add_noise(msg.twist.linear.x)
    filtered_speed = kalman_filter(noisy_speed)
    rospy.loginfo(f"Filtered Speed: {filtered_speed:.2f}")
    filtered_speed_msg = TwistStamped()
    filtered_speed_msg.twist.linear.x = filtered_speed
    filtered_speed_pub.publish(filtered_speed_msg)

# Main function to initialize the node
def main():
    rospy.init_node('kalman_filter_node', anonymous=True)
    global filtered_odom_pub, filtered_speed_pub

    filtered_odom_pub = rospy.Publisher('/filtered_odom', Odometry, queue_size=10)
    filtered_speed_pub = rospy.Publisher('/filtered_twist', TwistStamped, queue_size=10)

    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/twist', TwistStamped, twist_callback)

    rospy.loginfo("Kalman filter node started.")
    rospy.spin()

if __name__ == '__main__':
    main()
