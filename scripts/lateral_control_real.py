#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped

# Vehicle parameters (adjust according to your vehicle)
wheelbase = 2.65  # Wheelbase of the vehicle in meters (adjust as needed)

goal_x = 0.0  # Initial goal x coordinate
goal_y = 0.0  # Initial goal y coordinate

# Global variables for vehicle's position and orientation
current_x = 0.0
current_y = 0.0
current_yaw = 0.0

# Callback function to get the vehicle's position and orientation from /localization_pose
def localization_pose_callback(msg):
    global current_x, current_y, current_yaw
    current_x = msg.x
    current_y = msg.y
    current_yaw = msg.theta

    rospy.loginfo(f"Current Position: x={current_x:.2f}, y={current_y:.2f}, yaw={current_yaw:.2f}")

# Callback function to get the goal position from the /goal_pose topic
def goal_callback(msg):
    global goal_x, goal_y
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y

    rospy.loginfo(f"New Goal Received: x={goal_x:.2f}, y={goal_y:.2f}")

# Function to calculate the steering angle based on the goal point
def lateral_control():
    kp = 2  # Steering Gain
    global current_x, current_y, current_yaw, wheelbase, goal_x, goal_y

    # Calculate the error in the global frame (distance to the goal)
    dx = goal_x - current_x
    dy = goal_y - current_y

    # Transform the error to the vehicle's frame (rotate by the vehicle's yaw)
    error_x = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
    error_y = -dx * math.sin(current_yaw) + dy * math.cos(current_yaw)

    # Calculate curvature (1/r) based on the lateral error (y)
    curvature = 2 * error_y / (wheelbase ** 2)

    # Calculate the steering angle (gamma) based on curvature
    steering_angle = math.atan(curvature * wheelbase)

    return steering_angle * kp

def main():
    global current_x, current_y, current_yaw

    rospy.init_node('lateral_control', anonymous=True)

    # Publisher for steering commands
    steering_pub = rospy.Publisher("/steering_cmd", Float64, queue_size=10)

    # Subscribe to the localization pose to get position and orientation
    rospy.Subscriber("/localization_pose", Pose2D, localization_pose_callback)

    # Subscribe to the goal pose to get the goal position
    rospy.Subscriber("/goal_pose", PoseStamped, goal_callback)

    rate = rospy.Rate(10)  # 10 Hz control loop
    while not rospy.is_shutdown():
        # Calculate the steering command based on lateral control
        steering_cmd = lateral_control()

        # Publish the steering command to control the vehicle
        steering_pub.publish(Float64(steering_cmd))

        # Log control information
        rospy.loginfo(f"Steering Command: {steering_cmd:.2f}")

        # Maintain loop rate
        rate.sleep()

if __name__ == '__main__':
    main()
