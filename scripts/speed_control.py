#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math

# Global variables to store the current speed components
current_speed_x = 0.0
current_speed_y = 0.0
desired_speed = 0.0  # This will be set by the Position Control Node

# Callback function to print vehicle speed (linear velocity)
def twist_callback(msg):
    global current_speed_x, current_speed_y
    # Get the linear velocities in the x and y directions
    current_speed_x = msg.twist.linear.x
    current_speed_y = msg.twist.linear.y
    rospy.loginfo(f"Speed in x direction: {current_speed_x:.2f} m/s, Speed in y direction: {current_speed_y:.2f} m/s")

# Callback function to get the desired speed from the Position Control Node
def desired_speed_callback(msg):
    global desired_speed
    # Update the desired speed with the value from the Position Control Node
    desired_speed = msg.data
    rospy.loginfo(f"Received Desired Speed: {desired_speed:.2f} m/s")

def main():
    rospy.init_node('speed_control', anonymous=True)

    # Define publishers for throttle and brake commands
    throttle_pub = rospy.Publisher("/throttle_cmd", Float64, queue_size=10)
    brake_pub = rospy.Publisher("/brake_cmd", Float64, queue_size=10)  # Assuming you have a brake command topic
    
    # Define subscriber to read vehicle's current speed from /twist topic
    rospy.Subscriber("/twist", TwistStamped, twist_callback)
    
    # Subscribe to receive the desired speed from the Position Control Node
    rospy.Subscriber("/desired_speed", Float64, desired_speed_callback)

    # Give Gazebo some time before starting control
    rospy.sleep(2)

    # Control loop
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Calculate the resultant speed (magnitude of the velocity vector)
        current_speed = math.sqrt(current_speed_x**2 + current_speed_y**2)

        # If desired_speed is received, calculate the error between desired and current speed
        if desired_speed > 0:
            error = desired_speed - current_speed
        
            # Simple proportional control (you can tune Kp as needed)
            Kp = 0.25
            throttle_cmd = Kp * error

            # Clamp the throttle command between 0 and 1
            throttle_cmd = max(0.0, min(throttle_cmd, 1.0))

            # Publish the throttle command
            throttle_pub.publish(Float64(throttle_cmd))

            # Ensure the brake is disengaged when moving
            brake_pub.publish(Float64(0.0))

            # Log information for debugging
            rospy.loginfo(f"Desired Speed: {desired_speed:.2f} m/s | Current Speed: {current_speed:.2f} m/s | Throttle Command: {throttle_cmd:.2f}")
        
        # Handle case when desired_speed is 0 (brake the vehicle)
        elif desired_speed == 0:
            # Apply brake to stop the vehicle
            brake_pub.publish(Float64(150.0))  # Full brake (adjust as needed)
            throttle_pub.publish(Float64(0.0))  # En    sure throttle is set to 0
            rospy.loginfo("Desired Speed is 0, applying brakes to stop the vehicle.")

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == "__main__":
    main()
