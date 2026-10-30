#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, UInt8
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

# Callback functions to print vehicle states
def twist_callback(msg):
    rospy.loginfo(f"Velocity (linear): {msg.twist.linear.x:.2f} m/s, Yaw rate: {msg.twist.angular.z:.2f} rad/s")

def gear_callback(msg):
    gear_state = "DRIVE" if msg.data == 0 else "REVERSE"
    rospy.loginfo(f"Current Gear: {gear_state}")

def steering_callback(msg):
    rospy.loginfo(f"Steering Wheel Angle: {msg.data:.2f} rad")

def odom_callback(msg):
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    rospy.loginfo(f"Position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
    rospy.loginfo(f"Orientation: x={ori.x:.2f}, y={ori.y:.2f}, z={ori.z:.2f}, w={ori.w:.2f}")

def main():
    rospy.init_node("Autonomous_Systems_MS_2_OLR_Team_3", anonymous=True)

    # Get parameters from launch file
    throttle_value = rospy.get_param("~throttle", 0.0)
    steering_value = rospy.get_param("~steering", 0.0)
    brake_value = rospy.get_param("~brake", 0.0)
    gear_value = rospy.get_param("~gear", 0)  # 0 = DRIVE, 1 = REVERSE

    # Define publishers for vehicle control commands
    throttle_pub = rospy.Publisher("/throttle_cmd", Float64, queue_size=10)
    steering_pub = rospy.Publisher("/steering_cmd", Float64, queue_size=10)
    brake_pub = rospy.Publisher("/brake_cmd", Float64, queue_size=10)
    gear_pub = rospy.Publisher("/gear_cmd", UInt8, queue_size=10)

    # Define subscribers to read vehicle state
    rospy.Subscriber("/twist", TwistStamped, twist_callback)
    rospy.Subscriber("/gear_state", UInt8, gear_callback)
    rospy.Subscriber("/steering_state", Float64, steering_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    # Give Gazebo some time before sending commands
    rospy.sleep(2)

    # Publish control commands in a loop
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        throttle_pub.publish(Float64(throttle_value))  
        steering_pub.publish(Float64(steering_value))  
        brake_pub.publish(Float64(brake_value))  
        gear_pub.publish(UInt8(gear_value))  
    
        rate.sleep()

if __name__ == "__main__":
    main()
