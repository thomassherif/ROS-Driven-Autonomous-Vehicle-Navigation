#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# Vehicle parameters (adjust according to your vehicle)
wheelbase = 2.65  # meters

# Global variables for vehicle's position, orientation, and goal
current_x = 0.0
current_y = 0.0
current_yaw = 0.0
goal_x = None
goal_y = None

desired_speed = 0.0  # Initially set to 0.0

# Tolerance for reaching the goal
goal_tolerance = 0.0  # meters

# Callback: update current vehicle position from odometry
def odom_callback(msg):
    global current_x, current_y, current_yaw
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    orientation = msg.pose.pose.orientation
    _, _, current_yaw = euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

# Callback: update goal position from planner
def goal_callback(msg):
    global goal_x, goal_y
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y
    rospy.loginfo(f"New goal received: x={goal_x:.2f}, y={goal_y:.2f}")

# Utility: quaternion → euler conversion
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

# Calculate desired speed using a simple P-controller
def calculate_desired_speed():
    global current_x, current_y, goal_x, goal_y

    if goal_x is None or goal_y is None:
        return 0.0  # No goal yet

    distance_to_goal = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)

    if distance_to_goal < goal_tolerance:
        rospy.loginfo("Goal reached, setting desired speed to 0.0 m/s")
        return 0.0
    else:
        Kp = 0.5  # Tunable
        speed = Kp * distance_to_goal
        return min(speed, 10.0)  # cap max speed if needed

def main():
    global desired_speed

    rospy.init_node('position_control', anonymous=True)

    # Publisher
    desired_speed_pub = rospy.Publisher("/desired_speed", Float64, queue_size=10)

    # Subscribers
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/goal_pose", PoseStamped, goal_callback)

    rate = rospy.Rate(10)  # 10 Hz loop

    while not rospy.is_shutdown():
        desired_speed = calculate_desired_speed()
        desired_speed_pub.publish(desired_speed)

        rospy.loginfo(f"Publishing Desired Speed: {desired_speed:.2f} m/s")

        rate.sleep()

if __name__ == '__main__':
    main()
