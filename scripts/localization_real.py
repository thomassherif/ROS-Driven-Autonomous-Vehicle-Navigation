#!/usr/bin/env python3

import rospy
import serial
from geometry_msgs.msg import Pose2D, Vector3
import math

def parse_line(line):
    try:
        parts = line.strip().split(',')
        data = {}
        for part in parts:
            key, value = part.split(':')
            data[key.strip()] = float(value.strip())
        return data
    except Exception as e:
        rospy.logwarn(f"Failed to parse line: {line} — {e}")
        return None

def main():
    rospy.init_node('localization_node')

    pose_pub = rospy.Publisher('/localization_pose', Pose2D, queue_size=10)
    vel_pub = rospy.Publisher('/localization_vel', Vector3, queue_size=10)

    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()

    rate = rospy.Rate(20)  # 20 Hz

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            data = parse_line(line)
            if data:
                theta_deg = data.get("theta", 0.0)
                vd = data.get("Vd", 0.0)
                x = data.get("X", 0.0)
                y = data.get("Y", 0.0)

                theta_rad = math.radians(theta_deg)
                vx = vd * math.cos(theta_rad)
                vy = vd * math.sin(theta_rad)

                # Log the parsed and calculated data
                rospy.loginfo(f"Parsed Data -> theta: {theta_deg}°, Vd: {vd} m/s, X: {x} m, Y: {y} m")
                rospy.loginfo(f"Computed -> vx: {vx:.3f} m/s, vy: {vy:.3f} m/s")

                # Publish pose
                pose_msg = Pose2D(x=x, y=y, theta=theta_rad)
                pose_pub.publish(pose_msg)

                # Publish velocity
                vel_msg = Vector3(x=vx, y=vy, z=0.0)
                vel_pub.publish(vel_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass