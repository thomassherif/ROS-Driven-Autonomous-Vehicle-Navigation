#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Pose2D, PoseStamped

class PlannerNode:
    def __init__(self):
        rospy.init_node('planner_real', anonymous=True)  # node name is now 'planner_real'

        # Publisher to send next waypoint
        self.goal_pub = rospy.Publisher('/goal_pose', PoseStamped, queue_size=100)

        # Wait for the publisher to be advertised before proceeding
        while self.goal_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("Waiting for subscribers...")
            rospy.sleep(1)

        # Subscriber to get the current car position from the localization node
        rospy.Subscriber('/localization_pose', Pose2D, self.odom_callback)

        # Waypoints list (x, y)
        self.waypoints = [
            (13.0, 1.5),
            (20.0, 1.5),
            (32.0, -1.5),
            (52.0, -1.5),
            (65.0, 1.5),
            (90.0, 1.5)
        ]
        self.current_waypoint_idx = 0
        self.reached_threshold = 3.0  # meters

        self.current_pose = None  # updated from odom_callback

        self.rate = rospy.Rate(10)  # 10 Hz

        # Wait for user input before starting
        rospy.loginfo("Type 'start' and press Enter to begin...")
        user_input = input()
        while user_input.strip().lower() != "start" and not rospy.is_shutdown():
            rospy.loginfo("Waiting for 'start' command...")
            user_input = input()
        rospy.loginfo("Starting waypoint navigation!")

    def odom_callback(self, msg):
        # Update current pose with data from localization node
        self.current_pose = msg

    def distance(self, pose, target_x, target_y):
        # Calculate the distance to the target waypoint
        dx = target_x - pose.x
        dy = target_y - pose.y
        return math.hypot(dx, dy)

    def send_next_waypoint(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            rospy.loginfo("All waypoints reached!")
            return

        # Get the next waypoint
        x, y = self.waypoints[self.current_waypoint_idx]
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "odom"  # or "map" based on your setup

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0  # Assuming no rotation for simplicity

        rospy.loginfo(f"Publishing new waypoint: ({x}, {y})")
        self.goal_pub.publish(goal)

    def run(self):
        while not rospy.is_shutdown():
            if self.current_pose is None:
                self.rate.sleep()
                continue

            if self.current_waypoint_idx >= len(self.waypoints):
                self.rate.sleep()
                continue

            target_x, target_y = self.waypoints[self.current_waypoint_idx]
            dist = self.distance(self.current_pose, target_x, target_y)

            if dist < self.reached_threshold:
                rospy.loginfo(f"Reached waypoint {self.current_waypoint_idx} at ({target_x}, {target_y})")
                self.current_waypoint_idx += 1
                if self.current_waypoint_idx < len(self.waypoints):
                    self.send_next_waypoint()

            self.rate.sleep()

if __name__ == '__main__':
    try:
        planner = PlannerNode()
        planner.send_next_waypoint()  # Send the first waypoint after 'start'
        planner.run()
    except rospy.ROSInterruptException:
        pass
