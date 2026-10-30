#!/usr/bin/env python3

import rospy
import sys
import termios
import tty
from std_msgs.msg import Float64, UInt8

# Define key mappings
MOVE_BINDINGS = {
    'w': (1, 0, 0, 0),   # Accelerate (Throttle up)
    's': (0, 0, 60, 0),  # Brake (set to 60)
    'a': (0, -0.5, 0, 0), # Steer left
    'd': (0, 0.5, 0, 0),  # Steer right
    'r': (0, 0, 60, 1),  # Switch to Reverse gear (brake first)
    'f': (0, 0, 60, 0)   # Switch to Drive gear (brake first)
}

# Global vehicle state
throttle = 0.0
steering = 0.0
brake = 0.0
gear = 0  # 0 = Drive, 1 = Reverse

def get_key():
    """Capture keyboard input."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    global throttle, steering, brake, gear

    rospy.init_node("Autonomous_Systems_MS_2_Teleop_Team_3", anonymous=True)

    # Publishers
    throttle_pub = rospy.Publisher("/throttle_cmd", Float64, queue_size=10)
    steering_pub = rospy.Publisher("/steering_cmd", Float64, queue_size=10)
    brake_pub = rospy.Publisher("/brake_cmd", Float64, queue_size=10)
    gear_pub = rospy.Publisher("/gear_cmd", UInt8, queue_size=10)

    rospy.loginfo("Teleoperation node started. Use 'WASD' to drive, 'R' for Reverse, 'F' for Drive.")
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        key = get_key()

        if key in MOVE_BINDINGS:
            throttle_cmd, steering_cmd, brake_cmd, gear_cmd = MOVE_BINDINGS[key]

            # Handle throttle/brake logic
            if key == 'w':  # Accelerate
                throttle = min(1.0, throttle + 0.1)  # Gradual acceleration
                brake = 0.0
            elif key == 's':  # Brake
                throttle = 0.0
                brake = 60.0
            elif key == 'a' or key == 'd':  # Steering
                steering = steering_cmd
            elif key == 'r':  # Reverse
                throttle = 0.0
                brake = 60.0
                gear = 1
            elif key == 'f':  # Drive mode
                throttle = 0.0
                brake = 60.0
                gear = 0

        else:
            # If no key is pressed, reset throttle and steering gradually
            throttle = 0.0
            steering = 0.0
            brake = 0.0  # Only brake if user presses 's'

        # Publish updated values
        throttle_pub.publish(Float64(throttle))
        steering_pub.publish(Float64(steering))
        brake_pub.publish(Float64(brake))
        gear_pub.publish(UInt8(gear))

        rospy.loginfo(f"Throttle: {throttle}, Steering: {steering}, Brake: {brake}, Gear: {gear}")

        if key == '\x03':  # Ctrl+C to exit
            break

        rate.sleep()

if __name__ == "__main__":
    main()
