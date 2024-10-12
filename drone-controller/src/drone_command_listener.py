#!/usr/bin/env python3

import rospy
from plutodrone.msg import PlutoMsg

# Callback function to handle the incoming message
def callback(msg):
    # Print the received PlutoMsg data
    rospy.loginfo(f"Received PlutoMsg:\n"
                  f"rcYaw: {msg.rcYaw}, rcPitch: {msg.rcPitch}, rcRoll: {msg.rcRoll}, rcThrottle: {msg.rcThrottle}, "
                  f"rcAUX1: {msg.rcAUX1}, rcAUX2: {msg.rcAUX2}, rcAUX3: {msg.rcAUX3}, rcAUX4: {msg.rcAUX4}")

def listener():
    # Initialize the ROS node
    rospy.init_node('drone_command_listener', anonymous=True)

    # Subscribe to the /drone_command topic
    rospy.Subscriber('/drone_command', PlutoMsg, callback)

    # Keep the node alive and listening
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
