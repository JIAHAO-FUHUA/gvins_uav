#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

# Function to handle incoming PoseStamped messages
def callback(msg):
    # Extract time in seconds and nanoseconds and combine them into a single decimal value
    time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9

    # Extract the position and orientation from the message
    position = msg.pose.position
    orientation = msg.pose.orientation

    # Format the data as a string
    data = f"{time:.9f} {position.x} {position.y} {position.z} {orientation.x} {orientation.y} {orientation.z} {orientation.w}\n"

    # Open the file in append mode and write the data
    with open('/home/hu/data/output/vicon_pose.txt', 'a') as file:
        file.write(data)

def listener():
    # Initialize the ROS node
    rospy.init_node('pose_listener', anonymous=True)

    # Subscribe to the /mavros/vision_pose/pose topic
    rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, callback)

    # Keep the node running until interrupted
    rospy.spin()

if __name__ == '__main__':
    listener()
