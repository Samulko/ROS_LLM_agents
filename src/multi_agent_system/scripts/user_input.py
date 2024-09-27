#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def user_input_node():
    rospy.init_node('user_input_node', anonymous=True)
    user_command_pub = rospy.Publisher('/user_command', String, queue_size=10)
    rospy.loginfo("User Input Node Initialized. Type your commands below:")

    while not rospy.is_shutdown():
        try:
            user_command = input("Enter command: ")
            user_command_pub.publish(user_command)
            rospy.loginfo(f"Published command: {user_command}")
        except (EOFError, KeyboardInterrupt):
            rospy.loginfo("Shutting down User Input Node.")
            break

if __name__ == '__main__':
    try:
        user_input_node()
    except rospy.ROSInterruptException:
        pass
