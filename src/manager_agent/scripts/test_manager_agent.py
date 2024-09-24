#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Received feedback: %s", data.data)

def test_manager_agent():
    rospy.init_node('test_manager_agent', anonymous=True)
    pub = rospy.Publisher('/user_command', String, queue_size=10)
    rospy.Subscriber('/user_feedback', String, callback)
    rate = rospy.Rate(0.2)  # 0.2 Hz, one command every 5 seconds

    test_commands = [
        "Disassemble the a frame consisting of 3 elements, two columns and a beam",
    ]

    for command in test_commands:
        rospy.loginfo(f"Sending command: {command}")
        pub.publish(command)
        rospy.loginfo(f"Published command: {command}")
        rate.sleep()

    rospy.loginfo("Test commands sent. Listening for feedback...")
    rospy.spin()

if __name__ == '__main__':
    try:
        test_manager_agent()
    except rospy.ROSInterruptException:
        pass
