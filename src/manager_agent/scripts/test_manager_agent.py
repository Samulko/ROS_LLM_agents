#!/usr/bin/env python3

import unittest
import rospy
from std_msgs.msg import String
from multi_agent_system.srv import ValidateRequest

class TestManagerAgent(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_manager_agent', anonymous=True)
        self.user_command_pub = rospy.Publisher('/user_command', String, queue_size=10)
        self.feedback_received = False
        self.feedback_message = ""
        rospy.Subscriber('/user_feedback', String, self.feedback_callback)
        rospy.wait_for_service('/validate_request')
        self.validate_request = rospy.ServiceProxy('/validate_request', ValidateRequest)

    def feedback_callback(self, data):
        self.feedback_received = True
        self.feedback_message = data.data

    def test_user_command_processing(self):
        test_command = "Disassemble the frame"
        self.user_command_pub.publish(test_command)
        timeout = rospy.Duration(10)  # Increase timeout to 10 seconds
        start_time = rospy.Time.now()
        while not self.feedback_received and (rospy.Time.now() - start_time) < timeout:
            rospy.sleep(0.1)
        self.assertTrue(self.feedback_received, "No feedback received from manager_agent within 10 seconds")
        self.assertIn("Received command", self.feedback_message, "Unexpected feedback message")

    def test_validate_request_service(self):
        test_request = "Disassemble the frame"
        response = self.validate_request(test_request)
        self.assertIsNotNone(response, "No response from validate_request service")
        self.assertIsInstance(response.is_standard, bool, "is_standard should be a boolean")
        self.assertIsInstance(response.validation_details, str, "validation_details should be a string")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('manager_agent', 'test_manager_agent', TestManagerAgent)
