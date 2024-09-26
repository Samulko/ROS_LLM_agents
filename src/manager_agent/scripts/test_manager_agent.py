#!/usr/bin/env python3

import unittest
import rospy
from std_msgs.msg import String
from multi_agent_system.srv import ValidateRequest, StabilityAnalysis

class TestManagerAgent(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_manager_agent', anonymous=True)
        self.user_command_pub = rospy.Publisher('/user_command', String, queue_size=10)
        self.feedback_received = False
        self.feedback_message = ""
        rospy.Subscriber('/user_feedback', String, self.feedback_callback)
        rospy.wait_for_service('/validate_request', timeout=10)
        rospy.wait_for_service('/stability_analysis', timeout=10)
        self.validate_request = rospy.ServiceProxy('/validate_request', ValidateRequest)
        self.stability_analysis = rospy.ServiceProxy('/stability_analysis', StabilityAnalysis)

    def feedback_callback(self, data):
        self.feedback_received = True
        self.feedback_message = data.data

    def test_user_command_processing(self):
        test_command = "Disassemble the frame"
        self.user_command_pub.publish(test_command)
        timeout = rospy.Duration(20)  # Increased timeout
        start_time = rospy.Time.now()
        while not self.feedback_received and (rospy.Time.now() - start_time) < timeout:
            rospy.sleep(0.1)
        self.assertTrue(self.feedback_received, "No feedback received from manager_agent within 20 seconds")
        self.assertIn("Received command", self.feedback_message, "Unexpected feedback message")
        
        # Wait for the full processing cycle
        rospy.sleep(10)
        
        # Check for validation and stability analysis results
        self.assertIn("Validation result", self.feedback_message, "No validation result received")
        self.assertIn("Stability analysis", self.feedback_message, "No stability analysis result received")

    def test_validate_request_service(self):
        test_request = "Disassemble the frame"
        response = self.validate_request(test_request)
        self.assertIsNotNone(response, "No response from validate_request service")
        self.assertIsInstance(response.is_standard, bool, "is_standard should be a boolean")
        self.assertIsInstance(response.validation_details, str, "validation_details should be a string")

    def test_stability_analysis_service(self):
        test_request = "Disassemble the frame"
        response = self.stability_analysis(test_request)
        self.assertIsNotNone(response, "No response from stability_analysis service")
        self.assertIsInstance(response.is_safe, bool, "is_safe should be a boolean")
        self.assertIsInstance(response.modifications, str, "modifications should be a string")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('manager_agent', 'test_manager_agent', TestManagerAgent)
