#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from termcolor import colored

class AgentResponsesLogger:
    def __init__(self):
        rospy.init_node('agent_responses_logger', anonymous=True)
        
        # Subscribe to topics for each agent
        rospy.Subscriber('/user_feedback', String, self.log_manager_response)
        rospy.Subscriber('/structural_engineer_feedback', String, self.log_structural_engineer_response)
        rospy.Subscriber('/stability_feedback', String, self.log_stability_response)
        
        rospy.loginfo("Agent Responses Logger initialized.")

    def log_manager_response(self, msg):
        self.log_response("Manager Agent", msg.data, 'blue')

    def log_structural_engineer_response(self, msg):
        self.log_response("Structural Engineer Agent", msg.data, 'green')

    def log_stability_response(self, msg):
        self.log_response("Stability Agent", msg.data, 'yellow')

    def log_response(self, agent_name, message, color):
        formatted_message = f"\n{'-'*80}\n{colored(agent_name, color, attrs=['bold'])}: {message}\n{'-'*80}\n"
        print(formatted_message)
        rospy.loginfo(formatted_message)

if __name__ == '__main__':
    try:
        AgentResponsesLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
