#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from multi_agent_system.srv import PlanExecution, PlanExecutionResponse
from multi_agent_system.msg import AgentResponse
import openai
from langchain_community.llms import OpenAI
from langchain_community.vectorstores import FAISS
from langchain_community.embeddings import OpenAIEmbeddings
from langchain_community.agent_toolkits.load_tools import load_tools
from langchain_openai import ChatOpenAI

from dotenv import load_dotenv
import os

load_dotenv()

class PlanningAgent:
    def __init__(self):
        rospy.init_node('planning_agent', anonymous=False)

        # Parameters
        self.openai_api_key = rospy.get_param('/openai_api_key')
        openai.api_key = self.openai_api_key

        # Service to execute plans
        self.plan_execution_service = rospy.Service('/plan_execution', PlanExecution, self.handle_plan_execution)

        # Publisher to robot control interface
        self.robot_control_pub = rospy.Publisher('/robot_control_command', String, queue_size=10)

        rospy.loginfo("Planning Agent Initialized.")

    def handle_plan_execution(self, req):
        plan = req.plan
        rospy.loginfo(f"Planning execution for plan: {plan}")

        # Translate plan into actions
        action_sequence = self.translate_plan(plan)

        # Validate action sequence
        if self.validate_action_sequence(action_sequence):
            # Execute actions
            success, execution_details = self.execute_actions(action_sequence)
            return PlanExecutionResponse(success=success, execution_details=execution_details)
        else:
            return PlanExecutionResponse(success=False, execution_details="Invalid action sequence.")

    def translate_plan(self, plan):
        # Use OpenAI LLM to translate the plan into action sequences
        try:
            response = openai.Completion.create(
                engine="text-davinci-003",
                prompt=f"Translate the following plan into a sequence of low-level robotic actions in JSON format: '{plan}'",
                max_tokens=200
            )
            action_sequence = response.choices[0].text.strip()
            return action_sequence
        except Exception as e:
            rospy.logerr(f"Error translating plan: {e}")
            return ""

    def validate_action_sequence(self, action_sequence):
        # Implement validation logic
        # For simplicity, we'll assume the action sequence is valid if not empty
        return bool(action_sequence)

    def execute_actions(self, action_sequence):
        # Publish actions to the robot control interface
        try:
            self.robot_control_pub.publish(action_sequence)
            rospy.loginfo("Action sequence executed.")
            return True, "Action sequence executed successfully."
        except Exception as e:
            rospy.logerr(f"Error executing actions: {e}")
            return False, f"Error executing actions: {e}"

if __name__ == '__main__':
    try:
        planning_agent = PlanningAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
