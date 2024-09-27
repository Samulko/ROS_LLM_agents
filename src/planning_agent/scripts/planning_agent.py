#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
from instructor import Instructor
from multi_agent_system.srv import PlanExecution, PlanExecutionResponse
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

        # Translate plan into structured action sequences
        action_sequence = self.translate_plan(plan)

        # Validate action sequence
        if self.validate_action_sequence(action_sequence):
            # Execute preliminary steps
            self.execute_preliminary_steps()

            # Write the JSON file
            json_file_path = self.write_json_file(action_sequence)

            # Report back to the Manager Agent
            success, execution_details = self.execute_actions(action_sequence)
            return PlanExecutionResponse(success=success, execution_details=f"{execution_details}. JSON file created at {json_file_path}")
        else:
            return PlanExecutionResponse(success=False, execution_details="Invalid action sequence.")

    def translate_plan(self, plan):
        # Use Instructor to translate the plan into action sequences
        instructor = Instructor()
        action_sequence = instructor.translate(plan)
        return action_sequence

    def validate_action_sequence(self, action_sequence):
        # Implement validation logic
        # For simplicity, we'll assume the action sequence is valid if not empty
        return bool(action_sequence)

    def execute_preliminary_steps(self):
        # Placeholder for executing preliminary steps
        rospy.loginfo("Executing preliminary steps for safety.")

    def write_json_file(self, action_sequence):
        # Write the action sequence to a JSON file
        json_data = {
            "human_working": True,
            "selected_element": "element3",
            "planning_sequence": action_sequence
        }
        json_file_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'data', 'action_sequence.json')
        with open(json_file_path, 'w') as json_file:
            json.dump(json_data, json_file, indent=4)
        rospy.loginfo(f"Action sequence JSON file created at {json_file_path}")
        return json_file_path
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
