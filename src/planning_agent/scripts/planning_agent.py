#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
from multi_agent_system.srv import PlanExecution, PlanExecutionResponse
import openai
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

        # Initialize ChatOpenAI
        self.llm = ChatOpenAI(temperature=0, model_name="gpt-4-0125-preview", openai_api_key=self.openai_api_key)

        # Service to execute plans
        self.plan_execution_service = rospy.Service('/plan_execution', PlanExecution, self.handle_plan_execution)

        # Publisher to robot control interface
        self.robot_control_pub = rospy.Publisher('/robot_control_command', String, queue_size=10)

        rospy.loginfo("Planning Agent Initialized and ready to work.")

    def handle_plan_execution(self, req):
        plan = req.plan
        rospy.loginfo(f"Planning Agent: Received plan execution request: {plan}")

        # Translate plan into structured action sequences
        action_sequence = self.translate_plan(plan)

        # Validate action sequence
        if self.validate_action_sequence(action_sequence):
            # Execute preliminary steps
            self.execute_preliminary_steps()

            # Check if additional safety measures are needed
            if "unsafe" in plan.lower() or "modifications" in plan.lower():
                action_sequence = self.add_safety_measures(action_sequence)

            # Write the JSON file
            json_file_path = self.write_json_file(action_sequence)

            # Report back to the Manager Agent
            success, execution_details = self.execute_actions(action_sequence)
            return PlanExecutionResponse(success=success, execution_details=f"{execution_details}. JSON file created at {json_file_path}")
        else:
            return PlanExecutionResponse(success=False, execution_details="Invalid action sequence.")

    def add_safety_measures(self, action_sequence):
        rospy.loginfo("Planning Agent: Adding additional safety measures to the action sequence")
        # Add safety measures to the action sequence
        safety_measures = [
            "implement_temporary_supports",
            "distribute_load_evenly",
            "monitor_stability_continuously"
        ]
        action_sequence["planning_sequence"] = safety_measures + action_sequence["planning_sequence"]
        return action_sequence

    def translate_plan(self, plan):
        rospy.loginfo(f"Planning Agent: Translating plan: {plan}")
        # Use LLM to translate the plan into action sequences
        prompt = f"""
        Translate the following plan into a structured action sequence for disassembling a simple portal frame:
        {plan}

        Use only the following action schemas:
        - move_in_cartesian_path(group, move_distance_x, move_distance_y, move_distance_z)
        - moveto(pose_element, element_name)
        - picking(pose_element, element_name)
        - holding(pose_element, element_name)
        - placing(pose_element, element_name)

        Ensure the action sequence follows the correct order and includes all necessary steps.
        Format the response as a JSON object with the following structure:
        {{
            "human_working": true,
            "selected_element": "element_name",
            "planning_sequence": ["action1", "action2", ...]
        }}
        """
        response = self.llm.invoke(prompt)
        
        try:
            action_sequence = json.loads(response.content)
            rospy.loginfo(f"Planning Agent: Translated plan into action sequence: {action_sequence}")
            return action_sequence
        except json.JSONDecodeError:
            rospy.logerr("Planning Agent: Failed to parse LLM response as JSON")
            return None

    def validate_action_sequence(self, action_sequence):
        if action_sequence is None:
            return False
        
        # Check if all required keys are present
        required_keys = ["human_working", "selected_element", "planning_sequence"]
        if not all(key in action_sequence for key in required_keys):
            rospy.logerr("Planning Agent: Missing required keys in action sequence")
            return False

        # Check if planning_sequence is a list
        if not isinstance(action_sequence["planning_sequence"], list):
            rospy.logerr("Planning Agent: planning_sequence is not a list")
            return False

        # Check if all actions in the sequence are valid
        valid_actions = ["move_in_cartesian_path", "moveto", "picking", "holding", "placing"]
        for action in action_sequence["planning_sequence"]:
            if not any(action.startswith(valid) for valid in valid_actions):
                rospy.logerr(f"Planning Agent: Invalid action in sequence: {action}")
                return False
        rospy.loginfo("Planning Agent: Action sequence validated successfully")
        return True

    def execute_preliminary_steps(self):
        # Placeholder for executing preliminary steps
        rospy.loginfo("Planning Agent: Executing preliminary steps for safety.")

    def write_json_file(self, action_sequence):
        # Write the action sequence to a JSON file
        json_file_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'data', 'action_sequence.json')
        with open(json_file_path, 'w') as json_file:
            json.dump(action_sequence, json_file, indent=4)
        rospy.loginfo(f"Planning Agent: Action sequence JSON file created at {json_file_path}")
        return json_file_path

    def execute_actions(self, action_sequence):
        # Publish actions to the robot control interface
        try:
            for action in action_sequence["planning_sequence"]:
                self.robot_control_pub.publish(action)
                rospy.loginfo(f"Planning Agent: Executed action: {action}")
            return True, "Action sequence executed successfully."
        except Exception as e:
            rospy.logerr(f"Planning Agent: Error executing actions: {e}")
            return False, f"Error executing actions: {e}"

if __name__ == '__main__':
    try:
        planning_agent = PlanningAgent()
        rospy.loginfo("Planning Agent: Ready to receive plan execution requests.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
