#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from multi_agent_system.srv import ValidateRequest, ValidateRequestResponse, StabilityAnalysis, StabilityAnalysisResponse
import openai
from openai import OpenAI
from langchain_openai import ChatOpenAI
from langchain.memory import ConversationBufferMemory
from langchain.prompts import ChatPromptTemplate
from langchain.chains import LLMChain
from dotenv import load_dotenv
import os
import sys

# Load environment variables from .env file
load_dotenv()
print(f"OPENAI_API_KEY loaded in Manager Agent: {'Yes' if os.getenv('OPENAI_API_KEY') else 'No'}")

class ManagerAgent:
    def __init__(self):
        try:
            # Initialize ROS node
            rospy.init_node('manager_agent', anonymous=False)

            # Get OpenAI API key from environment variables
            self.openai_api_key = os.getenv('OPENAI_API_KEY')
            if not self.openai_api_key:
                rospy.logerr("OPENAI_API_KEY not found in .env file")
                raise ValueError("OPENAI_API_KEY not set")
            
            # Initialize ChatOpenAI model
            self.llm = ChatOpenAI(temperature=0, model_name="gpt-4o-mini", openai_api_key=self.openai_api_key)

            # Initialize conversation memory
            self.memory = ConversationBufferMemory(return_messages=True)

            # Define the prompt template for the AI
            self.prompt = ChatPromptTemplate.from_template("""
            You are the Manager Agent in a multi-agent robotic system for disassembly tasks. Your role is to coordinate all interactions and assign tasks to other agents. You are responsible for:

            1. Interpreting user commands accurately, even if they are ambiguous or complex.
            2. Maintaining context over multiple interactions using conversation history.
            3. Routing tasks to the Structural Engineer Agent for initial validation.
            4. Communicating results to the user.

            The agent you coordinate is:
            - Structural Engineer Agent: Validates requests against current disassembly manuals using a RAG system.

            Use your advanced natural language understanding to interpret the user's intent and maintain conversation context. Always strive for clear communication and efficient task routing.

            Current conversation:
            {history}
            Human: {human_input}
            AI: Let's process this request step by step:
            1. Interpret the user's intent.
            2. Determine if we need to route this to the Structural Engineer Agent.
            3. Formulate a clear response or action plan.

            Response:
            """)

            # Initialize service proxies to None
            self.validate_request = None
            self.stability_analysis = None

            # Subscribe to the /user_command topic
            self.user_command_sub = rospy.Subscriber('/user_command', String, self.handle_user_command)

            # Publisher for user feedback
            self.user_feedback_pub = rospy.Publisher('/user_feedback', String, queue_size=10)

            # Try to connect to the Structural Engineer Agent and Stability Agent services
            rospy.Timer(rospy.Duration(1), self.try_connect_services)

            rospy.loginfo("Manager Agent initialized successfully.")
        except rospy.ROSInitException as e:
            rospy.logerr(f"Failed to initialize ROS node: {e}")
            sys.exit(1)
        except Exception as e:
            rospy.logerr(f"Error initializing Manager Agent: {e}")
            sys.exit(1)

    def try_connect_services(self, event):
        # Attempt to connect to the Structural Engineer Agent service
        if self.validate_request is None:
            try:
                rospy.wait_for_service('/validate_request', timeout=1)
                self.validate_request = rospy.ServiceProxy('/validate_request', ValidateRequest)
                rospy.loginfo("Connected to /validate_request service")
            except rospy.ROSException:
                rospy.logwarn("Waiting for /validate_request service...")
            except rospy.ROSInterruptException:
                rospy.logwarn("ROS master is not running. Unable to connect to /validate_request service.")

        # Attempt to connect to the Stability Agent service
        if self.stability_analysis is None:
            try:
                rospy.wait_for_service('/stability_analysis', timeout=1)
                self.stability_analysis = rospy.ServiceProxy('/stability_analysis', StabilityAnalysis)
                rospy.loginfo("Connected to /stability_analysis service")
            except rospy.ROSException:
                rospy.logwarn("Waiting for /stability_analysis service...")
            except rospy.ROSInterruptException:
                rospy.logwarn("ROS master is not running. Unable to connect to /stability_analysis service.")

        # Check if ROS master is running
        if not rospy.is_shutdown():
            self.user_feedback_pub.publish("Manager Agent is running and attempting to connect to services.")
        else:
            rospy.logerr("ROS master is not running. Unable to initialize Manager Agent properly.")
            self.user_feedback_pub.publish("Error: ROS master is not running. Please start roscore and try again.")

    def handle_user_command(self, msg):
        try:
            # Process incoming user commands
            user_command = msg.data
            rospy.loginfo(f"[ManagerAgent] Received user command: {user_command}")

            # Always publish an initial acknowledgment
            self.user_feedback_pub.publish(f"Received command: {user_command}. Processing...")

            # Use the language model to interpret the command
            try:
                response = self.llm.invoke(input=user_command)
                interpreted_command = response.content.strip()
                rospy.loginfo(f"[ManagerAgent] Interpreted command: {interpreted_command}")

                # Process the interpreted command
                self.process_command(interpreted_command)
            except Exception as e:
                rospy.logerr(f"[ManagerAgent] Error interpreting command: {e}")
                self.user_feedback_pub.publish(f"Error interpreting command: {e}. Please try again.")
        except rospy.ROSInterruptException:
            rospy.logerr("ROS master is not running. Unable to process user command.")
            self.user_feedback_pub.publish("Error: ROS master is not running. Please start roscore and try again.")
        except Exception as e:
            rospy.logerr(f"[ManagerAgent] Unexpected error handling user command: {e}")
            self.user_feedback_pub.publish(f"An unexpected error occurred. Please try again later.")

    def process_command(self, command):
        # Process the interpreted command and interact with other agents
        if command.lower() == "test system":
            self.run_system_test()
            return

        if self.validate_request is None:
            rospy.logwarn("[ManagerAgent] Structural Engineer Agent service is not available. Retrying connection...")
            self.user_feedback_pub.publish("I'm sorry, but I can't validate your request at the moment. Please try again later.")
            return

        try:
            # Validate the request with Structural Engineer Agent
            rospy.loginfo(f"[ManagerAgent] Sending validation request to Structural Engineer Agent: {command}")
            validation_response = self.validate_request(command)
            rospy.loginfo(f"[ManagerAgent] Received validation response: {validation_response}")

            if validation_response.is_standard:
                rospy.loginfo(f"[ManagerAgent] Request is standard: {validation_response.validation_details}")
                self.user_feedback_pub.publish(f"Your request follows standard procedures. Proceeding with stability analysis.")
            
                # Perform stability analysis
                if self.stability_analysis is None:
                    rospy.logwarn("[ManagerAgent] Stability Agent service is not available.")
                    self.user_feedback_pub.publish("I'm sorry, but I can't perform stability analysis at the moment. Please try again later.")
                else:
                    try:
                        stability_response = self.stability_analysis(command)
                        if stability_response.is_safe:
                            self.user_feedback_pub.publish(f"Stability analysis complete. The task is safe to execute.")
                        else:
                            self.user_feedback_pub.publish(f"Stability analysis complete. The task requires modifications: {stability_response.modifications}")
                    
                        # Add the stability result to the conversation memory
                        self.memory.chat_memory.add_ai_message(f"Stability analysis result: {'Safe' if stability_response.is_safe else 'Unsafe'}")
                    except rospy.ServiceException as e:
                        rospy.logerr(f"[ManagerAgent] Stability analysis service call failed: {e}")
                        self.user_feedback_pub.publish(f"I encountered an error during stability analysis. Please try again.")
                    except Exception as e:
                        rospy.logerr(f"[ManagerAgent] Unexpected error during stability analysis: {e}")
                        self.user_feedback_pub.publish(f"An unexpected error occurred during stability analysis. Please try again later.")
            else:
                rospy.loginfo(f"[ManagerAgent] Request is non-standard: {validation_response.validation_details}")
                self.user_feedback_pub.publish(f"Your request doesn't follow standard procedures. Here's why: {validation_response.validation_details}")
        
            # Add the validation result to the conversation memory
            self.memory.chat_memory.add_ai_message(f"Validation result: {validation_response.validation_details}")
            
        except rospy.ServiceException as e:
            rospy.logerr(f"[ManagerAgent] Service call failed: {e}")
            self.user_feedback_pub.publish(f"I encountered an error while processing your request. Please try again.")

    def run_system_test(self):
        rospy.loginfo("[ManagerAgent] Running system test...")
        self.user_feedback_pub.publish("Running system test...")

        # Test Structural Engineer Agent
        try:
            validation_response = self.validate_request("Test disassembly request")
            self.user_feedback_pub.publish(f"Structural Engineer Agent test result: {validation_response.validation_details}")
        except Exception as e:
            self.user_feedback_pub.publish(f"Structural Engineer Agent test failed: {str(e)}")

        # Test Stability Agent
        try:
            stability_response = self.stability_analysis("Test stability analysis")
            self.user_feedback_pub.publish(f"Stability Agent test result: {'Safe' if stability_response.is_safe else 'Unsafe'}")
        except Exception as e:
            self.user_feedback_pub.publish(f"Stability Agent test failed: {str(e)}")

        self.user_feedback_pub.publish("System test completed.")

if __name__ == '__main__':
    try:
        manager_agent = ManagerAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
