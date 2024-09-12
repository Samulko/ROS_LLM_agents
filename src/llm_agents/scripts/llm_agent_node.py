#!/usr/bin/env python3

import rospy
import openai
from llm_agents.msg import LLMQuery
from dotenv import load_dotenv
import os
from std_msgs.msg import String

# Load environment variables from .env file
dotenv_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), '.env')
load_dotenv(dotenv_path)

class LLMAgentNode:
    def __init__(self):
        rospy.init_node('llm_agent', anonymous=True)
        self.agent_id = rospy.get_param('~agent_id', 'default_agent')
        self.sub = rospy.Subscriber('llm_query', LLMQuery, self.query_callback)
        self.response_pub = rospy.Publisher(f'llm_response_{self.agent_id}', String, queue_size=10)
        
        # Initialize OpenAI client
        self.client = openai.OpenAI()

        rospy.loginfo(f"LLM Agent {self.agent_id} initialized and ready to rock!")
        rospy.loginfo(f"Publishing responses on topic: llm_response_{self.agent_id}")

    def query_callback(self, data):
        rospy.loginfo(f"Received message: {data}")
        if data.agent_id == self.agent_id:
            rospy.loginfo(f"Received query for agent {self.agent_id}: {data.query}")
            response = self.generate_response(data.query)
            rospy.loginfo(f"Generated response: {response}")
            self.response_pub.publish(response)
            rospy.loginfo(f"Published response on topic: llm_response_{self.agent_id}")
        else:
            rospy.loginfo(f"Received query for different agent: {data.agent_id}")

    def generate_response(self, query):
        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a helpful assistant."},
                    {"role": "user", "content": query}
                ]
            )
            return response.choices[0].message.content
        except Exception as e:
            rospy.logerr(f"Error generating response: {str(e)}")
            return f"Error: Unable to generate response"

if __name__ == '__main__':
    try:
        node = LLMAgentNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
