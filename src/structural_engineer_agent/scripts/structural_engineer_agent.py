#!/usr/bin/env python3

import rospy
from multi_agent_system.srv import ValidateRequest, ValidateRequestResponse
import openai
from openai import OpenAI
from langchain.prompts import ChatPromptTemplate
from langchain_community.vectorstores import FAISS
from langchain_openai import OpenAIEmbeddings
from langchain.text_splitter import CharacterTextSplitter
from langchain_community.document_loaders import JSONLoader
from dotenv import load_dotenv
import os
import json

# Load environment variables from .env file
load_dotenv()
print(f"OPENAI_API_KEY loaded: {'Yes' if os.getenv('OPENAI_API_KEY') else 'No'}")

class StructuralEngineerAgent:
    def __init__(self):
        try:
            # Initialize ROS node
            rospy.init_node('structural_engineer_agent', anonymous=False)

            # Get OpenAI API key from environment variables
            self.openai_api_key = os.getenv('OPENAI_API_KEY')
            if not self.openai_api_key:
                raise ValueError("OPENAI_API_KEY not found in environment variables")

            # Initialize OpenAI client
            self.client = OpenAI(api_key=self.openai_api_key)

            # Initialize RAG (Retrieval-Augmented Generation) system
            self.initialize_rag_system()

            # Define the prompt template for the AI
            self.prompt = ChatPromptTemplate.from_template("""
            You are the Structural Engineer Agent in a multi-agent robotic system for disassembly tasks. Your role is to validate requests against current disassembly manuals using a RAG system. You are responsible for:

            1. Validating requests to ensure they follow standard procedures.
            2. Providing detailed validation results to the Manager Agent.

            Current request: {request}

            Relevant information from the manuals: {context}

            AI: Let's validate this request step by step:
            1. Analyze the request and compare it to the provided manual information.
            2. Determine if the request follows standard procedures.
            3. Provide a detailed explanation of your validation process and results.

            Validation result:
            """)

            # Set up ROS service for request validation
            self.validate_request_service = rospy.Service('/validate_request', ValidateRequest, self.handle_validate_request)
            self.structural_engineer_feedback_pub = rospy.Publisher('/structural_engineer_feedback', String, queue_size=10)

            rospy.loginfo("Structural Engineer Agent Initialized.")
        except Exception as e:
            rospy.logerr(f"Error initializing Structural Engineer Agent: {str(e)}")
            raise

    def initialize_rag_system(self):
        try:
            # Initialize the RAG system by loading and processing disassembly manuals
            manual_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'data', 'disassembly_manuals.json')
            rospy.loginfo(f"Manual path: {manual_path}")
            rospy.loginfo(f"File exists: {os.path.exists(manual_path)}")
            
            if not os.path.exists(manual_path):
                raise FileNotFoundError(f"Disassembly manual not found at {manual_path}")

            def extract_data(data):
                # Extract relevant information from the JSON data
                return ' '.join([
                    data['name'],
                    ' '.join(data['description_of_structure']),
                    ' '.join(data['components']),
                    ' '.join(data['disassembly_instructions']),
                    ' '.join(data['safety_notes'])
                ])

            # Load JSON data using Langchain's JSONLoader
            loader = JSONLoader(
                file_path=manual_path,
                jq_schema='.procedures[]',
                content_key=None,
                text_content=False  # Set this to False to handle non-string content
            )

            # Process the loaded documents
            documents = loader.load()
            rospy.loginfo(f"Loaded {len(documents)} documents")

            text_splitter = CharacterTextSplitter(chunk_size=1000, chunk_overlap=50)
            texts = text_splitter.split_documents(documents)

            # Create a vector store for efficient similarity search
            embeddings = OpenAIEmbeddings(openai_api_key=self.openai_api_key)
            self.vectorstore = FAISS.from_documents(texts, embeddings)
            rospy.loginfo("RAG system initialized successfully")
        except Exception as e:
            rospy.logerr(f"Error initializing RAG system: {str(e)}")
            raise

    def handle_validate_request(self, req):
        try:
            # Handle incoming validation requests
            request = req.request
            rospy.loginfo(f"StructuralEngineerAgent: Validating request: {request}")

            # Retrieve relevant context from the RAG system
            docs = self.vectorstore.similarity_search(request, k=2)
            context = "\n".join([doc.page_content for doc in docs])
            rospy.loginfo(f"StructuralEngineerAgent: Retrieved context: {context}")

            # Use the OpenAI client to validate the request
            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {"role": "system", "content": self.prompt.format(request=request, context=context)},
                    {"role": "user", "content": request}
                ]
            )
            validation_details = response.choices[0].message.content
            rospy.loginfo(f"StructuralEngineerAgent: Validation details: {validation_details}")
            self.structural_engineer_feedback_pub.publish(f"Validation result: {validation_details}")

            # Determine if the request follows standard procedures
            is_standard = "follows standard procedures" in validation_details.lower()

            return ValidateRequestResponse(is_standard=is_standard, validation_details=validation_details)
        except Exception as e:
            rospy.logerr(f"StructuralEngineerAgent: Error handling validate request: {str(e)}")
            return ValidateRequestResponse(is_standard=False, validation_details=f"Error: {str(e)}")

if __name__ == '__main__':
    try:
        structural_engineer_agent = StructuralEngineerAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Structural Engineer Agent failed: {str(e)}")
