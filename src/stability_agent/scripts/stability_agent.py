#!/usr/bin/env python3

import rospy
from multi_agent_system.srv import StabilityAnalysis, StabilityAnalysisResponse
from langchain_openai import ChatOpenAI
from langchain.prompts import ChatPromptTemplate
from langchain.vectorstores import FAISS
from langchain.embeddings import OpenAIEmbeddings
from langchain.text_splitter import CharacterTextSplitter
from langchain.document_loaders import JSONLoader
from dotenv import load_dotenv
import os
import json

load_dotenv()

class StabilityAgent:
    def __init__(self):
        rospy.init_node('stability_agent', anonymous=False)

        # Parameters
        self.openai_api_key = rospy.get_param('/openai_api_key')

        # Initialize ChatOpenAI
        self.llm = ChatOpenAI(temperature=0, model_name="gpt-4o-mini", openai_api_key=self.openai_api_key)

        # Initialize RAG system
        self.initialize_rag_system()

        # Initialize prompt template
        self.prompt = ChatPromptTemplate.from_template("""
        You are the Stability Agent in a multi-agent robotic system for disassembly tasks. Your role is to ensure that requested actions won't compromise structural stability. You are responsible for:

        1. Evaluating the impact of non-standard or complex actions on structural stability.
        2. Suggesting additional steps if necessary to maintain stability.
        3. Using advanced predictive capabilities, including physics simulations and machine learning models.

        Current task: {task}

        Relevant information from past assessments: {context}

        AI: Let's analyze this task for stability risks step by step:
        1. Evaluate the structural risks using simulations and ML models.
        2. Determine if the task is safe to execute as-is.
        3. If not safe, suggest necessary adjustments (e.g., supporting elements).
        4. Provide a detailed explanation of your analysis and recommendations.

        Stability analysis result:
        """)

        # Service to analyze stability
        self.stability_analysis_service = rospy.Service('/stability_analysis', StabilityAnalysis, self.handle_stability_analysis)

        rospy.loginfo("Stability Agent Initialized.")

    def initialize_rag_system(self):
        # Load and process the past stability assessments
        assessments_path = os.path.join(os.path.dirname(__file__), '..', 'data', 'stability_assessments.json')
        
        def extract_data(data):
            return ' '.join([
                data['task_description'],
                ' '.join(data['risk_factors']),
                ' '.join(data['stability_measures']),
                ' '.join(data['outcome'])
            ])

        loader = JSONLoader(
            file_path=assessments_path,
            jq_schema='.assessments[]',
            content_key=None,
            text_content=extract_data
        )

        documents = loader.load()
        text_splitter = CharacterTextSplitter(chunk_size=1000, chunk_overlap=0)
        texts = text_splitter.split_documents(documents)

        # Create vector store
        embeddings = OpenAIEmbeddings(openai_api_key=self.openai_api_key)
        self.vectorstore = FAISS.from_documents(texts, embeddings)

    def handle_stability_analysis(self, req):
        task = req.task
        rospy.loginfo(f"Analyzing stability for task: {task}")

        # Retrieve relevant context from the RAG system
        docs = self.vectorstore.similarity_search(task, k=2)
        context = "\n".join([doc.page_content for doc in docs])

        # Use the LLM to analyze stability
        response = self.llm(self.prompt.format(task=task, context=context))
        analysis = response.content.strip()

        # Determine if the task is safe
        is_safe = "safe to execute" in analysis.lower()
        modifications = analysis if not is_safe else ""

        return StabilityAnalysisResponse(is_safe=is_safe, modifications=modifications)

    def run_physics_simulation(self, task):
        # Placeholder for physics simulation
        # In a real implementation, this would interface with a physics engine or Unity simulation
        rospy.loginfo(f"Running physics simulation for task: {task}")
        # Return simulated results
        return "Simulation results: Task appears stable under normal conditions."

    def update_rag_system(self, task, analysis, outcome):
        # Placeholder for updating the RAG system with new successful task completions
        rospy.loginfo(f"Updating RAG system with new assessment: {task}")
        # In a real implementation, this would add the new assessment to the vector store
        # and potentially retrain or update the embeddings

if __name__ == '__main__':
    try:
        stability_agent = StabilityAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
