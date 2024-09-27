#!/usr/bin/env python3

import rospy
from multi_agent_system.srv import PlanExecution, PlanExecutionRequest
from std_msgs.msg import String

def test_planning_agent():
    rospy.init_node('test_planning_agent', anonymous=True)
    rospy.wait_for_service('/plan_execution')
    plan_execution_service = rospy.ServiceProxy('/plan_execution', PlanExecution)

    # Define a test plan
    test_plan = "Test plan for disassembly"

    # Call the service
    try:
        response = plan_execution_service(PlanExecutionRequest(plan=test_plan))
        rospy.loginfo(f"Test Plan Execution Response: {response.execution_details}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        test_planning_agent()
    except rospy.ROSInterruptException:
        pass
