execute_process(COMMAND "/home/samko/ros_llm_ws/build/llm_agents/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/samko/ros_llm_ws/build/llm_agents/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
