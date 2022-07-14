execute_process(COMMAND "/home/worms/Desktop/SA-Walking-Sim/catkin_ws/build/phantomx_gazebo/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/worms/Desktop/SA-Walking-Sim/catkin_ws/build/phantomx_gazebo/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
