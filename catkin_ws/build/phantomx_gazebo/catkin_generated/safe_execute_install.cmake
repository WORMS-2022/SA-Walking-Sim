execute_process(COMMAND "/home/aleks/catkin_ws/build/phantomx_gazebo/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/aleks/catkin_ws/build/phantomx_gazebo/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
