execute_process(COMMAND "/home/robotis/Tsen_ws/build/op3_util/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/robotis/Tsen_ws/build/op3_util/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
