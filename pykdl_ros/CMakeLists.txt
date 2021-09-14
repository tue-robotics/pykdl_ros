cmake_minimum_required(VERSION 3.0.2)
project(pykdl_ros)

find_package(catkin REQUIRED
  COMPONENTS
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS genpy std_msgs
  DEPENDS
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
