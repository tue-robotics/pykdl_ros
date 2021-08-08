cmake_minimum_required(VERSION 3.0.2)
project(tf2_pykdl_ros)

find_package(catkin REQUIRED
  COMPONENTS
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs pykdl_ros tf2_ros
  DEPENDS
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)

  find_package(catkin REQUIRED COMPONENTS rostest tf2_geometry_msgs)

  add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/rostest/test_tf2_ros_transform.test)
endif()
