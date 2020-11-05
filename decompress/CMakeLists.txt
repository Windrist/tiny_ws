cmake_minimum_required(VERSION 3.0.3)
project(decompress)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  roslib
  roslaunch
)

catkin_package()
roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(decode src/decode.cpp)
target_link_libraries(decode ${catkin_LIBRARIES})

#install(DIRECTORY  launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
