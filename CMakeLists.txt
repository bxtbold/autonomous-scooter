cmake_minimum_required(VERSION 3.0.2)
project(autonomous-scooter)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scooter
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

## Build ##

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
