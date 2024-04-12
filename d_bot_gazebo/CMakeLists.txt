cmake_minimum_required(VERSION 3.0.2)
project(d_bot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES d_bot_gazebo
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch model media worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
