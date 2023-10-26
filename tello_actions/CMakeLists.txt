cmake_minimum_required(VERSION 3.0.2)
project(tello_actions)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  std_msgs
  tello_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tello_actions
#  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp std_msgs tello_bridge
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tello_takeoff_client src/takeoff_action.cpp)
add_executable(tello_land_client src/land_action.cpp)

target_link_libraries( 
  tello_takeoff_client
  ${catkin_LIBRARIES}
)
target_link_libraries( 
  tello_land_client
  ${catkin_LIBRARIES}
)