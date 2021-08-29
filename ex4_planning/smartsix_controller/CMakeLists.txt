cmake_minimum_required(VERSION 2.8.3)

project(smartsix_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  moveit_dp_redundancy_resolution
  moveit_dp_redundancy_resolution_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES smartsix_controller
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(smartsix_controller src/smartsix_controller.cpp)
target_link_libraries(smartsix_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS smartsix_controller DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})