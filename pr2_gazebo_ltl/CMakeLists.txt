cmake_minimum_required(VERSION 2.8.3)
project(pr2_gazebo_ltl)
find_package(catkin REQUIRED COMPONENTS
gazebo_ros 
pr2_gazebo 
roscpp 
pr2_controllers_msgs
actionlib
geometry_msgs
std_msgs
tf
topic_tools)

find_package(Boost REQUIRED system filesystem date_time thread)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

add_executable(simple_trajectory src/simple_trajectory.cpp)
target_link_libraries(simple_trajectory ${catkin_LIBRARIES})

add_executable(pr2_base_control_pub src/pr2_base_control_pub.cpp)
target_link_libraries(pr2_base_control_pub ${catkin_LIBRARIES})

add_executable(pr2_r_gripper_control_pub src/pr2_r_gripper_control_pub.cpp)
target_link_libraries(pr2_r_gripper_control_pub ${catkin_LIBRARIES})

