cmake_minimum_required(VERSION 2.8.3)
project(ltlplanner)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED
             COMPONENTS
             moveit_core
             moveit_ros_planning
             moveit_ros_planning_interface
             pluginlib
             cmake_modules
             geometric_shapes
             geometry_msgs
             move_base_msgs
             actionlib
             pr2_controllers_msgs
             control_msgs
             std_msgs
  					 message_generation
  					 random_numbers
  					 moveit_msgs
  					 tf
  					 actionlib_msgs
  					 move_base_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

add_message_files(
  FILES
  region.msg
  limit.msg
  automaton.msg
  transition.msg
  state.msg
  goal.msg
  automaton_msg.msg
  region_msg.msg
  goal_msg.msg
  plan_msg.msg
  plan.msg
  task.msg
  map_state.msg
)

add_service_files(
  FILES
)

add_action_files(
  DIRECTORY action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  moveit_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    message_runtime
    std_msgs
    geometry_msgs
    moveit_msgs
    actionlib_msgs
)

find_package(Eigen REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(manager src/manager.cpp)
target_link_libraries(manager ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(manager ltlplanner_generate_messages_cpp)
install(TARGETS manager DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pub_goal src/pub_goal.cpp)
target_link_libraries(pub_goal ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pub_goal ltlplanner_generate_messages_cpp)
install(TARGETS pub_goal DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pub_region src/pub_region.cpp)
target_link_libraries(pub_region ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pub_region ltlplanner_generate_messages_cpp)
install(TARGETS pub_region DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})