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
             move_base
             actionlib
             pr2_controllers_msgs
             control_msgs
             std_msgs
  					 message_generation
  					 random_numbers
  					 moveit_msgs
  					 tf
  					 actionlib_msgs
  					 gazebo_msgs
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
  edge.msg
  node.msg
  position.msg
  ROI_edge.msg
  ROI_graph.msg
  ROI_node.msg
  object.msg
)

add_service_files(
  FILES
  RM_inRegion.srv
  RM_getRegionwithPoint.srv
  RM_getNeighboursofRegion.srv
  RM_rdmPointinRegion.srv
  RM_middlePointofRegion.srv
  RM_worldLimits.srv
  RM_getAllRegions.srv
  RM_ROIgraphGen.srv
)

add_action_files(
  DIRECTORY action
  FILES lowplan.action
  lowcontrol.action
  lowcontrol_step.action
  highplan.action
  planningRRT.action
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

add_executable(manager_levels src/manager_levels.cpp)
target_link_libraries(manager_levels ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(manager_levels ltlplanner_generate_messages_cpp)
install(TARGETS manager_levels DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pub_goal src/pub_goal.cpp)
target_link_libraries(pub_goal ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pub_goal ltlplanner_generate_messages_cpp)
install(TARGETS pub_goal DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pub_region src/pub_region.cpp)
target_link_libraries(pub_region ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pub_region ltlplanner_generate_messages_cpp)
install(TARGETS pub_region DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pub_automaton src/pub_automaton.cpp)
target_link_libraries(pub_automaton ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pub_automaton ltlplanner_generate_messages_cpp)
install(TARGETS pub_automaton DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pub_automaton_2 src/pub_automaton_2.cpp)
target_link_libraries(pub_automaton_2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pub_automaton_2 ltlplanner_generate_messages_cpp)
install(TARGETS pub_automaton_2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planner_manager src/planner_manager.cpp)
target_link_libraries(planner_manager ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(planner_manager ltlplanner_generate_messages_cpp)
install(TARGETS planner_manager DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planner_manager_client src/planner_manager_client.cpp)
target_link_libraries(planner_manager_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(planner_manager_client ltlplanner_generate_messages_cpp)
install(TARGETS planner_manager_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(highplanner_srv src/highplanner_srv.cpp)
target_link_libraries(highplanner_srv ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(highplanner_srv ${ltlplanner_EXPORTED_TARGETS})
install(TARGETS highplanner_srv DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(lowplanner_srv src/lowplanner_srv.cpp)
target_link_libraries(lowplanner_srv ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(lowplanner_srv ${ltlplanner_EXPORTED_TARGETS})
install(TARGETS lowplanner_srv DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(lowcontroller_srv src/lowcontroller_srv.cpp)
target_link_libraries(lowcontroller_srv ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(lowcontroller_srv ${ltlplanner_EXPORTED_TARGETS})
install(TARGETS lowcontroller_srv DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(lowcontroller_client src/lowcontroller_client.cpp)
target_link_libraries(lowcontroller_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(lowcontroller_client ${ltlplanner_EXPORTED_TARGETS})
install(TARGETS lowcontroller_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(lowplanner_client src/lowplanner_client.cpp)
target_link_libraries(lowplanner_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(lowplanner_client ${ltlplanner_EXPORTED_TARGETS})
install(TARGETS lowplanner_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(highplanner_client src/highplanner_client.cpp)
target_link_libraries(highplanner_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(highplanner_client ${ltlplanner_EXPORTED_TARGETS})
install(TARGETS highplanner_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pose_publisher src/pose_publisher.cpp)
target_link_libraries(pose_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pose_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(region_manager_srv src/region_manager_srv.cpp)
target_link_libraries(region_manager_srv ${catkin_LIBRARIES})
add_dependencies(region_manager_srv ltlplanner_gencpp)
install(TARGETS region_manager_srv DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(region_manager_client src/region_manager_client.cpp)
target_link_libraries(region_manager_client ${catkin_LIBRARIES})
add_dependencies(region_manager_client ltlplanner_gencpp)
install(TARGETS region_manager_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
