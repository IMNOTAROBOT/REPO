cmake_minimum_required(VERSION 2.8.3)
project(pr2_moveit_ltl)

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
  proposition.msg
  area_proposition.msg
  limit.msg
  automaton.msg
  transition.msg
  state.msg
  plan.msg
  plan_msg.msg
  baseState.msg
)

add_service_files(
  FILES
  task_queue_msg.srv
  automaton_gen_msg.srv
)

add_action_files(
  DIRECTORY action
  FILES pr2_base_controller_ltl.action
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

add_executable(base_controller src/base_controller.cpp)
target_link_libraries(base_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS base_controller DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_base_planner src/move_base_planner.cpp)
target_link_libraries(move_base_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(move_base_planner pr2_moveit_ltl_generate_messages_cpp)
install(TARGETS move_base_planner DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_arm_planner src/move_arm_planner.cpp)
target_link_libraries(move_arm_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_arm_planner DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pr2_moveit_ltl_planner src/pr2_moveit_ltl_planner.cpp)
target_link_libraries(pr2_moveit_ltl_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pr2_moveit_ltl_planner DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(controller_example src/controller_example.cpp)
target_link_libraries(controller_example ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS controller_example DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(automata_generator src/automata_generator.cpp)
target_link_libraries(automata_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(automata_generator pr2_moveit_ltl_gencpp)
install(TARGETS automata_generator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(task_queue src/task_queue.cpp)
target_link_libraries(task_queue ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(task_queue pr2_moveit_ltl_gencpp)
install(TARGETS task_queue DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(plan_generator src/plan_generator.cpp)
target_link_libraries(plan_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(plan_generator pr2_moveit_ltl_generate_messages_cpp)
install(TARGETS plan_generator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(task_queue_client src/task_queue_client.cpp)
target_link_libraries(task_queue_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS task_queue_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(plan_gen_client src/plan_gen_client.cpp)
target_link_libraries(plan_gen_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS plan_gen_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planner_client src/planner_client.cpp)
target_link_libraries(planner_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(planner_client ${pr2_moveit_ltl_EXPORTED_TARGETS})
install(TARGETS planner_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pr2_base_controller src/pr2_base_controller.cpp)
target_link_libraries(pr2_base_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pr2_base_controller DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pr2_base_controller_srv_fwd src/pr2_base_controller_srv_fwd.cpp)
target_link_libraries(pr2_base_controller_srv_fwd ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pr2_base_controller_srv_fwd ${pr2_moveit_ltl_EXPORTED_TARGETS})
install(TARGETS pr2_base_controller_srv_fwd DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pr2_base_controller_srv_angle src/pr2_base_controller_srv_angle.cpp)
target_link_libraries(pr2_base_controller_srv_angle ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pr2_base_controller_srv_angle ${pr2_moveit_ltl_EXPORTED_TARGETS})
install(TARGETS pr2_base_controller_srv_angle DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pr2_base_controller_srv src/pr2_base_controller_srv.cpp)
target_link_libraries(pr2_base_controller_srv ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pr2_base_controller_srv ${pr2_moveit_ltl_EXPORTED_TARGETS})
install(TARGETS pr2_base_controller_srv DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pr2_base_controller_client src/pr2_base_controller_client.cpp)
target_link_libraries(pr2_base_controller_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pr2_base_controller_client ${pr2_moveit_ltl_EXPORTED_TARGETS})
install(TARGETS pr2_base_controller_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(navegate_base src/navegate_base.cpp)
target_link_libraries(navegate_base ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS navegate_base DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(state_display src/state_display.cpp)
target_link_libraries(state_display ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS state_display DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(base_controller_display src/base_controller_display.cpp)
target_link_libraries(base_controller_display ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS base_controller_display DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(base_state_display src/base_state_display.cpp)
target_link_libraries(base_state_display ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(base_state_display pr2_moveit_ltl_generate_messages_cpp)
install(TARGETS base_state_display DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

