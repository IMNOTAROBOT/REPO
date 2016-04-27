#include <ros/ros.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "pr2_moveit_ltl/plan_msg.h"
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_base");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* This sleep is ONLY to allow Rviz to come up */
  sleep(10.0);
  moveit::planning_interface::MoveGroup group("base");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  
  ros::Publisher chatter_pub = node_handle.advertise<pr2_moveit_ltl::plan_msg>("chatter", 1000);

  ros::Publisher collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  while(collision_object_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }

  //Agregar objeto que colisiona
  moveit_msgs::CollisionObject co;
  co.header.frame_id = group.getPlanningFrame();
  co.id= "muros";

  shapes::Mesh* m = shapes::createMeshFromResource("package://pr2_moveit_ltl/models/ENV_6.dae");
  shape_msgs::Mesh co_mesh;
  shapes::ShapeMsg co_mesh_msg;
  shapes::constructMsgFromShape(m,co_mesh_msg);
  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
  co.meshes.resize(1);
  co.meshes[0] = co_mesh;
  co.mesh_poses.resize(1);
  co.mesh_poses[0].position.x = -5.0;
  co.mesh_poses[0].position.y = -5.0;
  co.mesh_poses[0].position.z = 0.0;
  co.mesh_poses[0].orientation.w= 0.7071;
  co.mesh_poses[0].orientation.x= 0.7071 ;
  co.mesh_poses[0].orientation.y= 0.0;
  co.mesh_poses[0].orientation.z= 0.0;

  co.meshes.push_back(co_mesh);
  co.mesh_poses.push_back(co.mesh_poses[0]);
  co.operation = co.ADD;
  
  /* Publish and sleep (to view the visualized results) */
  collision_object_publisher.publish(co);
  ros::WallDuration sleep_time(10.0);
  sleep_time.sleep();

  std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(co);
  planning_scene_interface.addCollisionObjects(collision_objects);
  sleep(10.0);

  
  // First get the current set of joint values for the group.
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  
  group_variable_values[0] = -4.0; 
  //group_variable_values[1] = -4.0;
  //group_variable_values[2] = 1.5;
  group.setJointValueTarget(group_variable_values);
	std::cout << "Vector de joints en el grupo"<< group_variable_values.size() << std::endl;
  moveit::planning_interface::MoveGroup::Plan my_plan;
  group.setPlannerId("RRTstarkConfigDefault");
  group.setPlanningTime(10.0);
  bool success = group.plan(my_plan);
  if (success){
    //group.execute(my_plan);
    moveit_msgs::RobotTrajectory trajState = my_plan.trajectory_;
  	trajectory_msgs::MultiDOFJointTrajectory traj =  trajState.multi_dof_joint_trajectory;
  	
  }
  std::vector<std::string> obj = planning_scene_interface.getKnownObjectNames();
  
if (obj.size() > 0)
  {
    std::cout << std::endl << "-- KNOWN COLLISION OBJECTS --" << std::endl;
    for (int i = 0; i < obj.size(); ++i)
      std::cout << obj[i] << std::endl;
  }
  
  moveit_msgs::RobotState stateTest = my_plan.start_state_;
  sensor_msgs::JointState jointstatetest = stateTest.joint_state;
  std::cout << std::endl << "-- Estado inicial --" << std::endl;
  int size = sizeof(jointstatetest.name)/sizeof(std::string);
  for (int i = 0; i < size; ++i){
      std::cout << jointstatetest.name[i] << std::endl;
      std::cout << jointstatetest.position[i] << std::endl;
      std::cout << jointstatetest.velocity[i] << std::endl;
      //std::cout << jointstatetest.effort[0] << std::endl;
      std::cout << size << std::endl;
  }
  
  moveit_msgs::RobotTrajectory trajState = my_plan.trajectory_;
  trajectory_msgs::JointTrajectory jointtraj = trajState.joint_trajectory;

  int sizeString = sizeof(jointtraj.joint_names)/sizeof(std::string);
  std::cout << sizeString << std::endl;
  
  
  pr2_moveit_ltl::plan_msg msg;
  msg.state_ = my_plan.start_state_;
  msg.traj_ = my_plan.trajectory_;
  chatter_pub.publish(msg);
  
  
  
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
  
ros::spin();
  return 0;
}
