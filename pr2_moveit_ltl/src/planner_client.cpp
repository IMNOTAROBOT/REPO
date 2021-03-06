#include <ros/ros.h>
#include <cstdlib>

#include "pr2_moveit_ltl/task_queue_msg.h"
#include "pr2_moveit_ltl/automaton_gen_msg.h"
#include "pr2_moveit_ltl/proposition.h"
#include "pr2_moveit_ltl/automaton.h"
#include "pr2_moveit_ltl/area_proposition.h"
#include "pr2_moveit_ltl/limit.h"
#include "pr2_moveit_ltl/state.h"
#include "pr2_moveit_ltl/transition.h"
#include "geometry_msgs/Point.h"

#include <geometric_shapes/shape_operations.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_moveit_ltl/pr2_base_controller_ltlAction.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "task_queue_client_ltl");
  ros::NodeHandle node_handle;
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(10.0);
  
  ros::ServiceClient client_a = node_handle.serviceClient<pr2_moveit_ltl::automaton_gen_msg>("automaton_gen_ltl");
  ros::ServiceClient client_p = node_handle.serviceClient<pr2_moveit_ltl::task_queue_msg>("plan_gen_ltl");
  
  //Base controller
  actionlib::SimpleActionClient<pr2_moveit_ltl::pr2_base_controller_ltlAction> ac("robot_driver", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); 
  ROS_INFO("Action server started, sending goal.");
  
  //Getting automata
  pr2_moveit_ltl::automaton_gen_msg srv_auto;
  srv_auto.request.client = "auto_gen_client_ltl";
  
  pr2_moveit_ltl::automaton automata;
  if (client_a.call(srv_auto))
  {
  	automata = srv_auto.response.automata; 
	  std::cout << "Automata:" << automata.name << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service plan_gen_ltl");
    return 1;
  }
  
  pr2_moveit_ltl::task_queue_msg srv;
  srv.request.name = "task_queue_client_ltl";
  srv.request.automata = automata;
  
  pr2_moveit_ltl::proposition props[10];
  bool plan = false;
  int size = 0;
  if (client_p.call(srv))
  {
    size = srv.response.size;
    int i = 0;
    for(std::vector<pr2_moveit_ltl::proposition>::const_iterator it = srv.response.propositions.begin(); it != srv.response.propositions.end(); ++it)
	{
		props[i] = *it;
		i++;
	}
	ROS_INFO("service plan_gen_ltl on");
	plan = true;
	  
  }
  else
  {
    ROS_ERROR("Failed to call service plan_gen_ltl");
    return 1;
  }
  
  //Para planear
  moveit::planning_interface::MoveGroup group("base");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  //Collision objects
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
  
  /*
  * Para planear
  */
  
  if(plan){
  	
		for(int j = 0; j < size; j++){
			 // First get the current set of joint values for the group.
  		std::vector<double> group_variable_values;
  		group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  
  		// Now, let's modify one of the joints, plan to the new joint
  		// space goal and visualize the plan.
  		group_variable_values[0] = props[j].point.x; 
  		group_variable_values[1] = props[j].point.y;
  		group_variable_values[2] = props[j].point.z;
  		group.setJointValueTarget(group_variable_values);

  		moveit::planning_interface::MoveGroup::Plan my_plan;
  		group.setPlannerId("RRTstarkConfigDefault");
  		group.setPlanningTime(20.0);
  		bool success = group.plan(my_plan);
  		//Mover el robot
  		if (success){
  			trajectory_msgs::MultiDOFJointTrajectoryPoint points_plan[20];
  			int k = 0;
  			moveit_msgs::RobotTrajectory robotTraj = my_plan.trajectory_;
    		trajectory_msgs::MultiDOFJointTrajectory multi_traj = robotTraj.multi_dof_joint_trajectory;
    		for(std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::const_iterator it = multi_traj.points.begin(); it != multi_traj.points.end(); ++it){
    			points_plan[k] = *it;
    			k++;
				}
				
				for(int iter=0; iter<k; iter++){
					std::cout << "ITER:" << iter << std::endl;
					geometry_msgs::Vector3 value = points_plan[iter].transforms.at(0).translation;
					geometry_msgs::Quaternion quat = points_plan[iter].transforms.at(0).rotation;
    			
    			pr2_moveit_ltl::pr2_base_controller_ltlGoal goal;
  				goal.point.linear.x = value.x;
  				goal.point.linear.y = value.y;
  				goal.point.angular.z = quat.z;
    			ac.sendGoal(goal);
  				while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ros::ok())
  				{
    				usleep(50000);
  				}
  				std::cout << "Complete..."<< std::endl;
				}
				
  		}
	    sleep(10.0);
		}
  }
  ros::spin();
  return 0;
}
