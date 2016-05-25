#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ltlplanner/lowplanAction.h>
#include <map>
#include "geometry_msgs/Point.h"
#include <geometric_shapes/shape_operations.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <geometry_msgs/PoseStamped.h>
#include <ltlplanner/lowcontrol_stepAction.h>
#include <actionlib/client/simple_action_client.h>

#include <gazebo_msgs/ModelState.h>
/*
* Planeador de bajo nivel que utiliza moveit para planear
*/

typedef actionlib::SimpleActionClient<ltlplanner::lowcontrol_stepAction> LowControlClient;

class LowPlanner
{
protected:

  ros::NodeHandle nh_;
  ros::NodeHandle client;
  actionlib::SimpleActionServer<ltlplanner::lowplanAction> as_; 
  std::string action_name_;
  ros::Publisher display_publisher;
  
  ltlplanner::lowplanFeedback feedback_;
  ltlplanner::lowplanResult result_;
  
  ros::Subscriber sub_pose;
  geometry_msgs::PoseStamped actPose_;

	//LowControlClient* ac_;
	moveit::planning_interface::MoveGroup* group;
	moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;  
	std::vector<double> current_state;
	ros::Publisher collision_object_publisher;	
public:

  LowPlanner(std::string name) :
    as_(nh_, name, boost::bind(&LowPlanner::executeCB, this, _1), false),
    action_name_(name)
  {
  	current_state.push_back(0.0);
    current_state.push_back(0.0);
    current_state.push_back(0.0);
  	sub_pose = nh_.subscribe("/ltlplanner/PR2pose", 1000, &LowPlanner::getPR2pose, this);
  	display_publisher = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1, true);
  	//ac_ = new LowControlClient(client,"lowcontroller_step", true);
 		//current_state = new(std::vector<double>);
    group = new moveit::planning_interface::MoveGroup("base");
    group->startStateMonitor();
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    
    collision_object_publisher = nh_.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  	while(collision_object_publisher.getNumSubscribers() < 1)
  	{
    	ros::WallDuration sleep_t(0.5);
    	sleep_t.sleep();
  	}
    
    //Agregar objeto que colisiona
  	moveit_msgs::CollisionObject co;
  	std::string frameplan = group->getPlanningFrame();
  	std::cout << "Frame de planeacion: "<< frameplan <<std::endl;
  	co.header.frame_id = group->getPlanningFrame();
  	co.id= "muros";

  	shapes::Mesh* m = shapes::createMeshFromResource("package://ltlplanner/models/ENV_6.dae");
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
  	planning_scene_interface->addCollisionObjects(collision_objects);
  	sleep(10.0);
    
    as_.start();
  }

  ~LowPlanner(void){
  	/*if(ac_ != NULL){
      delete ac_;
    }*/
  	if(group != NULL){
      delete group;
    }
    if(planning_scene_interface != NULL){
      delete planning_scene_interface;
    }
    /*if(current_state != NULL){
    	delete current_state;
    }*/
  }

  void executeCB(const ltlplanner::lowplanGoalConstPtr &goal)
  {
    bool success = true;
    bool pathfound = false;
  	feedback_.done = false;
  	
    /*while(!ac_->waitForServer(ros::Duration(5.0))){
    	ROS_INFO("LOWPLANNER : Waiting for lowcontroler_step action server to come up");
  	}*/
  	
  	
  	std::vector<double> group_variable_values;
  	group->getCurrentState()->copyJointGroupPositions(group->getCurrentState()->getRobotModel()->getJointModelGroup(group->getName()), group_variable_values);
  	
  	//Imprimir valores act de estado de robotmodel
  	for(std::vector<double>::const_iterator it = group_variable_values.begin(); it != group_variable_values.end(); ++it){
  		double aux = *it;
  		std::cout << "Edo actual:"<< aux <<std::endl;
  	}
  
  	group_variable_values[0] = goal->values[0]; //x
  	group_variable_values[1] = goal->values[1]; //y
  	group_variable_values[2] = goal->values[2]; //w
  	group->setJointValueTarget(group_variable_values);
		
  	moveit::planning_interface::MoveGroup::Plan my_plan;
  	group->setPlannerId("RRTstarkConfigDefault");
  	group->setPlanningTime(30.0);
  	
  	ROS_INFO("Reference frame: %s", group->getPlanningFrame().c_str());
  	//Actualizar state inicial del robot
  	
  	std::cout << "paso 1 " <<std::endl;
  	robot_state::RobotState robotstart(*group->getCurrentState());
  
  	std::vector<double> act_state;
  	group->getCurrentState()->copyJointGroupPositions(group->getCurrentState()->getRobotModel()->getJointModelGroup(group->getName()), act_state);
  	act_state[0] = current_state[0];
  	act_state[1] = current_state[1];
  	act_state[2] = current_state[2];
  	
  	std::cout << "paso 2 " <<std::endl;
  	robotstart.setJointPositions("virtual_joint", act_state);
  	std::cout << "paso 3 " <<std::endl;
  	group->setStartState(robotstart);
  	
  	std::vector<double> group_change;
  	group->getCurrentState()->copyJointGroupPositions(group->getCurrentState()->getRobotModel()->getJointModelGroup(group->getName()), group_change);
  	
  	//Imprimir valores act de estado de robotmodel
  	for(std::vector<double>::const_iterator it = group_change.begin(); it != group_change.end(); ++it){
  		double aux = *it;
  		std::cout << "Edo cambiado:"<< aux <<std::endl;
  	}
  	
  	pathfound = group->plan(my_plan);
  	//Mover el robot
  	if (pathfound){
  		moveit_msgs::RobotTrajectory rTraj = my_plan.trajectory_;
  		trajectory_msgs::MultiDOFJointTrajectory DOFtraj =  rTraj.multi_dof_joint_trajectory;
  		bool controlExe = false;
  		int k = 0;
  		for(std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::const_iterator it = DOFtraj.points.begin(); it != DOFtraj.points.end(); ++it){
  			trajectory_msgs::MultiDOFJointTrajectoryPoint aux = *it;
  				
  			if (as_.isPreemptRequested() || !ros::ok()){
     			feedback_.done = false;
       		as_.setPreempted();
       		success = false;
       		break;
       	}
  			
  			feedback_.done = false;
      	as_.publishFeedback(feedback_);
      	
  			ltlplanner::lowcontrol_stepGoal goalp;

  			goalp.goalPose.header.frame_id = "map";
  			goalp.goalPose.header.stamp = ros::Time::now();

  			goalp.goalPose.pose.position.x = aux.transforms[0].translation.x;
  			goalp.goalPose.pose.position.y = aux.transforms[0].translation.y;
  			goalp.goalPose.pose.position.z = aux.transforms[0].translation.z;
  			goalp.goalPose.pose.orientation.x = aux.transforms[0].rotation.x;
  			goalp.goalPose.pose.orientation.y = aux.transforms[0].rotation.y;
  			goalp.goalPose.pose.orientation.z = aux.transforms[0].rotation.z;
  			goalp.goalPose.pose.orientation.w = aux.transforms[0].rotation.w;
				
				std::cout << "LOWPLANNER : Paso = " << k <<std::endl;
  			std::cout << "LOWPLANNER : x =" << goalp.goalPose.pose.position.x << std::endl;
  			std::cout << "LOWPLANNER : y =" << goalp.goalPose.pose.position.y << std::endl;
  			std::cout << "LOWPLANNER : w =" << goalp.goalPose.pose.orientation.w << std::endl;
  			
  			gazebo_msgs::ModelState state;
  			state.model_name = "pr2";
  			state.pose.position.x = goalp.goalPose.pose.position.x;
  			state.pose.position.y = goalp.goalPose.pose.position.y;
  			state.pose.position.z = goalp.goalPose.pose.position.z;
  			state.pose.orientation.x = goalp.goalPose.pose.orientation.x;
  			state.pose.orientation.y = goalp.goalPose.pose.orientation.y;
  			state.pose.orientation.z = goalp.goalPose.pose.orientation.z;
  			state.pose.orientation.w = goalp.goalPose.pose.orientation.w;
  			//state.reference_frame = "base_footprint";
  			display_publisher.publish(state);
  			sleep(1.0);
  			//ac_->sendGoal(goalp);
  			//ac_->waitForResult();
  			
  			/*if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    			ROS_INFO("LOWPLANNER : Bien");
  			else
    			ROS_INFO("LOWPLANNER : Mal"); */
  			
  			k ++;
  			as_.publishFeedback(feedback_);
  		}
		}
  	
    if(success)
    {
      result_.done = true;
  		result_.trajectory = my_plan.trajectory_;
  		as_.setSucceeded(result_);
    }
  }
  
  void getPR2pose(const geometry_msgs::PoseStamped::ConstPtr& msg){
  		actPose_ = *msg;
  		current_state[0] = actPose_.pose.position.x;
  		current_state[1] = actPose_.pose.position.y;
  		current_state[2] = actPose_.pose.orientation.y;
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lowplan_srv");

  LowPlanner lplan(ros::this_node::getName());
  ros::spin();

  return 0;
}
