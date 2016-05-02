#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ltlplanner/lowplanAction.h>

#include "geometry_msgs/Point.h"
#include <geometric_shapes/shape_operations.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

class LowPlanner
{
protected:

  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<ltlplanner::lowplanAction> as_; 
  std::string action_name_;
  
  ltlplanner::lowplanFeedback feedback_;
  ltlplanner::lowplanResult result_;

public:

  LowPlanner(std::string name) :
    as_(nh_, name, boost::bind(&LowPlanner::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~LowPlanner(void)
  {
  }

  void executeCB(const ltlplanner::lowplanGoalConstPtr &goal)
  {
    bool success = true;
    bool pathfound = false;
  	feedback_.done = false;
  	
  	if (as_.isPreemptRequested() || !ros::ok())
    {
      as_.setPreempted();
      success = false;
    }
      
    as_.publishFeedback(feedback_);
		
		moveit::planning_interface::MoveGroup group(goal->group_name);
  	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  	ros::Publisher display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  	moveit_msgs::DisplayTrajectory display_trajectory;
  	
  	std::vector<double> group_variable_values;
  	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  
  	group_variable_values[0] = goal->values[0]; 
  	group_variable_values[1] = goal->values[1];
  	group_variable_values[2] = goal->values[2];
  	group.setJointValueTarget(group_variable_values);
		
  	moveit::planning_interface::MoveGroup::Plan my_plan;
  	group.setPlannerId("RRTstarkConfigDefault");
  	group.setPlanningTime(30.0);
  	pathfound = group.plan(my_plan);
  	//Mover el robot
  	if (pathfound){
  		result_.done = true;
  		result_.trajectory = my_plan.trajectory_;
  		success = true;
		}
  	
    if(success)
    {
      as_.setSucceeded(result_);
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "lowplan_srv");

  LowPlanner lplan(ros::this_node::getName());
  ros::spin();

  return 0;
}
