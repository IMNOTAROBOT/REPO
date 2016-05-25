#include <ros/ros.h>
#include <ltlplanner/lowplanAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<ltlplanner::lowplanAction> LowPlanClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_lowplan_client");

  //tell the action client that we want to spin a thread by default
  LowPlanClient ac("lowplan_srv", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the lowplan_srv action server to come up");
  }

  ltlplanner::lowplanGoal goal;
	goal.id = 0;
	goal.group_name = "base";
	goal.values.push_back(-2.0);
	goal.values.push_back(-4.0);
	goal.values.push_back(1.0);
	
  ROS_INFO("Sending goal from lowerplanner client 1");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Bien");
  else
    ROS_INFO("Mal");  
    
  goal.id = 1;
	goal.group_name = "base";
	goal.values[0] = 1.0;
	goal.values[1] = 1.0;
	goal.values[2] = 0.0;
	
  ROS_INFO("Sending goal from lowerplanner client 2");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Bien");
  else
    ROS_INFO("Mal");  
    
  goal.id = 2;
	goal.group_name = "base";
	goal.values[0] = -3.0;
	goal.values[1] = 3.0;
	goal.values[2] = 0.5;
	
  ROS_INFO("Sending goal from lowerplanner client 3");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Bien");
  else
    ROS_INFO("Mal");  
    
  return 0;
}
