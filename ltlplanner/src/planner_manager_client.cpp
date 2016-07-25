#include <ros/ros.h>
#include <ltlplanner/planningRRTAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<ltlplanner::planningRRTAction> HighPlanClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_highplan_client");

  //tell the action client that we want to spin a thread by default
  HighPlanClient ac("twolevelplanning_srv", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the highplan_srv action server to come up");
  }

  ltlplanner::planningRRTGoal goal;
	goal.id = 0;
	goal.group_name = "base";
	goal.region_id.push_back(-1);
	
  ROS_INFO("Sending goal from highplanner client 1");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Bien");
  else
    ROS_INFO("Mal");  
  
  ltlplanner::planningRRTGoal goalp;
  goalp.id = 1;
	goalp.group_name = "base";
	goalp.region_id.push_back(1);
	
  ROS_INFO("Sending goal from highplanner client 2");
  ac.sendGoal(goalp);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Bien");
  else
    ROS_INFO("Mal");  
  
  ltlplanner::planningRRTGoal goalp2;  
  goalp2.id = 2;
	goalp2.group_name = "base";
	goalp2.region_id.push_back(2);
	
  ROS_INFO("Sending goal from highplanner client 3");
  ac.sendGoal(goalp2);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Bien");
  else
    ROS_INFO("Mal");  
    
  return 0;
}
