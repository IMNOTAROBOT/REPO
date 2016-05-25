#include <ros/ros.h>
#include <ltlplanner/lowcontrol_stepAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<ltlplanner::lowcontrol_stepAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("lowcontroller_step", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the lowcontroller_step action server to come up");
  }

  ltlplanner::lowcontrol_stepGoal goal;

  goal.goalPose.header.frame_id = "map";
  goal.goalPose.header.stamp = ros::Time::now();

  goal.goalPose.pose.position.x = 0.2;
  goal.goalPose.pose.position.y = 0.0;
  goal.goalPose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Bien");
  else
    ROS_INFO("Mal");
    
  goal.goalPose.pose.position.x = 0.4;
  goal.goalPose.pose.position.y = 0.0;
  goal.goalPose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Bien");
  else
    ROS_INFO("Mal");  
    
  goal.goalPose.pose.position.x = 0.5;
  goal.goalPose.pose.position.y = 0.0;
  goal.goalPose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Bien");
  else
    ROS_INFO("Mal");  

  return 0;
}
