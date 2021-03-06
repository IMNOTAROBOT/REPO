#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_moveit_ltl/pr2_base_controller_ltlAction.h>
#include <geometry_msgs/Twist.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "server_controller_base");

  actionlib::SimpleActionClient<pr2_moveit_ltl::pr2_base_controller_ltlAction> ac_fwd("robot_driver_angle", true);

  ROS_INFO("Waiting for action server to start.");
  ac_fwd.waitForServer(); 

  ROS_INFO("Action server started, sending goal.");
  
  pr2_moveit_ltl::pr2_base_controller_ltlGoal goal;
  goal.point.linear.x = 1.0;
  goal.point.linear.y = 1.0;
  goal.point.angular.z = 0.0;
  goal.dist= 2.0;
  goal.angle = 1.57;
  goal.clock = false;
  ac_fwd.sendGoal(goal);
  

  while(ac_fwd.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ros::ok())
  {
    usleep(50000);
  }
  
  ROS_INFO("Action finished");
  return 0;
}
