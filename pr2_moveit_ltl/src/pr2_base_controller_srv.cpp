#include <iostream>
#include <math.h> 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_moveit_ltl/pr2_base_controller_ltlAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

class BaseAction
{
protected:

  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<pr2_moveit_ltl::pr2_base_controller_ltlAction> as_; 
  
  std::string action_name_;
  
  pr2_moveit_ltl::pr2_base_controller_ltlFeedback feedback_;
  pr2_moveit_ltl::pr2_base_controller_ltlResult result_;
  
public:

  BaseAction(std::string name) :
    as_(nh_, name, boost::bind(&BaseAction::executeCB, this, _1), false),
    action_name_(name)
  {	
    as_.start();
  }

  ~BaseAction(void)
  {
  }

  void executeCB(const pr2_moveit_ltl::pr2_base_controller_ltlGoalConstPtr &goal)
  {
  	/*actionlib::SimpleActionClient<pr2_moveit_ltl::pr2_base_controller_ltlAction> ac_fwd("robot_driver_fwd", true);
  	actionlib::SimpleActionClient<pr2_moveit_ltl::pr2_base_controller_ltlAction> ac_angle("robot_driver_angle", true);
  	ac_fwd.waitForServer(); 
  	ac_angle.waitForServer();
    //Resolver angulo primero
    bool success = true;
    bool doneboth = false;
    feedback_.done = false;
    
    goal->angle = atan2((goal->point.linear.x - goal->origin.linear.x),(goal->point.linear.y - goal->origin.linear.y)); 
    
  	ac_angle.sendGoal(goal);
  	while(ac_angle.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ros::ok())
  	{
  		if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      
      as_.publishFeedback(feedback_);
    	usleep(50000);
    		
  	}
  	double distx = (goal->point.linear.x - goal->origin.linear.x) * (goal->point.linear.x - goal->origin.linear.x);
  	double disty = (goal->point.linear.y - goal->origin.linear.y) * (goal->point.linear.y - goal->origin.linear.y);
  	goal->dist = sqrt(distx + disty);
  	
  	ac_fwd.sendGoal(goal);
  	while(ac_fwd.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ros::ok())
  	{
  		if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        doneboth = false;
        break;
      }
      
      as_.publishFeedback(feedback_);
    	usleep(50000);
    		
  	}
  	
  	if(success && doneboth)
    {
      result_.done = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }*/
  }
};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;
  
  BaseAction base(ros::this_node::getName());
  ros::spin();
  
  return 0; 
}
