#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include <actionlib/server/simple_action_server.h>
#include <ltlplanner/lowstepAction.h>

class BaseFwdAction
{
protected:

  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<ltlplanner::lowstepAction> as_; 
  std::string action_name_;
  
  ltlplanner::lowstepFeedback feedback_;
  ltlplanner::lowstepResult result_;

  ros::Publisher cmd_vel_pub_;
  
  tf::TransformListener listener_;
  
public:

  BaseFwdAction(std::string name) :
    as_(nh_, name, boost::bind(&BaseFwdAction::executeCB, this, _1), false),
    action_name_(name)
  {
  	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("base_controller/command", 1);
    as_.start();
  }

  ~BaseFwdAction(void)
  {
  }

  void executeCB(const ltlplanner::lowstepGoalConstPtr &goal)
  {
    bool success = true;
		bool done = false;
    feedback_.done = false;
    
    // publish info to the console for the user
    ROS_INFO("%s: Executing, movement sequence", action_name_.c_str());

		listener_.waitForTransform("base_footprint", "odom_combined", ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_footprint", "odom_combined", ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 1.5 m/s
    base_cmd.linear.y = 0.0;
    base_cmd.linear.x = 1.5;
    base_cmd.angular.z = 0.0;
    
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      
      as_.publishFeedback(feedback_);
      
      
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "odom_combined", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform = start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();
			double value = goal->dist;
      if(dist_moved >= value){
       done = true;
     }  
    }
    
    if(success)
    {
      result_.done = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver_fwd");
  ros::NodeHandle nh;
  
  BaseFwdAction base(ros::this_node::getName());
  ros::spin();
  
  return 0; 
}
