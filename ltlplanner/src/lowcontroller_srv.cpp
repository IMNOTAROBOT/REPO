#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ltlplanner/lowcontrol_stepAction.h>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <stdlib.h> 
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

/*
* Clase interfaz con move_base para controles de la base
*/

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class LowControllerAction
{
protected:

  ros::NodeHandle nh_;
  ros::NodeHandle client;
  actionlib::SimpleActionServer<ltlplanner::lowcontrol_stepAction> as_;
  MoveBaseClient* ac_;
  std::string action_name_;
  
  ltlplanner::lowcontrol_stepFeedback feedback_;
  ltlplanner::lowcontrol_stepResult result_;
  
public:

  LowControllerAction(std::string name) : 
    as_(nh_, name, boost::bind(&LowControllerAction::executeCB, this, _1), false),
    action_name_(name)
  {
  	ac_ = new MoveBaseClient(client, "move_base", true);
    as_.start(); 
  }

  ~LowControllerAction(void)
  {
    if(ac_ != NULL)
      delete ac_;
  }

  void executeCB(const ltlplanner::lowcontrol_stepGoalConstPtr &goal)
  {
    bool success = true;
    // start executing the action
    
    while(!ac_->waitForServer(ros::Duration(5.0))){
    	ROS_INFO("LOWCONTROL : Waiting for the move_base action server to come up");
  	}

  	move_base_msgs::MoveBaseGoal goalp;

  	goalp.target_pose.header.frame_id = goal->goalPose.header.frame_id;
  	goalp.target_pose.header.stamp = ros::Time::now();

  	goalp.target_pose.pose.position.x = goal->goalPose.pose.position.x;
  	goalp.target_pose.pose.position.y = goal->goalPose.pose.position.y;
  	goalp.target_pose.pose.position.z = goal->goalPose.pose.position.z;
  	goalp.target_pose.pose.orientation.x = goal->goalPose.pose.orientation.x;
  	goalp.target_pose.pose.orientation.y = goal->goalPose.pose.orientation.y;
  	goalp.target_pose.pose.orientation.z = goal->goalPose.pose.orientation.z;
  	goalp.target_pose.pose.orientation.w = goal->goalPose.pose.orientation.w;

  	ROS_INFO("LOWCONTROL : Sending goal to move_base");
  	std::cout << "LOWCONTROL : x =" << goal->goalPose.pose.position.x << std::endl;
  	std::cout << "LOWCONTROL : y =" << goal->goalPose.pose.position.y << std::endl;
  	std::cout << "LOWCONTROL : w =" << goal->goalPose.pose.orientation.w << std::endl;
  	ac_->sendGoal(goalp);

    while(ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ros::ok())
    {
      if (as_.isPreemptRequested() || !ros::ok())
      {
      	feedback_.done = false;
        as_.setPreempted();
        success = false;
        break;
      }
      
      feedback_.done = false;
      as_.publishFeedback(feedback_);
      usleep(50000);
      
    }

    if(success)
    {
      result_.done = feedback_.done;
      as_.setSucceeded(result_);
    }
  }
  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "lowcontroller_step");
	ros::NodeHandle n;
  LowControllerAction lowcontroller(ros::this_node::getName());
  ros::spin();
 
  return 0;
}



