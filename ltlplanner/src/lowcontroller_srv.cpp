#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ltlplanner/lowcontrolAction.h>

class LowController
{
protected:

  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<ltlplanner::lowcontrolAction> as_; 
  std::string action_name_;
  
  ltlplanner::lowcontrolFeedback feedback_;
  ltlplanner::lowcontrolResult result_;

public:

  LowController(std::string name) :
    as_(nh_, name, boost::bind(&LowController::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~LowController(void)
  {
  }

  void executeCB(const ltlplanner::lowcontrolGoalConstPtr &goal)
  {
    bool success = true;
    bool pathfound = false;
  	feedback_.done = false;
  	
  	if (as_.isPreemptRequested() || !ros::ok())
    {
      as_.setPreempted();
      success = false;
    }
     
    //Debe llamar a los controladores de bajo nivel para ejecutar un plan  
    as_.publishFeedback(feedback_);
		
  	
    if(success)
    {
      as_.setSucceeded(result_);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lowcontrol_srv");

  LowController lcontrol(ros::this_node::getName());
  ros::spin();

  return 0;
}
