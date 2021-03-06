#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>

class pr2_r_gripper_control_pub
{
private:
  int value;
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  pr2_controllers_msgs::Pr2GripperCommand open_cmd_;
  pr2_controllers_msgs::Pr2GripperCommand close_cmd_;

public:
  pr2_r_gripper_control_pub()
  {
    ros::NodeHandle private_nh_("~");

    private_nh_.param("open_position",   open_cmd_.position,     0.08);
    private_nh_.param("open_max_effort", open_cmd_.max_effort,  -1.0);

    private_nh_.param("close_position",   close_cmd_.position,  -100.00);
    private_nh_.param("close_max_effort", close_cmd_.max_effort,   -1.0);

    pub_ = nh_.advertise<pr2_controllers_msgs::Pr2GripperCommand>("r_gripper_controller/command", 1, false);
    value = 1;
  }

  void r_gripper_action()
  {
    
    if (value == 1) {
      pub_.publish(open_cmd_);
      value = 1;
    }
    else {
      pub_.publish(close_cmd_);
      value = 1;
    }
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_r_gripper_control_pub");
  //ros::Rate loop_rate(1000);
  pr2_r_gripper_control_pub pr2rgcp;

  while (ros::ok()) {
     pr2rgcp.r_gripper_action();
     ros::spinOnce();
     //loop_rate.sleep();
  }
  return(0);
}

