#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

/*
* publisher class for PR2 base motion
*/

class pr2_base_control_pub
{
  private:
  //Auxiliary values
  double walk_vel, yaw_rate;
  geometry_msgs::Twist cmd;
  
  //To turn it into a node
  ros::NodeHandle n_;

  //Create a publisher 
  ros::Publisher vel_pub_;

  public:
  void init()
  { 
    //Initial values for message cmd to 0
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
    
    //Initialization of publisher called cmd_vel
    vel_pub_ = n_.advertise<geometry_msgs::Twist>("base_controller/command", 1);

    //Creates a private nodeHandle
    ros::NodeHandle n_private("~");
    //Parameters of node n_private
    n_private.param("walk_vel", walk_vel, 1.5);
    n_private.param("yaw_rate", yaw_rate, 1.5);

  }
  
  ~pr2_base_control_pub()   { }
  void make_public();

};

int main(int argc, char** argv)
{
  //Initialize ROS node publisher
  ros::init(argc, argv, "pr2_base_control_pub");
  
  //Create object pr2_base_control_pub
  pr2_base_control_pub pr2bcpub;
  pr2bcpub.init();
  
  
  while (ros::ok()) {
     pr2bcpub.make_public();
     ros::spinOnce();
  }

  return(0);
}

void pr2_base_control_pub::make_public(){
  cmd.linear.x = cmd.linear.y =  walk_vel;
  cmd.angular.z = yaw_rate;
  vel_pub_.publish(cmd);
}
