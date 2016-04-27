#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

// PI
#include <boost/math/constants/constants.hpp>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "state_display_tutorial");

  /* Needed for ROS_INFO commands to work */
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* Load the robot model */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  /* Get a shared pointer to the model */
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  const robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("base");

  ros::NodeHandle nh;
  ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>( "tutorial_robot_state", 1 );

  /* loop at 1 Hz */
  ros::Rate loop_rate(1);

  for (int cnt=0; cnt<5 && ros::ok(); cnt++)
  {
    kinematic_state->setToRandomPositions(joint_model_group);

    /* get a robot state message describing the pose in kinematic_state */
    moveit_msgs::DisplayRobotState msg;
    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

    /* send the message to the RobotState display */
    std::cout << "Num:" << cnt << std::endl;
    robot_state_publisher.publish( msg );

    /* let ROS send the message, then wait a while */
    ros::spinOnce();
    loop_rate.sleep();
  }



 

  ros::shutdown();
  return 0;
  
}
