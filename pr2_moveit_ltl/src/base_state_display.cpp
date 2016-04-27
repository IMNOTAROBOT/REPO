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
#include <pr2_moveit_ltl/baseState.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "state_display_tutorial");
	ros::NodeHandle nh;
  /* Needed for ROS_INFO commands to work */
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* Load the robot model */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  /* Get a shared pointer to the model */
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  /* Get the configuration for the joints in the right arm of the PR2*/
  const robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("base");

	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

	std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  
  ros::Publisher robot_state_publisher = nh.advertise<pr2_moveit_ltl::baseState>( "base_robot_state", 1 );
	ros::Rate loop_rate(1);
	
	while(ros::ok()){
		
  	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  	pr2_moveit_ltl::baseState msg;
  	msg.joints = joint_values;
  	msg.names = joint_names;
  	robot_state_publisher.publish( msg );
  	
  	joint_values[0] = joint_values[0] + 0.2;
  	kinematic_state->setJointGroupPositions("base", joint_values);
		ros::spinOnce();
    loop_rate.sleep();
	}
  /* loop at 1 Hz */
  
  return 0;
}
