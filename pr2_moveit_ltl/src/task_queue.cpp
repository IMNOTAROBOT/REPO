#include <ros/ros.h>
#include "pr2_moveit_ltl/task_queue_msg.h"
#include "pr2_moveit_ltl/proposition.h"
#include "geometry_msgs/Point.h"
/*
* Clase que genera un plan de alto nivel.
*/

bool generate_task(pr2_moveit_ltl::task_queue_msg::Request  &req,
         pr2_moveit_ltl::task_queue_msg::Response &res)
{
  pr2_moveit_ltl::proposition pro_1;
  pro_1.name = "Walking";
  pro_1.routine="NULL";
  pro_1.DOF = 2;
  pro_1.point.x = 1.0;
  pro_1.point.y = 4.0;
  pro_1.point.z = 0.0;
  pro_1.dtimes = 5;
  
  pr2_moveit_ltl::proposition pro_2;
  pro_2.name = "Walking";
  pro_2.routine="NULL";
  pro_2.DOF = 2;
  pro_2.point.x = -4.0;
  pro_2.point.y = -4.0;
  pro_2.point.z = 0.0;
  pro_2.dtimes = 5;
   
  res.size = 2;
  res.propositions.push_back(pro_1);
  res.propositions.push_back(pro_2);
  
  return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "task_queue_genltl");
	ros::NodeHandle n;
	
	//Recibimos automata
	
	ros::ServiceServer service = n.advertiseService("task_queue_genltl", generate_task);
  ROS_INFO("Ready to generate plan (task queue).");
  ros::spin();

  return 0;
}


