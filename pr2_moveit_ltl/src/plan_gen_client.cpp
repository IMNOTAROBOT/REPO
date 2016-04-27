#include "ros/ros.h"
#include "pr2_moveit_ltl/task_queue_msg.h"
#include "pr2_moveit_ltl/automaton_gen_msg.h"
#include "pr2_moveit_ltl/proposition.h"
#include "geometry_msgs/Point.h"
#include "pr2_moveit_ltl/automaton.h"
#include "pr2_moveit_ltl/area_proposition.h"
#include "pr2_moveit_ltl/limit.h"
#include "pr2_moveit_ltl/state.h"
#include "pr2_moveit_ltl/transition.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_client");

  ros::NodeHandle n;
  
  ros::ServiceClient client_a = n.serviceClient<pr2_moveit_ltl::automaton_gen_msg>("automaton_gen_ltl");
  ros::ServiceClient client = n.serviceClient<pr2_moveit_ltl::task_queue_msg>("plan_gen_ltl");
  
  pr2_moveit_ltl::automaton_gen_msg srv_auto;
  srv_auto.request.client = "auto_gen_client_ltl";
  
  pr2_moveit_ltl::automaton automata;
  if (client_a.call(srv_auto))
  {
  	automata = srv_auto.response.automata; 
	  std::cout << "Automata:" << automata.name << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service plan_gen_ltl");
    return 1;
  }
  
  pr2_moveit_ltl::task_queue_msg srv;
  srv.request.name = "task_queue_client_ltl";
  
  srv.request.automata = automata;
  
  pr2_moveit_ltl::proposition props[10];
  bool plan = false;
  int size = 0;
  if (client.call(srv))
  {
    size = srv.response.size;
    int i = 0;
    std::cout << "Tamano:" << size << std::endl;
    for(std::vector<pr2_moveit_ltl::proposition>::const_iterator it = srv.response.propositions.begin(); it != srv.response.propositions.end(); ++it)
		{
			props[i] = *it;
			std::cout << "Objeto:" << i << std::endl;
			std::cout << props[i].name << std::endl;
			std::cout << props[i].routine << std::endl;
			std::cout << props[i].point.x << std::endl;
			std::cout << props[i].point.y << std::endl;
			std::cout << props[i].point.z << std::endl;
			i++;
		}
		ROS_INFO("service plan_gen_ltl on");
		plan = true;
	  
  }
  else
  {
    ROS_ERROR("Failed to call service plan_gen_ltl");
    return 1;
  }

  return 0;
}
