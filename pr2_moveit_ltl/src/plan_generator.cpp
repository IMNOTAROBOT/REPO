#include <ros/ros.h>
#include "pr2_moveit_ltl/task_queue_msg.h"
#include "pr2_moveit_ltl/proposition.h"
#include "pr2_moveit_ltl/area_proposition.h"
#include "pr2_moveit_ltl/limit.h"
#include "pr2_moveit_ltl/automaton.h"
#include "pr2_moveit_ltl/plan.h"
#include "pr2_moveit_ltl/transition.h"
#include "pr2_moveit_ltl/state.h"
#include "geometry_msgs/Point.h"

/*
* Clase que genera un plan de alto nivel.
*/

class PlanGen
{
private:
	pr2_moveit_ltl::automaton auto_;
  
public:
    
  PlanGen(pr2_moveit_ltl::automaton &automata) 
  {
  	auto_ = automata;  
  }

  ~PlanGen()
  {
   
  }

 void setPlan(pr2_moveit_ltl::plan &plan)
  {
    int act = auto_.initial_state;
    int i = 0;
    int path[10];
    while (auto_.states[act].is_acceptance_state == false && i < 10){
    	path[i] = auto_.states[act].transitions[0].trans_[0];
    	act = auto_.states[act].transitions[0].final_state;
    	std::cout << "path alphabet:" << path[i] << std::endl;
    	i++;
    }
    
    for(int j = 0; j < i; j++){ 
    	pr2_moveit_ltl::propositionPtr prop(new pr2_moveit_ltl::proposition);
    	prop->name = "Walking";
  		prop->routine="NULL";
  		prop->DOF = 2;
  		prop->point.x = auto_.alphabet[path[j]].limits[0].max;
  		prop->point.y = auto_.alphabet[path[j]].limits[1].max;
  		prop->point.z = 0.0;
  		prop->dtimes = 5;
  		
  		plan.propositions.push_back(*prop);
    }
      
  }
  
};

bool generate_task(pr2_moveit_ltl::task_queue_msg::Request  &req,
         pr2_moveit_ltl::task_queue_msg::Response &res)
{
  std::cout << "Generating for:" << req.name << std::endl;
  PlanGen planeador(req.automata) ;
  pr2_moveit_ltl::planPtr plan(new pr2_moveit_ltl::plan);
  planeador.setPlan(*plan);
  
	int i = 0;
	for(std::vector<pr2_moveit_ltl::proposition>::const_iterator it = plan->propositions.begin(); it != plan->propositions.end(); ++it)
	{
   res.propositions.push_back(*it);
   i++;
	}
	res.size = i;
  return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "plan_gen_ltl");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("plan_gen_ltl", generate_task);
  ROS_INFO("Ready to generate plan (task queue).");
  ros::spin();

  return 0;
}

