#include <ros/ros.h>

#include "pr2_moveit_ltl/automaton_gen_msg.h"
#include "pr2_moveit_ltl/automaton.h"
#include "pr2_moveit_ltl/area_proposition.h"
#include "pr2_moveit_ltl/limit.h"
#include "pr2_moveit_ltl/state.h"
#include "pr2_moveit_ltl/transition.h"
#include <cstdlib>

/*
* Clase que genera un plan de alto nivel.
*/

class AutomataGen
{
private:
  
public:
  
  AutomataGen() 
  {
    
  }

  ~AutomataGen()
  {
   
  }

  void setAutomata(pr2_moveit_ltl::automaton &auto_)
  {
  
  	auto_.name = "Example";
  	auto_.id = 1;
  	
  	pr2_moveit_ltl::area_propositionPtr area_0(new pr2_moveit_ltl::area_proposition);
  	area_0->name_a = "P0";
  	area_0->alias_a = "The rest";
		area_0->id = 0;
		area_0->name_mg = "base";
		area_0->DOF_mg = 3;
	
		pr2_moveit_ltl::limitPtr lim_0_1(new pr2_moveit_ltl::limit);
		lim_0_1->min = -5.0;
		lim_0_1->max = 5.0;
	
		pr2_moveit_ltl::limitPtr lim_0_2(new pr2_moveit_ltl::limit);
		lim_0_2->min = -5.0;
		lim_0_2->max = 5.0;
	
		pr2_moveit_ltl::limitPtr lim_0_3(new pr2_moveit_ltl::limit);
		lim_0_3->min = -3.14;
		lim_0_3->max = 3.14;
	
		area_0->limits.push_back(*lim_0_1);
		area_0->limits.push_back(*lim_0_2);
		area_0->limits.push_back(*lim_0_3);
		
		pr2_moveit_ltl::area_propositionPtr area_1(new pr2_moveit_ltl::area_proposition);
		area_1->name_a = "P1";
		area_1->alias_a = "Kitchen";
		area_1->id = 1;
		area_1->name_mg = "base";
		area_1->DOF_mg = 3;
	
		pr2_moveit_ltl::limitPtr lim_1_1(new pr2_moveit_ltl::limit);
		lim_1_1->min = 0.9;
		lim_1_1->max = 1.1;
	
		pr2_moveit_ltl::limitPtr lim_1_2(new pr2_moveit_ltl::limit);
		lim_1_2->min = 3.8;
		lim_1_2->max = 4.2;
	
		pr2_moveit_ltl::limitPtr lim_1_3(new pr2_moveit_ltl::limit);
		lim_1_3->min = -3.14;
		lim_1_3->max = 3.14;
	
		area_1->limits.push_back(*lim_1_1);
		area_1->limits.push_back(*lim_1_2);
		area_1->limits.push_back(*lim_1_3);
	
		pr2_moveit_ltl::area_propositionPtr area_2(new pr2_moveit_ltl::area_proposition);
		area_2->name_a = "P2";
		area_2->alias_a = "Living";
		area_2->id = 2;
		area_2->name_mg = "base";
		area_2->DOF_mg = 3;
		
		pr2_moveit_ltl::limitPtr lim_2_1(new pr2_moveit_ltl::limit);
		lim_2_1->min = -4.1;
		lim_2_1->max = -3.8;
	
		pr2_moveit_ltl::limitPtr lim_2_2(new pr2_moveit_ltl::limit);
		lim_2_2->min = -4.1;
		lim_2_2->max = -3.8;
	
		pr2_moveit_ltl::limitPtr lim_2_3(new pr2_moveit_ltl::limit);
		lim_2_3->min = -3.14;
		lim_2_3->max = 3.14;
		
		area_2->limits.push_back(*lim_2_1);
		area_2->limits.push_back(*lim_2_2);
		area_2->limits.push_back(*lim_2_3);
		
		auto_.alphabet.push_back(*area_0);
		auto_.alphabet.push_back(*area_1);
		auto_.alphabet.push_back(*area_2);
		
		pr2_moveit_ltl::statePtr sta0(new pr2_moveit_ltl::state);
		sta0->name = "Edo0";
		sta0->id= 0;
		sta0->is_initial_state = true;
		sta0->is_acceptance_state = false;
		sta0->valid_propositions.push_back(0);
		sta0->valid_propositions.push_back(1);
		sta0->valid_propositions.push_back(2);
		
		pr2_moveit_ltl::transitionPtr trans0_0(new pr2_moveit_ltl::transition);
		trans0_0->name = "Trans edo 0 0";
		trans0_0->id = 0;
		trans0_0->initial_state = 0;
		trans0_0->final_state = 1;
		trans0_0->trans_.push_back(1);
	
		pr2_moveit_ltl::transitionPtr trans0_1(new pr2_moveit_ltl::transition);
		trans0_1->name = "Trans edo 0 1";
		trans0_1->id = 1;
		trans0_1->initial_state = 0;
		trans0_1->final_state = 0;
		trans0_1->trans_.push_back(2);
		trans0_1->trans_.push_back(0);
	
		sta0->transitions.push_back(*trans0_0);
		sta0->transitions.push_back(*trans0_1);
	
		pr2_moveit_ltl::statePtr sta1(new pr2_moveit_ltl::state);
		sta1->name = "Edo1";
		sta1->id= 1;
		sta1->is_initial_state = false;
		sta1->is_acceptance_state = false;
		sta1->valid_propositions.push_back(0);
		sta1->valid_propositions.push_back(1);
		
		pr2_moveit_ltl::transitionPtr trans1_0(new pr2_moveit_ltl::transition);
		trans1_0->name = "Trans edo 1 0";
		trans1_0->id = 10;
		trans1_0->initial_state = 1;
		trans1_0->final_state = 2;
		trans1_0->trans_.push_back(2);
		
		pr2_moveit_ltl::transitionPtr trans1_1(new pr2_moveit_ltl::transition);
		trans1_1->name = "Trans edo 1 1";
		trans1_1->id = 11;
		trans1_1->initial_state = 1;
		trans1_1->final_state = 1;
		trans1_1->trans_.push_back(1);
		trans1_1->trans_.push_back(0);
		
		sta1->transitions.push_back(*trans1_0);
		sta1->transitions.push_back(*trans1_1);
		
		pr2_moveit_ltl::statePtr sta2(new pr2_moveit_ltl::state);
		sta2->name = "Edo2";
		sta2->id= 2;
		sta2->is_initial_state = false;
		sta2->is_acceptance_state = true;
		sta2->valid_propositions.push_back(2);
	
		auto_.states.push_back(*sta0);
		auto_.states.push_back(*sta1);
		auto_.states.push_back(*sta2);
		
		auto_.initial_state = 0;
		auto_.acceptance_states.push_back(2);
  }
  
};

bool generate_automaton(pr2_moveit_ltl::automaton_gen_msg::Request  &req,
         pr2_moveit_ltl::automaton_gen_msg::Response &res)
{
	std::cout << "Generating for:" << req.client << std::endl;
  AutomataGen autom;
  pr2_moveit_ltl::automatonPtr au(new pr2_moveit_ltl::automaton);
  autom.setAutomata(*au);
	
	res.automata = *au;
  return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "automaton_gen_ltl");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("automaton_gen_ltl", generate_automaton);
  ROS_INFO("Ready to generate plan (automaton generator).");
  ros::spin();

  return 0;
}

