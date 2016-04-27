#include <ros/ros.h>

#include "pr2_moveit_ltl/automaton_gen_msg.h"
#include "pr2_moveit_ltl/automaton.h"
#include "pr2_moveit_ltl/area_proposition.h"
#include "pr2_moveit_ltl/limit.h"
#include "pr2_moveit_ltl/state.h"
#include "pr2_moveit_ltl/transition.h"
#include <cstdlib>
#include <vector>

class Manager{
	private:
		std::vector<pr2_moveit_ltl::automaton> automatas;
		std::vector<pr2_moveit_ltl::areas> areas;  
	public:
  
  	Manager(){
    
  	}

  	~Manager(){
   
  	}

};

int main(int argc, char **argv){
	ros::init(argc, argv, "MANAGER");
	ros::NodeHandle n;

  ros::spin();

  return 0;
}
