#include "ros/ros.h"
#include <sstream>
#include <vector>
#include "ltlplanner/automaton.h"
#include "ltlplanner/automaton_msg.h"
#include "ltlplanner/region.h"
#include "ltlplanner/limit.h"
#include "ltlplanner/region_msg.h"
#include "ltlplanner/state.h"
#include "ltlplanner/transition.h"

class Automaton{
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_auto;
		std::vector<ltlplanner::automaton> automatas_;
		int index;
		
	public:
		Automaton(ros::NodeHandle &nh){
			nh_ = nh;
			pub_auto = nh_.advertise<ltlplanner::automaton_msg>("public/automata", 100);
			index = 1;
			setAutomata();
			sleep(10.0);
		}
		
		~Automaton(){
		
		}
		
		bool setAutomata(){
			ltlplanner::automaton auto_;
			
			auto_.name = "Example";
  		auto_.id = 0;
  	
			ltlplanner::state sta0;
			sta0.name = "Edo0";
			sta0.id= 0;
			sta0.is_initial_state = true;
			sta0.is_acceptance_state = false;
			sta0.valid_propositions.push_back(0);
			sta0.valid_propositions.push_back(1);
			sta0.valid_propositions.push_back(2);
		
			ltlplanner::transition trans0_0;
			trans0_0.name = "Trans edo 0 0";
			trans0_0.id = 0;
			trans0_0.initial_state = 0;
			trans0_0.final_state = 1;
			trans0_0.value = 2;
			trans0_0.group_name = "base";
			trans0_0.trans_alphabet.push_back(1);
	
			ltlplanner::transition trans0_1;
			trans0_1.name = "Trans edo 0 1";
			trans0_1.id = 1;
			trans0_1.initial_state = 0;
			trans0_1.final_state = 0;
			trans0_1.value = 2;
			trans0_1.group_name = "base";
			trans0_1.trans_alphabet.push_back(2);
			trans0_1.trans_alphabet.push_back(0);
	
			sta0.transitions.push_back(trans0_0);
			sta0.transitions.push_back(trans0_1);
	
			ltlplanner::state sta1;
			sta1.name = "Edo1";
			sta1.id= 1;
			sta1.is_initial_state = false;
			sta1.is_acceptance_state = false;
			sta1.valid_propositions.push_back(0);
			sta1.valid_propositions.push_back(1);
		
			ltlplanner::transition trans1_0;
			trans1_0.name = "Trans edo 1 0";
			trans1_0.id = 10;
			trans1_0.initial_state = 1;
			trans1_0.final_state = 2;
			trans1_0.value = 2;
			trans1_0.group_name = "base";
			trans1_0.trans_alphabet.push_back(2);
		
			ltlplanner::transition trans1_1;
			trans1_1.name = "Trans edo 1 1";
			trans1_1.id = 11;
			trans1_1.initial_state = 1;
			trans1_1.final_state = 1;
			trans1_1.value = 2;
			trans1_1.group_name = "base";
			trans1_1.trans_alphabet.push_back(1);
			trans1_1.trans_alphabet.push_back(0);
		
			sta1.transitions.push_back(trans1_0);
			sta1.transitions.push_back(trans1_1);
		
			ltlplanner::state sta2;
			sta2.name = "Edo2";
			sta2.id= 2;
			sta2.is_initial_state = false;
			sta2.is_acceptance_state = true;
			sta2.valid_propositions.push_back(2);
	
			auto_.states.push_back(sta0);
			auto_.states.push_back(sta1);
			auto_.states.push_back(sta2);
		
			auto_.initial_state = 0;
			auto_.acceptance_states.push_back(2);
		
			automatas_.push_back(auto_);
		}
		
		bool sendAutomata(){
			bool res = false;
			while(ros::ok()){
				sleep(10.0);
				for(std::vector<ltlplanner::automaton>::const_iterator it = automatas_.begin(); it != automatas_.end(); ++it){
					sleep(10.0);
					ltlplanner::automaton aux = *it;
					ltlplanner::automaton_msg auto_msg;
					auto_msg.id = index;
					auto_msg.automata = aux;
					pub_auto.publish(auto_msg);
					index++;	
				}
				res = true;
			}
			return res;
		}
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "AUTOMATA_PUBLISHER_1");
  ros::NodeHandle n;

	Automaton auto_(n);
	
	bool res = auto_.sendAutomata();
	
  return 0;
}
