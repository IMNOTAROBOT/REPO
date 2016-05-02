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
			trans0_0.value = 7;
			trans0_0.group_name = "base";
			trans0_0.trans_alphabet.push_back(1);
	
			ltlplanner::transition trans0_1;
			trans0_1.name = "Trans edo 0 1";
			trans0_1.id = 1;
			trans0_1.initial_state = 0;
			trans0_1.final_state = 2;
			trans0_1.value = 9;
			trans0_1.group_name = "base";
			trans0_1.trans_alphabet.push_back(2);
			trans0_1.trans_alphabet.push_back(0);
			
			ltlplanner::transition trans0_2;
			trans0_2.name = "Trans edo 0 3";
			trans0_2.id = 2;
			trans0_2.initial_state = 0;
			trans0_2.final_state = 5;
			trans0_2.value = 14;
			trans0_2.group_name = "base";
			trans0_2.trans_alphabet.push_back(2);
			trans0_2.trans_alphabet.push_back(0);
	
			sta0.transitions.push_back(trans0_0);
			sta0.transitions.push_back(trans0_1);
			sta0.transitions.push_back(trans0_2);
	
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
			trans1_0.final_state = 0;
			trans1_0.value = 7;
			trans1_0.group_name = "base";
			trans1_0.trans_alphabet.push_back(2);
		
			ltlplanner::transition trans1_1;
			trans1_1.name = "Trans edo 1 1";
			trans1_1.id = 11;
			trans1_1.initial_state = 1;
			trans1_1.final_state = 2;
			trans1_1.value = 10;
			trans1_1.group_name = "base";
			trans1_1.trans_alphabet.push_back(1);
			trans1_1.trans_alphabet.push_back(0);
		
			ltlplanner::transition trans1_2;
			trans1_2.name = "Trans edo 1 2";
			trans1_2.id = 11;
			trans1_2.initial_state = 1;
			trans1_2.final_state = 3;
			trans1_2.value = 15;
			trans1_2.group_name = "base";
			trans1_2.trans_alphabet.push_back(1);
			trans1_2.trans_alphabet.push_back(0);
			
			sta1.transitions.push_back(trans1_0);
			sta1.transitions.push_back(trans1_1);
			sta1.transitions.push_back(trans1_2);
		
			ltlplanner::state sta2;
			sta2.name = "Edo2";
			sta2.id= 2;
			sta2.is_initial_state = false;
			sta2.is_acceptance_state = false;
			sta2.valid_propositions.push_back(2);
			
			ltlplanner::transition trans2_0;
			trans2_0.name = "Trans edo 2 0";
			trans2_0.id = 20;
			trans2_0.initial_state = 2;
			trans2_0.final_state = 0;
			trans2_0.value = 9;
			trans2_0.group_name = "base";
			trans2_0.trans_alphabet.push_back(2);
			
			ltlplanner::transition trans2_1;
			trans2_1.name = "Trans edo 2 1";
			trans2_1.id = 21;
			trans2_1.initial_state = 2;
			trans2_1.final_state = 1;
			trans2_1.value = 10;
			trans2_1.group_name = "base";
			trans2_1.trans_alphabet.push_back(2);
			
			ltlplanner::transition trans2_2;
			trans2_2.name = "Trans edo 2 1";
			trans2_2.id = 22;
			trans2_2.initial_state = 2;
			trans2_2.final_state = 3;
			trans2_2.value = 11;
			trans2_2.group_name = "base";
			trans2_2.trans_alphabet.push_back(2);
			
			ltlplanner::transition trans2_3;
			trans2_3.name = "Trans edo 2 3";
			trans2_3.id = 23;
			trans2_3.initial_state = 2;
			trans2_3.final_state = 5;
			trans2_3.value = 2;
			trans2_3.group_name = "base";
			trans2_3.trans_alphabet.push_back(2);
			
			sta2.transitions.push_back(trans2_0);
			sta2.transitions.push_back(trans2_1);
			sta2.transitions.push_back(trans2_2);
			sta2.transitions.push_back(trans2_3);
			
			ltlplanner::state sta3;
			sta3.name = "Edo3";
			sta3.id= 3;
			sta3.is_initial_state = false;
			sta3.is_acceptance_state = false;
			sta3.valid_propositions.push_back(2);
			
			ltlplanner::transition trans3_0;
			trans3_0.name = "Trans edo 3 0";
			trans3_0.id = 30;
			trans3_0.initial_state = 3;
			trans3_0.final_state = 1;
			trans3_0.value = 15;
			trans3_0.group_name = "base";
			trans3_0.trans_alphabet.push_back(2);
			
			ltlplanner::transition trans3_1;
			trans3_1.name = "Trans edo 3 1";
			trans3_1.id = 31;
			trans3_1.initial_state = 3;
			trans3_1.final_state = 2;
			trans3_1.value = 11;
			trans3_1.group_name = "base";
			trans3_1.trans_alphabet.push_back(2);
			
			ltlplanner::transition trans3_2;
			trans3_2.name = "Trans edo 3 2";
			trans3_2.id = 22;
			trans3_2.initial_state = 3;
			trans3_2.final_state = 4;
			trans3_2.value = 6;
			trans3_2.group_name = "base";
			trans3_2.trans_alphabet.push_back(2);
			
			sta3.transitions.push_back(trans3_0);
			sta3.transitions.push_back(trans3_1);
			sta3.transitions.push_back(trans3_2);
			
			ltlplanner::state sta4;
			sta4.name = "Edo4";
			sta4.id= 4;
			sta4.is_initial_state = false;
			sta4.is_acceptance_state = false;
			sta4.valid_propositions.push_back(2);
			
			ltlplanner::transition trans4_0;
			trans4_0.name = "Trans edo 4 0";
			trans4_0.id = 40;
			trans4_0.initial_state = 4;
			trans4_0.final_state = 3;
			trans4_0.value = 6;
			trans4_0.group_name = "base";
			trans4_0.trans_alphabet.push_back(2);
			
			ltlplanner::transition trans4_1;
			trans4_1.name = "Trans edo 4 1";
			trans4_1.id = 41;
			trans4_1.initial_state = 4;
			trans4_1.final_state = 5;
			trans4_1.value = 9;
			trans4_1.group_name = "base";
			trans4_1.trans_alphabet.push_back(2);
			
			sta4.transitions.push_back(trans4_0);
			sta4.transitions.push_back(trans4_1);
			
			ltlplanner::state sta5;
			sta5.name = "Edo5";
			sta5.id= 5;
			sta5.is_initial_state = false;
			sta5.is_acceptance_state = true;
			sta5.valid_propositions.push_back(2);
			
			ltlplanner::transition trans5_0;
			trans5_0.name = "Trans edo 5 0";
			trans5_0.id = 50;
			trans5_0.initial_state = 5;
			trans5_0.final_state = 0;
			trans5_0.value = 14;
			trans5_0.group_name = "base";
			trans5_0.trans_alphabet.push_back(2);
			
			ltlplanner::transition trans5_1;
			trans5_1.name = "Trans edo 5 1";
			trans5_1.id = 51;
			trans5_1.initial_state = 5;
			trans5_1.final_state = 2;
			trans5_1.value = 2;
			trans5_1.group_name = "base";
			trans5_1.trans_alphabet.push_back(2);
			
			ltlplanner::transition trans5_2;
			trans5_2.name = "Trans edo 5 2";
			trans5_2.id = 52;
			trans5_2.initial_state = 5;
			trans5_2.final_state = 4;
			trans5_2.value = 9;
			trans5_2.group_name = "base";
			trans5_2.trans_alphabet.push_back(2);
			
			sta5.transitions.push_back(trans5_0);
			sta5.transitions.push_back(trans5_1);
			sta5.transitions.push_back(trans5_2);
			
			auto_.states.push_back(sta0);
			auto_.states.push_back(sta1);
			auto_.states.push_back(sta2);
			auto_.states.push_back(sta3);
			auto_.states.push_back(sta4);
			auto_.states.push_back(sta5);
		
			auto_.initial_state = 0;
			auto_.acceptance_states.push_back(5);
		
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
