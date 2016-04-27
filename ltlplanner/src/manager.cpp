#include <ros/ros.h>

#include "ltlplanner/automaton.h"
#include "ltlplanner/region.h"
#include "ltlplanner/goal.h"
#include "ltlplanner/limit.h"
#include "ltlplanner/state.h"
#include "ltlplanner/transition.h"
#include "ltlplanner/plan.h"
#include "ltlplanner/task.h"
#include "ltlplanner/map_state.h"

#include <cstdlib>
#include <vector>

#include "ltlplanner/automaton_msg.h"
#include "ltlplanner/region_msg.h"
#include "ltlplanner/goal_msg.h"
#include "ltlplanner/plan_msg.h"

class Manager{
	private:
		std::vector<ltlplanner::automaton> automatas;
		std::vector<ltlplanner::region> regions;  
		ltlplanner::goal actGoal;
		ltlplanner::plan plan_;
		ros::NodeHandle nh_;
  	ros::Publisher pub_plan;
		
	public:
  
  	Manager(ros::NodeHandle &nh){
    	nh_ = nh;
    	pub_plan = nh_.advertise<ltlplanner::plan_msg>("public/plan", 100);
    	actGoal.id = -1;
  	}

  	~Manager(){
   
  	}
  	
  	bool addRegion(ltlplanner::region &region_){
  		bool agr = isinRegions(region_);
  		bool res = false;
  		if(!agr){
  			regions.push_back(region_);
  			res = true;
  		}
  		return res;
  	}

		bool addAutomata(ltlplanner::automaton &automata_){
			bool agr = isinAutomatas(automata_);
  		bool res = false;
  		if(!agr){
  			automatas.push_back(automata_);
  			res = true;
  		}
  		return res;
		}
		
		bool isinRegions(ltlplanner::region &region_){
			bool res = false;
			for(std::vector<ltlplanner::region>::const_iterator it = regions.begin(); it != regions.end(); ++it){
				ltlplanner::region aux = *it;
				if(aux.id == region_.id){
					res = true;
				}
			}
			return res;
		}
		
		bool isinRegions(int region_id){
			bool res = false;
			for(std::vector<ltlplanner::region>::const_iterator it = regions.begin(); it != regions.end(); ++it){
				ltlplanner::region aux = *it;
				if(aux.id == region_id){
					res = true;
				}
			}
			return res;
		}
		
		bool isinAutomatas(ltlplanner::automaton &automata_){
			bool res = false;
			for(std::vector<ltlplanner::automaton>::const_iterator it = automatas.begin(); it != automatas.end(); ++it){
				ltlplanner::automaton aux = *it;
				if(aux.id == automata_.id){
					res = true;
				}
			}
			return res;
		}
		
		bool isinAutomatas(int automata_id){
			bool res = false;
			for(std::vector<ltlplanner::automaton>::const_iterator it = automatas.begin(); it != automatas.end(); ++it){
				ltlplanner::automaton aux = *it;
				if(aux.id == automata_id){
					res = true;
				}
			}
			return res;
		}
		
		void setGoal(ltlplanner::goal &goal_){
			actGoal.id = goal_.id;
			actGoal.goal = goal_.goal;
		}
		
		void getGoal(ltlplanner::goal &goal_){
			goal_ = actGoal;
		}
		
		void listeningRegions(const ltlplanner::region_msgPtr& msg){
  		bool res = addRegion(msg->region);
  		if(res){
  			std::cout << "Se agrego: " << msg->region.id << std::endl;
  		}else{
  			std::cout << "No se agrego: " << msg->region.id << std::endl;
  		}
		}

		void listeningAutomatas(const ltlplanner::automaton_msgPtr& msg){
  		bool res = addAutomata(msg->automata);
  		if(res){
  			std::cout << "Se agrego: " << msg->automata.id << std::endl;
  		}else{
  			std::cout << "No se agrego: " << msg->automata.id << std::endl;
  		}
		}
		
		void listeningGoals(const ltlplanner::goal_msgPtr& msg){
			if(actGoal.id != msg->goal.id){
				setGoal(msg->goal);
  			std::cout << "Se agrego: " << msg->goal.id << " " << msg->goal.goal << std::endl;	
  			setPlan();	
			}
		}
		
		bool getAutomata(int automata_id, ltlplanner::automaton &auto_){
			bool res = false;
			int place = -1;
			int i = 0;
			for(std::vector<ltlplanner::automaton>::const_iterator it = automatas.begin(); it != automatas.end(); ++it){
				ltlplanner::automaton aux = *it;
				if(aux.id == automata_id){
					res = true;
					place = i;
					auto_ = automatas.at(place);
				}
				i++;
			}
			return res;
		}
		
		bool getState(int state_id, ltlplanner::statePtr &state_, std::vector<ltlplanner::statePtr> 
		&states){
			bool res = false;
			int place = -1;
			int i = 0;
			for(std::vector<ltlplanner::statePtr>::const_iterator it = states.begin(); it != states.end(); ++it){
				ltlplanner::statePtr aux = *it;
				if(aux->id == state_id){
					res = true;
					place = i;
					state_ = states.at(place);
				}
				i++;
			}
			return res;
		}
		
		bool removeState(std::vector<ltlplanner::statePtr> &states, int state_id){
			bool res = false;
			int place = -1;
			int i = 0;
			for(std::vector<ltlplanner::statePtr>::const_iterator it = states.begin(); it != states.end(); ++it){
				ltlplanner::statePtr aux = *it;
				if(aux->id == state_id){
					res = true;
					place = i;
					states.erase(states.begin() + place);
				}
				i++;
			}
			return res;
		}
		
		bool lookforpath(ltlplanner::automaton &auto_, ltlplanner::planPtr &theplan){
			bool res = false;
			
			//Get graph
			std::vector<ltlplanner::statePtr> states;
			std::vector<ltlplanner::transitionPtr> transitions;
			int i = 1;
			for(std::vector<ltlplanner::state>::const_iterator it = auto_.states.begin(); it != auto_.states.end(); ++it){
				ltlplanner::statePtr aux(new ltlplanner::state);
				ltlplanner::state aux_x = *it;
				aux->id = aux_x.id;
				aux->value = 99999;
				states.push_back(aux);
				for(std::vector<ltlplanner::transition>::const_iterator it_t = aux_x.transitions.begin(); it_t != aux_x.transitions.end(); ++it_t){
					ltlplanner::transitionPtr trans_(new ltlplanner::transition);
					ltlplanner::transition aux_t = *it_t;
					trans_->id = i;
					trans_->value = aux_t.value;
					trans_->initial_state = aux_t.initial_state;
					trans_->final_state = aux_t.final_state;
					trans_->group_name = aux_t.group_name;
					trans_->name = aux_t.name;
					trans_->dtimes = aux_t.dtimes;
					trans_->routine = aux_t.routine;
					trans_->trans_alphabet = aux_t.trans_alphabet;
					transitions.push_back(trans_);
					i++;
				}
			}
			
			//Auxiliary initialization
			std::vector<ltlplanner::statePtr> settledNodes;
			std::vector<ltlplanner::statePtr> unSettledNodes;
			std::vector<ltlplanner::map_statePtr> predecessors;
			std::vector<ltlplanner::map_statePtr> distance;
			
			//Initial and final node
			ltlplanner::statePtr initial(new ltlplanner::state);
			bool boolinistate = getState(auto_.initial_state, initial, states);
			ltlplanner::statePtr end(new ltlplanner::state);
			bool boolendstate = getState(auto_.acceptance_states.at(0), end, states);
			
			//Dikjstra
			ltlplanner::map_statePtr dist_aux_ini(new ltlplanner::map_state);
			dist_aux_ini->id_a = initial->id;
			dist_aux_ini->dist = 0.0;
			distance.push_back(dist_aux_ini);
			unSettledNodes.push_back(initial);
			
			while(unSettledNodes.size() > 0){
				ltlplanner::statePtr min(new ltlplanner::state);
				bool minfind = getMinimum(unSettledNodes, min, distance);
				settledNodes.push_back(min);
				bool removed = removeState(unSettledNodes, min->id);
				findMinimalDistances(min, unSettledNodes, predecessors, distance, settledNodes, states, transitions);
			}
			std::vector<ltlplanner::statePtr> path;
			bool pathmade = getPath(end, path, predecessors, states);
			
			if(pathmade){
				bool planmsg = generatePlanMsg(path, theplan, transitions);
				if(planmsg){
					res = true;
				}
			}
			return res;
		}
		
		bool generatePlanMsg(std::vector<ltlplanner::statePtr> &path, ltlplanner::planPtr &plan, std::vector<ltlplanner::transitionPtr> &transitions){
			bool res = false;
			for(std::vector<ltlplanner::statePtr>::const_iterator it = path.begin(); it != (path.end() - 1); ++it){
				ltlplanner::statePtr aux = *it;
				ltlplanner::statePtr aux_next = *(it + 1);
				
				ltlplanner::transitionPtr trans(new ltlplanner::transition);
				bool trans_bool = getTransition(aux, aux_next, trans, transitions);	
				int k = 0;
				
				if(trans_bool){
					ltlplanner::taskPtr prop(new ltlplanner::task);
    			prop->id = k;
  				prop->routine= trans->routine;
  				prop->name_mg = trans->group_name;
  				prop->trans_alphabet = trans->trans_alphabet;
  				prop->dtimes = trans->dtimes;
  				plan->tasks.push_back(*prop);
  				k++;
				}
			}
			return res;
		}
		
		bool getTransition(ltlplanner::statePtr &ini, ltlplanner::statePtr &fin, ltlplanner::transitionPtr &trans, std::vector<ltlplanner::transitionPtr> &transitions){
			bool res = false;
			int i = 0;
			int place = -1;
			for(std::vector<ltlplanner::transitionPtr>::const_iterator it_t = transitions.begin(); it_t != transitions.end(); ++it_t){
				ltlplanner::transitionPtr aux = *it_t;
				if(aux->initial_state == ini->id && aux->final_state == fin->id){
					place = i;
					trans = aux;
					res = true;
				}
				i++;
			}
			return res;	
		}
		
		void findMinimalDistances(ltlplanner::statePtr &node, std::vector<ltlplanner::statePtr> &unSettledNodes, 
			std::vector<ltlplanner::map_statePtr> &predecessors, std::vector<ltlplanner::map_statePtr> &distance, std::vector<ltlplanner::statePtr> &settledNodes, std::vector<ltlplanner::statePtr> &states, std::vector<ltlplanner::transitionPtr> &transitions){
			std::vector<ltlplanner::statePtr> adjacentNodes;
			getNeighbors(node, adjacentNodes, transitions, states, settledNodes);
			for(std::vector<ltlplanner::statePtr>::const_iterator it = adjacentNodes.begin(); it != adjacentNodes.end(); ++it){
				ltlplanner::statePtr aux = *it;
				if(getShortestDistance(aux, distance) > (getShortestDistance(node, distance) + getDistance(node, aux, transitions))){
					ltlplanner::map_statePtr dist_aux(new ltlplanner::map_state);
					dist_aux->id_a = aux->id;
					dist_aux->dist = getShortestDistance(node, distance) + getDistance(node, aux, transitions);
					distance.push_back(dist_aux);
					
					ltlplanner::map_statePtr ante_aux(new ltlplanner::map_state);
					ante_aux->id_a = aux->id;
					ante_aux->id_b = node->id;
					predecessors.push_back(ante_aux);
					unSettledNodes.push_back(aux);
				}
			}
		}
		
		float getDistance(ltlplanner::statePtr &node, ltlplanner::statePtr &target, std::vector<ltlplanner::transitionPtr> &transitions){
			int dist = -1;
			for(std::vector<ltlplanner::transitionPtr>::const_iterator it_t = transitions.begin(); it_t != transitions.end(); ++it_t){
				ltlplanner::transitionPtr aux = *it_t;
				if(aux->initial_state == node->id && aux->final_state == target->id){
					dist = aux->value;
				}
			}
			return dist;
		}
		
		bool getNeighbors(ltlplanner::statePtr &node , std::vector<ltlplanner::statePtr> &adjacentNodes, std::vector<ltlplanner::transitionPtr> &transitions, std::vector<ltlplanner::statePtr> &states, std::vector<ltlplanner::statePtr> &settledNodes){
			bool res = false;
			for(std::vector<ltlplanner::transitionPtr>::const_iterator it_t = transitions.begin(); it_t != transitions.end(); ++it_t){
				ltlplanner::transitionPtr aux = *it_t;
				if(aux->initial_state == node->id && !isSettled(aux->final_state, settledNodes)){
					ltlplanner::statePtr target(new ltlplanner::state);
					bool boolendstate = getState(aux->final_state, target, states);
					if(boolendstate){
						adjacentNodes.push_back(target);
					}
					res = boolendstate;
				}
			}
			return res;
		}
		
		bool getMinimum(std::vector<ltlplanner::statePtr> &vertexes, ltlplanner::statePtr &minimum, std::vector<ltlplanner::map_statePtr> &distance){
			bool res = false;
			ltlplanner::statePtr min = vertexes.at(0);
			for(std::vector<ltlplanner::statePtr>::const_iterator it_t = vertexes.begin(); it_t != vertexes.end(); ++it_t){
				ltlplanner::statePtr aux = *it_t;
				if(getShortestDistance(aux, distance) < getShortestDistance(min, distance)){
					min = aux;
				}
			}
			minimum = min;
			return res;
		}
		
		bool isSettled(int node_id, std::vector<ltlplanner::statePtr> &settledNodes){
			bool res = false;
			for(std::vector<ltlplanner::statePtr>::const_iterator it = settledNodes.begin(); it != settledNodes.end(); ++it){
				ltlplanner::statePtr aux = *it;
				if(aux->id == node_id){
					res = true;
				}
			}
			return res;
		}
		
		int getShortestDistance(ltlplanner::statePtr &destination, std::vector<ltlplanner::map_statePtr> &distance){
			int d = 99999;
			for(std::vector<ltlplanner::map_statePtr>::const_iterator it = distance.begin(); it != distance.end(); ++it){
				ltlplanner::map_statePtr aux = *it;
				if(aux->id_a == destination->id){
					d = aux->dist;
				}
			}
			return d;
		}
		
		bool getPath(ltlplanner::statePtr &target, std::vector<ltlplanner::statePtr> &path, std::vector<ltlplanner::map_statePtr> &predecessors, std::vector<ltlplanner::statePtr> &states){
			std::vector<ltlplanner::statePtr> pathaux;
			ltlplanner::statePtr step(new ltlplanner::state);
			ltlplanner::statePtr aux(new ltlplanner::state);
			step = target;
			if(!predecessorsGet(step->id, aux, predecessors, states)){
				return false;
			}
			pathaux.push_back(step);
			while(predecessorsGet(step->id, aux, predecessors, states)){
				predecessorsGet(step->id, aux, predecessors, states);
				step = aux;
				pathaux.push_back(step);
			}
			reverse(pathaux, path);
			return true;
		}
		
		bool predecessorsGet(int id, ltlplanner::statePtr &node, std::vector<ltlplanner::map_statePtr> &predecessors, std::vector<ltlplanner::statePtr> &states){
			bool res = false;
			int place = -1;
			for(std::vector<ltlplanner::map_statePtr>::const_iterator it = predecessors.begin(); it != predecessors.end(); ++it){
				ltlplanner::map_statePtr aux = *it;
				if(aux->id_a == id){
					place = aux->id_b;
					res = true;
				}
			}
			if(res){
				getState(place, node, states);
			}
			return res;
		}
		
		void reverse(std::vector<ltlplanner::statePtr> &pathaux, std::vector<ltlplanner::statePtr> &path){
			for(std::vector<ltlplanner::statePtr>::const_iterator it = pathaux.begin(); it != pathaux.end(); ++it){
				ltlplanner::statePtr aux = *it;
				path.push_back(aux);
			}
		}
		
		bool setPlan(){
			bool res = false;
			if(actGoal.id != -1 && isinAutomatas(actGoal.id_automata)){
				ltlplanner::automatonPtr auto_(new ltlplanner::automaton);
				bool isAuto = getAutomata(actGoal.id_automata, *auto_);
				if(isAuto){
					ltlplanner::planPtr tasks(new ltlplanner::plan);
					bool isPlan = lookforpath(*auto_, tasks);
					if(isPlan){
						plan_ = *tasks;
						ltlplanner::plan_msgPtr plan(new ltlplanner::plan_msg);
						plan->id = tasks->id;
						plan->plan = *tasks;
						if(ros::ok()){
							pub_plan.publish(*plan);
							ros::spinOnce();
						}
					}
				}	
			}
		}
};

int main(int argc, char **argv){
	ros::init(argc, argv, "MANAGER");
	ros::NodeHandle n;
	Manager man(n);
	
	ros::Subscriber sub_reg = n.subscribe("public/region", 100, &Manager::listeningRegions, &man);
	ros::Subscriber sub_aut = n.subscribe("public/automata", 100, &Manager::listeningAutomatas, &man);
	ros::Subscriber sub_goa = n.subscribe("public/goal", 100, &Manager::listeningGoals, &man);
	
	ros::spin();

  return 0;
}
