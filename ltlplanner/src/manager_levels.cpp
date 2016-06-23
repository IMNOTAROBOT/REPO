#include <ros/ros.h>

#include "ltlplanner/automaton.h"
#include "ltlplanner/region.h"
#include "ltlplanner/goal.h"
#include "ltlplanner/limit.h"
#include "ltlplanner/state.h"
#include "ltlplanner/transition.h"
#include "ltlplanner/plan.h"
#include "ltlplanner/task.h"
#include "ltlplanner/edge.h"
#include "ltlplanner/node.h"
#include "ltlplanner/map_state.h"

#include <cstdlib>
#include <vector>
#include <sstream>
#include <stdlib.h>

#include "ltlplanner/automaton_msg.h"
#include "ltlplanner/region_msg.h"
#include "ltlplanner/goal_msg.h"
#include "ltlplanner/plan_msg.h"

//Planear
#include <ltlplanner/planningRRTAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<ltlplanner::planningRRTAction> HighPlanClient;

class Manager{
	private:
		ros::NodeHandle client;
		std::vector<ltlplanner::automaton> automatas;
		std::vector<ltlplanner::region> regions;  
		ltlplanner::goal actGoal;
		ltlplanner::plan plan_;
		ros::NodeHandle nh_;
  	ros::Publisher pub_plan;
		int index;
		bool planning;
		
		HighPlanClient* ac_;
	public:
  
  	Manager(ros::NodeHandle &nh){
    	nh_ = nh;
    	pub_plan = nh_.advertise<ltlplanner::plan_msg>("/public/plan", 1);
    	ac_ = new HighPlanClient(client,"twolevelplanning_srv", true);
    	actGoal.id = -1;
    	index = 0;
    	planning = false;
    	while(!ac_->waitForServer(ros::Duration(5.0))){
    		ROS_INFO("HIGHPLANNER : Waiting for lowplanner action server to come up");
  		}
  	}

  	~Manager(){
   		if(ac_ != NULL){
      	delete ac_;
    	}
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
		
		ltlplanner::automaton getAutomata(int automata_id){
			ltlplanner::automaton res;
			int place = -1;
			int i = 0;
			for(std::vector<ltlplanner::automaton>::const_iterator it = automatas.begin(); it != automatas.end(); ++it){
				ltlplanner::automaton aux = *it;
				if(aux.id == automata_id){
					place = i;
					res = automatas.at(place);
				}
				i++;
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
			actGoal.id_automata = goal_.id_automata;
		}
		
		ltlplanner::goal getGoal(){
			return actGoal;
		}
		
		void listeningRegions(const ltlplanner::region_msgPtr& msg){
  		bool res = addRegion(msg->region);
  		if(res){
  			std::cout << "Se agrego region: " << msg->region.id << std::endl;
  		}
		}

		void listeningAutomatas(const ltlplanner::automaton_msgPtr& msg){
  		bool res = addAutomata(msg->automata);
  		if(res){
  			std::cout << "Se agrego automata: " << msg->automata.id << std::endl;
  		}
		}
		
		void listeningGoals(const ltlplanner::goal_msgPtr& msg){
			if(actGoal.id != msg->goal.id){
				setGoal(msg->goal);
  			std::cout << "Se agrego objetivo: " << msg->goal.id << " " << msg->goal.goal << std::endl;	
  			planning = false;	
			}
			if(!planning){
				setPlan();
				executePlan();
			}else{
				publishingPlan();
			}
		}
		
		//Metodo que publica el plan actual
		void publishingPlan(){
			ltlplanner::plan_msg plan_msg;
			plan_msg.id = index;
			plan_msg.plan = plan_;
			std::cout << "Se publico plan: " << plan_.id << std::endl;
			pub_plan.publish(plan_msg);
			index++;	
		}
		
		bool executePlan(){
			bool res = false;
			for(std::vector<ltlplanner::task>::const_iterator it = plan_.tasks.begin(); it != plan_.tasks.end(); ++it){
				ltlplanner::task aux = *it;
				
				ltlplanner::planningRRTGoal goal;
 	 			
 	 			goal.id = aux.id;
 	 			goal.group_name = aux.name_mg;
 	 			goal.region_id = aux.trans_alphabet;
 	 		
	  		ac_->sendGoal(goal);
				
				ac_->waitForResult();
  			
  			if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    			std::cout << "MANAGER: Se complio: " << aux.id << std::endl;
    			res = true;
    		}else{
    			ROS_INFO("MANAGER : Mal");
    			res = false;
  			}
				
			}
			return res;
		
		}
		//I. Metodo raiz para crear plan segun un objetivo
		void setPlan(){
			if(actGoal.id != -1 && isinAutomatas(actGoal.id_automata)){
				std::cout << "Planear" << std::endl;
				ltlplanner::automaton auto_ = getAutomata(actGoal.id_automata);
				std::vector<ltlplanner::taskPtr> tasks;
				bool res = lookforpath(auto_, tasks);
				if(res){
					plan_.tasks.clear();
					for(std::vector<ltlplanner::taskPtr>::const_iterator it_t = tasks.begin(); it_t != tasks.end(); ++it_t){
						ltlplanner::taskPtr aux = *it_t;
						plan_.tasks.push_back(*aux);
					}
					
					planning = true;
				}
			}
		}
		
		//II. Metodo Dikjstra para generar una trayectoria corta y valida dentro del automata.
		bool lookforpath(ltlplanner::automaton &auto_, std::vector<ltlplanner::taskPtr> &path_res){
			bool res = false;
			
			//Get graph
			std::vector<ltlplanner::nodePtr> states;
			std::vector<ltlplanner::edgePtr> transitions;
			
			for(std::vector<ltlplanner::state>::const_iterator it = auto_.states.begin(); it != auto_.states.end(); ++it){
				ltlplanner::state aux= *it;
				ltlplanner::nodePtr node(new ltlplanner::node);
				node->id = aux.id;
				node->value = 99999;
				states.push_back(node);
				
				for(std::vector<ltlplanner::transition>::const_iterator it_t = aux.transitions.begin(); it_t != aux.transitions.end(); ++it_t){
					ltlplanner::transition aux_t = *it_t;
					ltlplanner::edgePtr trans_(new ltlplanner::edge);
					trans_->id = aux_t.id;
					trans_->value = aux_t.value;
					trans_->id_a = aux_t.initial_state;
					trans_->id_b = aux_t.final_state;
  				trans_->routine = aux_t.routine;
  				trans_->group_name = aux_t.group_name;
  				trans_->trans_alphabet = aux_t.trans_alphabet;
  				trans_->dtimes = aux_t.dtimes;
					transitions.push_back(trans_);
				}
			}
			
			//Auxiliary initialization
			std::vector<ltlplanner::nodePtr> settledNodes;
			std::vector<ltlplanner::nodePtr> unSettledNodes;
			std::vector<ltlplanner::map_statePtr> predecessors;
			std::vector<ltlplanner::map_statePtr> distance;
			
			//Initial and final node
			ltlplanner::node initial = getNode(auto_.initial_state, states);
			ltlplanner::node end = getNode(auto_.acceptance_states.at(0), states);
			
			//Dikjstra
			ltlplanner::nodePtr initialP(new ltlplanner::node);
			initialP->id = initial.id;
			initialP->value = initial.value;
			ltlplanner::nodePtr endP(new ltlplanner::node);
			endP->id = end.id;
			endP->value = end.value;
			ltlplanner::map_statePtr dist_aux_ini(new ltlplanner::map_state);
			dist_aux_ini->id_a = initial.id;
			dist_aux_ini->dist = 0.0;
			distance.push_back(dist_aux_ini);
			unSettledNodes.push_back(initialP);
			
			while(unSettledNodes.size() > 0){
				
				ltlplanner::nodePtr min(new ltlplanner::node);
				bool minfind = getMinimum(unSettledNodes, min, distance);
				settledNodes.push_back(min);
				bool removed = removeNode(unSettledNodes, min->id);
				findMinimalDistances(min, unSettledNodes, predecessors, distance, settledNodes, states, transitions);
			}
			std::vector<ltlplanner::nodePtr> path;
			bool pathmade = getPath(endP, path, predecessors, states);
			
			
			if(pathmade){
				generatePlanMsg(path, path_res, transitions);
				res = true;
			}
			return res;
		}
		
		//Funcion que encuentra la distancia minima de todos los vecinos de un nodo hacia el origen
		void findMinimalDistances(ltlplanner::nodePtr &node, std::vector<ltlplanner::nodePtr> &unSettledNodes, 
			std::vector<ltlplanner::map_statePtr> &predecessors, std::vector<ltlplanner::map_statePtr> &distance, std::vector<ltlplanner::nodePtr> &settledNodes, std::vector<ltlplanner::nodePtr> &states, std::vector<ltlplanner::edgePtr> &transitions){
			
			std::vector<ltlplanner::nodePtr> adjacentNodes;
			getNeighbors(node, adjacentNodes, transitions, states, settledNodes);
			
			for(std::vector<ltlplanner::nodePtr>::const_iterator it = adjacentNodes.begin(); it != adjacentNodes.end(); ++it){
				ltlplanner::nodePtr aux = *it;
				if(getShortestDistance(aux, distance) > (getShortestDistance(node, distance) + getDistance(node, aux, transitions))){
					ltlplanner::map_statePtr dist_aux(new ltlplanner::map_state);
					dist_aux->id_a = aux->id;
					dist_aux->dist = getShortestDistance(node, distance) + getDistance(node, aux, transitions);
					addDistance(dist_aux, distance);
					
					ltlplanner::map_statePtr ante_aux(new ltlplanner::map_state);
					ante_aux->id_a = aux->id;
					ante_aux->id_b = node->id;
					addPredecessors(ante_aux, predecessors);
					unSettledNodes.push_back(aux);
				}
			}
		}
		
		//Agrega nuevos valores de distancias a vector de distancias
		void addDistance(ltlplanner::map_statePtr &dist, std::vector<ltlplanner::map_statePtr> &distance){
			bool found = false;
			int i = 0;
			int place = 0;
			for(std::vector<ltlplanner::map_statePtr>::const_iterator it = distance.begin(); it != distance.end(); ++it){
				ltlplanner::map_statePtr aux = *it;
				if(aux->id_a == dist->id_a){
					found = true;
					place = i;
				}
				i++;
			}
			if(found){
				distance.at(place)->dist = dist->dist;
			}else{
				distance.push_back(dist);
			}
		
		}
		
		void addPredecessors(ltlplanner::map_statePtr &map, std::vector<ltlplanner::map_statePtr> &predecessors){
			bool found = false;
			int i = 0;
			int place = 0;
			for(std::vector<ltlplanner::map_statePtr>::const_iterator it = predecessors.begin(); it != predecessors.end(); ++it){
				ltlplanner::map_statePtr aux = *it;
				if(aux->id_a == map->id_a){
					found = true;
					place = i;
				}
				i++;
			}
			if(found){
				predecessors.at(place)->id_b = map->id_b;
			}else{
				predecessors.push_back(map);
			}
		
		}
		
		//Regresa vector con los nodos vecinos de un nodo.
		bool getNeighbors(ltlplanner::nodePtr &node , std::vector<ltlplanner::nodePtr> &adjacentNodes, std::vector<ltlplanner::edgePtr> &transitions, std::vector<ltlplanner::nodePtr> &states, std::vector<ltlplanner::nodePtr> &settledNodes){
			bool res = false;
			for(std::vector<ltlplanner::edgePtr>::const_iterator it_t = transitions.begin(); it_t != transitions.end(); ++it_t){
				ltlplanner::edgePtr aux = *it_t;
				if(aux->id_a == node->id && !isSettled(aux->id_b, settledNodes)){
					ltlplanner::nodePtr target(new ltlplanner::node);
					bool boolendstate = getNodePtr(aux->id_b, target, states);
					if(boolendstate){
						adjacentNodes.push_back(target);
					}
					res = boolendstate;
				}
			}
			return res;
		}
		
		//Regresa el valor de un nodo.
		ltlplanner::node getNode(int state_id, std::vector<ltlplanner::nodePtr> &states){
			ltlplanner::node state_;
			int place = -1;
			int i = 0;
			for(std::vector<ltlplanner::nodePtr>::const_iterator it = states.begin(); it != states.end(); ++it){
				ltlplanner::nodePtr aux = *it;
				if(aux->id == state_id){
					place = i;
					state_.id = aux->id;
					state_.value = aux->value;
				}
				i++;
			}
			return state_;
		}
		//Regresa un nodo.
		bool getNodePtr(int state_id, ltlplanner::nodePtr &state_, std::vector<ltlplanner::nodePtr> &states){
			bool res = false;
			int place = -1;
			int i = 0;
			for(std::vector<ltlplanner::nodePtr>::const_iterator it = states.begin(); it != states.end(); ++it){
				ltlplanner::nodePtr aux = *it;
				if(aux->id == state_id){
					state_ = aux;
					res = true;
				}
				i++;
			}
			return res;
		}
		
		//Regresa el nodo dentro del vector vertexes con menor distancia se;alada en el vector distancia
		bool getMinimum(std::vector<ltlplanner::nodePtr> &vertexes, ltlplanner::nodePtr &minimum, std::vector<ltlplanner::map_statePtr> &distance){
			bool res = false;
			ltlplanner::nodePtr min = vertexes.at(0);
			for(std::vector<ltlplanner::nodePtr>::const_iterator it_t = vertexes.begin(); it_t != vertexes.end(); ++it_t){
				ltlplanner::nodePtr aux = *it_t;
				if(getShortestDistance(aux, distance) < getShortestDistance(min, distance)){
					min = aux;
					res = true;
				}
			}
			minimum = min;
			return res;
		}
		
		//Regresa la distancia de un nodo al origen
		int getShortestDistance(ltlplanner::nodePtr &destination, std::vector<ltlplanner::map_statePtr> &distance){
			int d = 99999;
			for(std::vector<ltlplanner::map_statePtr>::const_iterator it = distance.begin(); it != distance.end(); ++it){
				ltlplanner::map_statePtr aux = *it;
				if(aux->id_a == destination->id){
					d = aux->dist;
				}
			}
			return d;
		}
		
		//Elimina nodo de vector de nodos
		bool removeNode(std::vector<ltlplanner::nodePtr> &states, int state_id){
			bool res = false;
			int place = -1;
			int i = 0;
			for(std::vector<ltlplanner::nodePtr>::const_iterator it = states.begin(); it != states.end(); ++it){
				ltlplanner::nodePtr aux = *it;
				if(aux->id == state_id){
					res = true;
					place = i;
					states.erase(states.begin() + place);
					return res;
				}
				i++;
			}
			return res;
		}
		
		//Indica si el nodo esta en el vector settled
		bool isSettled(int node_id, std::vector<ltlplanner::nodePtr> &settledNodes){
			bool res = false;
			for(std::vector<ltlplanner::nodePtr>::const_iterator it = settledNodes.begin(); it != settledNodes.end(); ++it){
				ltlplanner::nodePtr aux = *it;
				if(aux->id == node_id){
					res = true;
				}
			}
			return res;
		}
		
		//Devuelve la distancia entre dos nodos.
		float getDistance(ltlplanner::nodePtr &node, ltlplanner::nodePtr &target, std::vector<ltlplanner::edgePtr> &transitions){
			int dist = -1;
			for(std::vector<ltlplanner::edgePtr>::const_iterator it_t = transitions.begin(); it_t != transitions.end(); ++it_t){
				ltlplanner::edgePtr aux = *it_t;
				if(aux->id_a == node->id && aux->id_b == target->id){
					dist = aux->value;
				}
			}
			return dist;
		}
		
		//Devuelve la trayectoria mas corta entre nodo origen y otro nodo
		bool getPath(ltlplanner::nodePtr &target, std::vector<ltlplanner::nodePtr> &path, std::vector<ltlplanner::map_statePtr> &predecessors, std::vector<ltlplanner::nodePtr> &states){
			std::vector<ltlplanner::nodePtr> pathaux;
			ltlplanner::nodePtr step(new ltlplanner::node);
			ltlplanner::nodePtr aux(new ltlplanner::node);
			step = target;
			
			if(!predecessorsGet(step->id, aux, predecessors, states)){
				return false;
			}
			pathaux.push_back(step);
			while(predecessorsGet(step->id, aux, predecessors, states)){
				//predecessorsGet(step->id, aux, predecessors, states);
				step = aux;
				pathaux.push_back(step);
			}
			reverse(pathaux, path);
			
			return true;
		}
		
		bool predecessorsGet(int id, ltlplanner::nodePtr &node, std::vector<ltlplanner::map_statePtr> &predecessors, std::vector<ltlplanner::nodePtr> &states){
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
				getNodePtr(place, node, states);
			}
			return res;
		}
		
		void reverse(std::vector<ltlplanner::nodePtr> &pathaux, std::vector<ltlplanner::nodePtr> &path){
			for(std::vector<ltlplanner::nodePtr>::const_iterator it = pathaux.end()-1; it >= pathaux.begin(); --it){
				ltlplanner::nodePtr aux = *it;
				path.push_back(aux);
			}
		}
		
		void generatePlanMsg(std::vector<ltlplanner::nodePtr> &path, std::vector<ltlplanner::taskPtr> &plan, std::vector<ltlplanner::edgePtr> &transitions){
			int k = 0;
			for(std::vector<ltlplanner::nodePtr>::const_iterator it = path.begin(); it != (path.end() - 1); ++it){
				ltlplanner::nodePtr aux = *it;
				ltlplanner::nodePtr aux_next = *(it + 1);
				
				ltlplanner::edgePtr trans (new ltlplanner::edge);
				bool trans_bool = getTransition(aux, aux_next, trans, transitions);	
				
				if(trans_bool){
					ltlplanner::taskPtr prop(new ltlplanner::task);
    			prop->id = k;
  				prop->routine= trans->routine;
  				prop->name_mg = trans->group_name;
  				prop->trans_alphabet = trans->trans_alphabet;
  				prop->dtimes = trans->dtimes;
  				plan.push_back(prop);
  				k++;
				}
				
			}
		}
		
		bool getTransition(ltlplanner::nodePtr &ini, ltlplanner::nodePtr &fin, ltlplanner::edgePtr &trans, std::vector<ltlplanner::edgePtr> &transitions){
			bool res = false;
			int i = 0;
			int place = -1;
			for(std::vector<ltlplanner::edgePtr>::const_iterator it_t = transitions.begin(); it_t != transitions.end(); ++it_t){
				ltlplanner::edgePtr aux = *it_t;
				if(aux->id_a == ini->id && aux->id_b == fin->id){
					place = i;
					trans = aux;
					res = true;
				}
				i++;
			}
			return res;	
		}	
		
};

int main(int argc, char **argv){
	ros::init(argc, argv, "MANAGER");
	ros::NodeHandle n;
	Manager man(n);
	
	ros::Subscriber sub_reg = n.subscribe("/public/region", 100, &Manager::listeningRegions, &man);
	ros::Subscriber sub_aut = n.subscribe("/public/automata", 100, &Manager::listeningAutomatas, &man);
	ros::Subscriber sub_goa = n.subscribe("/public/goal", 100, &Manager::listeningGoals, &man);
	
	ros::spin();

  return 0;
}
