#include "ros/ros.h"
#include <sstream>
#include <vector>
#include <cstdlib>
#include <stdlib.h> 
//#include <random_numbers/random_numbers.h>

#include "ltlplanner/automaton.h"
#include "ltlplanner/region.h"
#include "ltlplanner/goal.h"
#include "ltlplanner/limit.h"
#include "ltlplanner/state.h"
#include "ltlplanner/transition.h"
#include "ltlplanner/plan.h"
#include "ltlplanner/task.h"
#include "ltlplanner/map_state.h"

#include "ltlplanner/automaton_msg.h"
#include "ltlplanner/region_msg.h"
#include "ltlplanner/goal_msg.h"
#include "ltlplanner/plan_msg.h"

//Planear
#include <ltlplanner/lowplanAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit_msgs/RobotTrajectory.h>

class Planner{
	private:
		ltlplanner::goal goal_;
		std::vector<ltlplanner::region> regions_;
		std::vector<ltlplanner::automaton> automatas_; 
		ltlplanner::plan plan_;
		bool validplan;
		ltlplanner::task actTask_;
		
		bool goalChange;
	public:
		Planner(){
			goalChange = false;
			validplan = false;
			plan_.id = -1;
		}
		
		~Planner(){
		
		}
		
		void goalCallback(const ltlplanner::goal_msgPtr& msg){
			if(goal_.id != msg->goal.id){
				std::cout << "Nuevo Goal" << std::endl;
				goal_ = msg->goal;
				goalChange = true;
				validplan = false;
				printGoal();
			}	
		}
		
		void printGoal(){
			std::cout << "Goal ID: " << goal_.id << std::endl;
			std::cout << "Goal: " << goal_.goal << std::endl;
			std::cout << "Id_automata: " << goal_.id_automata << std::endl;
		}
		
		void planCallback(const ltlplanner::plan_msgPtr& msg){
			if(plan_.id != msg->plan.id){
				std::cout << "Nuevo plan" << std::endl;
				plan_ = msg->plan;
				validplan = true;
				printPlan();
			}	
		}
		
		void printPlan(){
			std::cout << "Plan ID: " << plan_.id << std::endl;
		}
		
		ltlplanner::region getRegion(int region_id){
			ltlplanner::region res;
			int place = -1;
			int i = 0;
			for(std::vector<ltlplanner::region>::const_iterator it = regions_.begin(); it != regions_.end(); ++it){
				ltlplanner::region aux = *it;
				if(aux.id == region_id){
					place = i;
					res = regions_.at(place);
				}
				i++;
			}
			return res;
		}
		
		ltlplanner::automaton getAutomaton(int auto_id){
			ltlplanner::automaton res;
			int place = -1;
			int i = 0;
			for(std::vector<ltlplanner::automaton>::const_iterator it = automatas_.begin(); it != automatas_.end(); ++it){
				ltlplanner::automaton aux = *it;
				if(aux.id == auto_id){
					place = i;
					res = automatas_.at(place);
				}
				i++;
			}
			return res;
		}
		
		void printRegion(int region_id){
			ltlplanner::region aux = getRegion(region_id);
			std::cout << "Region ID: " << aux.id << std::endl;
			std::cout << "Name: " << aux.name_r << std::endl;
			std::cout << "AKA: " << aux.alias_r << std::endl;
		}
		
		void printAutomaton(int auto_id){
			ltlplanner::automaton aux = getAutomaton(auto_id);
			std::cout << "Auto ID: " << aux.id << std::endl;
			std::cout << "Name: " << aux.name << std::endl;
		}
		
		bool isinRegions(int region_id){
			bool res = false;
			for(std::vector<ltlplanner::region>::const_iterator it = regions_.begin(); it != regions_.end(); ++it){
				ltlplanner::region aux = *it;
				if(aux.id == region_id){
					res = true;
				}
			}
			return res;
		}
		
		bool isinAutomatas(int auto_id){
			bool res = false;
			for(std::vector<ltlplanner::automaton>::const_iterator it = automatas_.begin(); it != automatas_.end(); ++it){
				ltlplanner::automaton aux = *it;
				if(aux.id == auto_id){
					res = true;
				}
			}
			return res;
		}
		
		bool addRegion(ltlplanner::region &region_){
  		bool agr = isinRegions(region_.id);
  		bool res = false;
  		if(!agr){
  			regions_.push_back(region_);
  			res = true;
  		}
  		return res;
  	}
  	
  	bool addAutomaton(ltlplanner::automaton &auto_){
  		bool agr = isinAutomatas(auto_.id);
  		bool res = false;
  		if(!agr){
  			automatas_.push_back(auto_);
  			res = true;
  		}
  		return res;
  	}
		
		void regionCallback(const ltlplanner::region_msgPtr& msg){
			bool res = addRegion(msg->region);
			if(res){
				std::cout << "Nueva Region" << std::endl;
				printRegion(msg->region.id);
			}	
		}
		
		void automataCallback(const ltlplanner::automaton_msgPtr& msg){
			bool res = addAutomaton(msg->automata);
			if(res){
				std::cout << "Nuevo Automata" << std::endl;
				printAutomaton(msg->automata.id);
			}	
		}
		
		
		/*
		 * Metodo que planea en alto nivel
		 * Nota: Volver un RRT 
		 */
		void setTaskPoint(ltlplanner::taskPtr &task, std::vector<double> &points){
			srand(time(0));
			ltlplanner::region reg = getRegion(task->trans_alphabet.at(0));
			for(std::vector<ltlplanner::limit>::const_iterator it = reg.limits.begin(); it != reg.limits.end(); ++it){
				ltlplanner::limit lim = *it;
				double value = lim.min + (((double)rand()/RAND_MAX)*(lim.min - lim.max));
				points.push_back(value);
			}
		}
		
		//LLama accion para plan por punto
		bool planforPoint(ltlplanner::taskPtr &task, moveit_msgs::RobotTrajectory &traj){
			bool res = false;
			std::vector<double> points;
			setTaskPoint(task, points);
			actionlib::SimpleActionClient<ltlplanner::lowplanAction> ac("lowplan_srv", true);
			
			ROS_INFO("Waiting for action server to start.");
 			ac.waitForServer();
			ROS_INFO("Action server started, sending goal.");
  
	  	ltlplanner::lowplanGoal goal;
 	 		goal.id = task->id;
 	 		goal.group_name = task->name_mg;
 	 		
 	 		for(std::vector<double>::const_iterator it = points.begin(); it != points.end(); ++it){
				double value = *it;
				goal.values.push_back(value);
			}
 	 		
	  	ac.sendGoal(goal);

			while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ros::ok()){
    		usleep(50000);
  		}
  		
  		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
  			res = true;
  			ltlplanner::lowplanResultConstPtr res = ac.getResult();
  			traj = res->trajectory;
  		}
			return res;
		}
		
		//Convertir en hilo independiente
		void plan(){
			//Checa que el plan que se tiene sea valido
			while(validplan){
				for(std::vector<ltlplanner::task>::const_iterator it = plan_.tasks.begin(); it != plan_.tasks.end(); ++it){
					ltlplanner::task aux = *it;
					ltlplanner::taskPtr poin(new ltlplanner::task);
					poin->id = aux.id;
					poin->routine = aux.routine;
					poin->id_routine = aux.id_routine;
					poin->name_mg = aux.name_mg;
					poin->trans_alphabet = aux.trans_alphabet;
					poin->dtimes = aux.dtimes;
 
					std::vector<double> points;
					setTaskPoint(poin, points);
					moveit_msgs::RobotTrajectory traj;
				
					bool succ = planforPoint(poin, traj);
				
					if(succ){
						//Mandar al controlador
					}else{
						//Hacer algo
					}
				}
			}		
		}
};

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "PLANNER");
  ros::NodeHandle n;
  
  Planner planner;
	
  ros::Subscriber sub_goal = n.subscribe("/public/goal", 100, &Planner::goalCallback, &planner);
  ros::Subscriber sub_region = n.subscribe("/public/region", 100, &Planner::regionCallback, &planner);
  ros::Subscriber sub_auto = n.subscribe("/public/automata", 100, &Planner::automataCallback, &planner);
  ros::Subscriber sub_plan = n.subscribe("/public/plan", 100, &Planner::planCallback, &planner);
  ros::spin();

  return 0;
}
