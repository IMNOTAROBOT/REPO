#include "ros/ros.h"
#include <sstream>
#include "ltlplanner/goal.h"
#include "ltlplanner/goal_msg.h"

class Goal{
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_goal;
		ltlplanner::goal goal_;
		int index;
		
	public:
		Goal(ros::NodeHandle &nh){
			nh_ = nh;
			pub_goal = nh_.advertise<ltlplanner::goal_msg>("/public/goal", 1);
			index = 1;
			setGoal();
			sleep(10.0);
		}
		
		~Goal(){
		
		}
		
		bool setGoal(){
			goal_.id = index;
			goal_.goal = "EXAMPLE";
			goal_.id_automata = 0;
		}
		
		bool sendGoal(){
			bool res = false;
			while(ros::ok()){
				sleep(10.0);
				ltlplanner::goal_msg goal_msg;
				goal_msg.id = index;
				goal_msg.goal = goal_;
				pub_goal.publish(goal_msg);
				index++;
				res = true;
			}
			return res;
		}
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "GOAL_PUBLISHER");
  ros::NodeHandle n;

	Goal goal(n);
	
	bool res = goal.sendGoal();
	
  return 0;
}
