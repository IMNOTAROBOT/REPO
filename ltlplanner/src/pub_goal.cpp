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
			pub_goal = nh_.advertise<ltlplanner::goal_msg>("public/goal", 100);
			index = 1;
			setGoal();
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
			if(ros::ok()){
				ltlplanner::goal_msg goal_msg;
				goal_msg.id = index;
				goal_msg.goal = goal_;
				pub_goal.publish(goal_msg);
				index++;
				res = true;
				ros::spinOnce();
			}
			return res;
		}
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "GOAL_PUBLISHER");
  ros::NodeHandle n;

	Goal goal(n);
	
	if(ros::ok()){
		bool res = goal.sendGoal();
		ros::spinOnce();
	}
	
	ros::spin();
  return 0;
}
