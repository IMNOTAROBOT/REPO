#include "ros/ros.h"
#include <gazebo_msgs/ModelStates.h>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cstring>
#include <string>
#include <cstdlib>
 
class Pose_Publisher{
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_pose;
		ros::Subscriber sub;
		
	public:
		Pose_Publisher(ros::NodeHandle &nh){
			nh_ = nh;
			sub = nh_.subscribe("/gazebo/model_states", 1000, &Pose_Publisher::publishPR2pose, this);
			pub_pose = nh_.advertise<geometry_msgs::PoseStamped>("/ltlplanner/PR2pose", 1000);
		}
		
		~Pose_Publisher(){
		
		}
		
		void publishPR2pose(const gazebo_msgs::ModelStates::ConstPtr& msg){
  		geometry_msgs::PoseStamped pub_msg;
  		pub_msg.header.frame_id = "base_link";
  		pub_msg.header.stamp = ros::Time::now();
  		
  		int it = findPR2place(msg->name);
  		
  		if(it != -1){
  			pub_msg.pose.position.x = msg->pose[it].position.x;
  			pub_msg.pose.position.y = msg->pose[it].position.y;
  			pub_msg.pose.position.z = msg->pose[it].position.z;
  			pub_msg.pose.orientation.x = msg->pose[it].orientation.x;
  			pub_msg.pose.orientation.y = msg->pose[it].orientation.y;
  			pub_msg.pose.orientation.z = msg->pose[it].orientation.z;
  			pub_msg.pose.orientation.w = msg->pose[it].orientation.w;
  			pub_pose.publish(pub_msg);
  		}
  		
		}
		
		int findPR2place(const std::vector<std::string> &names){
			int res = -1;
			int i = 0;
			for(std::vector<std::string>::const_iterator it = names.begin(); it != names.end(); ++it){
				std::string aux = *it;
				std::string pr2 = "pr2";
				if(aux.compare(pr2) == 0){
					res = i;
				}
				i ++;
			}
			return res;
		}
  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Pose_Publisher");
  ros::NodeHandle n;

	Pose_Publisher publisher(n);
	ros::spin();
  return 0;
}
