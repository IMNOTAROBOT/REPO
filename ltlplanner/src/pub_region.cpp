#include "ros/ros.h"
#include <sstream>
#include <vector>
#include "ltlplanner/region.h"
#include "ltlplanner/region_msg.h"

class Region{
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_region;
		std::vector<ltlplanner::region> regions_;
		int index;
		
	public:
		Region(ros::NodeHandle &nh){
			nh_ = nh;
			pub_region = nh_.advertise<ltlplanner::region_msg>("public/region", 100);
			index = 1;
			setRegions();
		}
		
		~Region(){
		
		}
		
		bool setRegions(){
			ltlplanner::region area_0;
  		area_0.name_r = "P0";
  		area_0.alias_r = "The rest";
			area_0.id = 0;
			area_0.name_mg = "base";
			area_0.DOF_mg = 3;
	
			ltlplanner::limit lim_0_1;
			lim_0_1.min = -5.0;
			lim_0_1.max = 5.0;
	
			ltlplanner::limit lim_0_2;
			lim_0_2.min = -5.0;
			lim_0_2.max = 5.0;
	
			ltlplanner::limit lim_0_3;
			lim_0_3.min = -3.14;
			lim_0_3.max = 3.14;
	
			area_0.limits.push_back(lim_0_1);
			area_0.limits.push_back(lim_0_2);
			area_0.limits.push_back(lim_0_3);
		
			ltlplanner::region area_1;
			area_1.name_r = "P1";
			area_1.alias_r = "Kitchen";
			area_1.id = 1;
			area_1.name_mg = "base";
			area_1.DOF_mg = 3;
	
			ltlplanner::limit lim_1_1;
			lim_1_1.min = 0.9;
			lim_1_1.max = 1.1;
	
			ltlplanner::limit lim_1_2;
			lim_1_2.min = 3.8;
			lim_1_2.max = 4.2;
	
			ltlplanner::limit lim_1_3;
			lim_1_3.min = -3.14;
			lim_1_3.max = 3.14;
	
			area_1.limits.push_back(lim_1_1);
			area_1.limits.push_back(lim_1_2);
			area_1.limits.push_back(lim_1_3);
	
			ltlplanner::region area_2;
			area_2.name_r = "P2";
			area_2.alias_r = "Living";
			area_2.id = 2;
			area_2.name_mg = "base";
			area_2.DOF_mg = 3;
		
			ltlplanner::limit lim_2_1;
			lim_2_1.min = -4.1;
			lim_2_1.max = -3.8;
	
			ltlplanner::limit lim_2_2;
			lim_2_2.min = -4.1;
			lim_2_2.max = -3.8;
	
			ltlplanner::limit lim_2_3;
			lim_2_3.min = -3.14;
			lim_2_3.max = 3.14;
		
			area_2.limits.push_back(lim_2_1);
			area_2.limits.push_back(lim_2_2);
			area_2.limits.push_back(lim_2_3);
			
			regions_.push_back(area_0);
			regions_.push_back(area_1);
			regions_.push_back(area_2);
		}
		
		bool sendRegions(){
			bool res = false;
			if(ros::ok()){
				for(std::vector<ltlplanner::region>::const_iterator it = regions_.begin(); it != regions_.end(); ++it){
					ltlplanner::region aux = *it;
					ltlplanner::region_msg region_msg;
					region_msg.id = index;
					region_msg.region = aux;
					pub_region.publish(region_msg);
					index++;
					ros::spinOnce();
				}
				res = true;
			}
			return res;
		}
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "REGION_PUBLISHER");
  ros::NodeHandle n;

	Region region(n);
	
	if(ros::ok()){
		bool res = region.sendRegions();
	}
  return 0;
}