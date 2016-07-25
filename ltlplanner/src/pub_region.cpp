#include "ros/ros.h"
#include <sstream>
#include <vector>
#include "ltlplanner/region.h"
#include "ltlplanner/limit.h"
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
			index = 0;
			setRegions();
			sleep(10.0);
		}
		
		~Region(){
		
		}
		
		bool setRegions(){
			ltlplanner::region area_0;
  		area_0.name_r = "P0";
  		area_0.alias_r = "Pasillo";
			area_0.id = 0;
			area_0.name_mg = "base";
			area_0.DOF_mg = 3;
	
			ltlplanner::limit lim_0_1;
			lim_0_1.min = -4.5;
			lim_0_1.max = 4.5;
	
			ltlplanner::limit lim_0_2;
			lim_0_2.min = -0.4;
			lim_0_2.max = 1.8;
	
			ltlplanner::limit lim_0_3;
			lim_0_3.min = -3.14;
			lim_0_3.max = 3.14;
	
			area_0.limits.push_back(lim_0_1);
			area_0.limits.push_back(lim_0_2);
			area_0.limits.push_back(lim_0_3);
		
			area_0.idneighbours.push_back(1);
			area_0.idneighbours.push_back(2);
			area_0.idneighbours.push_back(3);
			area_0.idneighbours.push_back(4);
			area_0.idneighbours.push_back(5);
			
			ltlplanner::region area_1;
			area_1.name_r = "P1";
			area_1.alias_r = "Cuarto_P1";
			area_1.id = 1;
			area_1.name_mg = "base";
			area_1.DOF_mg = 3;
	
			ltlplanner::limit lim_1_1;
			lim_1_1.max = -3.0;
			lim_1_1.min = -4.5;
	
			ltlplanner::limit lim_1_2;
			lim_1_2.min = -4.0;
			lim_1_2.max = -2.0;
	
			ltlplanner::limit lim_1_3;
			lim_1_3.min = -3.14;
			lim_1_3.max = 3.14;
	
			area_1.limits.push_back(lim_1_1);
			area_1.limits.push_back(lim_1_2);
			area_1.limits.push_back(lim_1_3);
	
			area_1.idneighbours.push_back(0);
			
			ltlplanner::region area_2;
			area_2.name_r = "P2";
			area_2.alias_r = "Cuarto_P2";
			area_2.id = 2;
			area_2.name_mg = "base";
			area_2.DOF_mg = 3;
		
			ltlplanner::limit lim_2_1;
			lim_2_1.min = 0.0;
			lim_2_1.max = 1.5;
	
			ltlplanner::limit lim_2_2;
			lim_2_2.min = -4.0;
			lim_2_2.max = -2.0;
	
			ltlplanner::limit lim_2_3;
			lim_2_3.min = -3.14;
			lim_2_3.max = 3.14;
		
			area_2.limits.push_back(lim_2_1);
			area_2.limits.push_back(lim_2_2);
			area_2.limits.push_back(lim_2_3);
			
			area_2.idneighbours.push_back(0);
			
			ltlplanner::region area_3;
			area_3.name_r = "P3";
			area_3.alias_r = "Cuarto_P3";
			area_3.id = 3;
			area_3.name_mg = "base";
			area_3.DOF_mg = 3;
		
			ltlplanner::limit lim_3_1;
			lim_3_1.max = 4.0;
			lim_3_1.min = 3.0;
	
			ltlplanner::limit lim_3_2;
			lim_3_2.min = -4.0;
			lim_3_2.max = -2.0;
	
			ltlplanner::limit lim_3_3;
			lim_3_3.min = -3.14;
			lim_3_3.max = 3.14;
		
			area_3.limits.push_back(lim_3_1);
			area_3.limits.push_back(lim_3_2);
			area_3.limits.push_back(lim_3_3);
			
			area_3.idneighbours.push_back(0);
			
			ltlplanner::region area_4;
			area_4.name_r = "P4";
			area_4.alias_r = "Cuarto_P4";
			area_4.id = 4;
			area_4.name_mg = "base";
			area_4.DOF_mg = 3;
		
			ltlplanner::limit lim_4_1;
			lim_4_1.max = -0.5;
			lim_4_1.min = -1.5;
	
			ltlplanner::limit lim_4_2;
			lim_4_2.min = 2.5;
			lim_4_2.max = 4.5;
	
			ltlplanner::limit lim_4_3;
			lim_4_3.min = -3.14;
			lim_4_3.max = 3.14;
		
			area_4.limits.push_back(lim_4_1);
			area_4.limits.push_back(lim_4_2);
			area_4.limits.push_back(lim_4_3);
			
			area_4.idneighbours.push_back(0);
			
			ltlplanner::region area_5;
			area_5.name_r = "P5";
			area_5.alias_r = "Cuarto_P5";
			area_5.id = 5;
			area_5.name_mg = "base";
			area_5.DOF_mg = 3;
		
			ltlplanner::limit lim_5_1;
			lim_5_1.max = -0.5;
			lim_5_1.min = -2.0;
	
			ltlplanner::limit lim_5_2;
			lim_5_2.min = 2.5;
			lim_5_2.max = 4.5;
	
			ltlplanner::limit lim_5_3;
			lim_5_3.min = -3.14;
			lim_5_3.max = 3.14;
		
			area_5.limits.push_back(lim_5_1);
			area_5.limits.push_back(lim_5_2);
			area_5.limits.push_back(lim_5_3);
			
			area_5.idneighbours.push_back(0);
			
			regions_.push_back(area_0);
			regions_.push_back(area_1);
			regions_.push_back(area_2);
			regions_.push_back(area_3);
			regions_.push_back(area_4);
			regions_.push_back(area_5);
		}
		
		bool sendRegions(){
			bool res = false;
			while(ros::ok()){
				sleep(1.0);
				for(std::vector<ltlplanner::region>::const_iterator it = regions_.begin(); it != regions_.end(); ++it){
					sleep(0.5);
					ltlplanner::region aux = *it;
					ltlplanner::region_msg region_msg;
					region_msg.id = index;
					region_msg.region = aux;
					region_msg.totalRegions = 6;
					pub_region.publish(region_msg);
					index++;
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
	bool res = region.sendRegions();
  return 0;
}
