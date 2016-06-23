#include "ros/ros.h"
#include <sstream>
#include <vector>
#include <cstdlib>
#include <stdlib.h> 
#include "ltlplanner/region.h"
#include "ltlplanner/limit.h"
#include "ltlplanner/region_msg.h"

#include "ltlplanner/RM_inRegion.h"
#include "ltlplanner/RM_getRegionwithPoint.h"
#include "ltlplanner/RM_getNeighboursofRegion.h"
#include "ltlplanner/RM_rdmPointinRegion.h"
#include "ltlplanner/RM_middlePointofRegion.h"
#include "ltlplanner/RM_worldLimits.h"
#include "ltlplanner/RM_getAllRegions.h"
#include "ltlplanner/RM_ROIgraphGen.h"

//Para grafo de areas de interes
//Para planear con regiones de interes ROI
#include "ltlplanner/ROI_edge.h"
#include "ltlplanner/ROI_node.h"
#include "ltlplanner/ROI_graph.h"

class RegionManager{
	private:
		ros::NodeHandle nh_;
		std::vector<ltlplanner::region> regions_;
		ltlplanner::ROI_graph graph_;
		ros::Subscriber sub_reg;
		ros::ServiceServer inRegion_srv;
		ros::ServiceServer regionIdwithPoint_srv;
		ros::ServiceServer neighboursofRegion_srv;
		ros::ServiceServer rdmPointinRegion_srv;
		ros::ServiceServer middlePointofRegion_srv;
		ros::ServiceServer worldLimits_srv;
		ros::ServiceServer getAllRegions_srv;
		ros::ServiceServer RM_ROIgraphGen_srv;
		
		ltlplanner::limit x_axis;
		ltlplanner::limit y_axis;
		long int totalRegions;
		long int knownRegions;
		
	public:
		RegionManager(ros::NodeHandle &nh){
			srand(142857);
			totalRegions = 999;
			knownRegions = 0;
			x_axis.min = -6.0;
			x_axis.max = 6.0;
			y_axis.min = -6.0;
			y_axis.max = 6.0;
			nh_ = nh;
			sub_reg = nh_.subscribe("/public/region", 100, &RegionManager::listeningRegions, this);
			
			inRegion_srv = nh_.advertiseService("inRegion_srv", &RegionManager::inPointinRegionSRV, this);
			regionIdwithPoint_srv = nh_.advertiseService("regionIdwithPoint_srv", &RegionManager::regionIdwithPointSRV, this);
			neighboursofRegion_srv = nh_.advertiseService("neighboursofRegion_srv", &RegionManager::neighboursofRegionSRV, this);
			rdmPointinRegion_srv = nh_.advertiseService("rdmPointinRegion_srv", &RegionManager::rdmPointinRegionSRV, this);
			middlePointofRegion_srv = nh_.advertiseService("middlePointofRegion_srv", &RegionManager::middlePointofRegionSRV, this);
			worldLimits_srv = nh_.advertiseService("worldLimits_srv", &RegionManager::getWorldLimitsSRV, this);
			getAllRegions_srv = nh_.advertiseService("getAllRegions_srv", &RegionManager::getAllRegionsSRV, this);
			RM_ROIgraphGen_srv = nh_.advertiseService("RM_ROIgraphGen_srv", &RegionManager::generateGraphSRV, this);
		}
		
		~RegionManager(){
		
		}
		
		void listeningRegions(const ltlplanner::region_msgPtr& msg){
  		totalRegions = msg->totalRegions;
  		bool res = addRegion(msg->region);
  		if(res){
  			knownRegions++;
  			std::cout << "REGION_MANAGER::Se agrego region: " << msg->region.id << std::endl;
  		}
		}
		
		bool addRegion(ltlplanner::region &reg){
  		bool agr = isinRegions(reg);
  		bool res = false;
  		if(!agr){
  			regions_.push_back(reg);
  			res = true;
  		}
  		return res;
  	}
  	
  	bool isinRegions(ltlplanner::region &reg){
			bool res = false;
			for(std::vector<ltlplanner::region>::const_iterator it = regions_.begin(); it != regions_.end(); ++it){
				ltlplanner::region aux = *it;
				if(aux.id == reg.id){
					res = true;
				}
			}
			return res;
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
		
		int getRegionIndex(int region_id){
			int index = -1;
			int i = 0;
			for(std::vector<ltlplanner::region>::const_iterator it = regions_.begin(); it != regions_.end(); ++it){
				ltlplanner::region aux = *it;
				if(aux.id == region_id){
					index = i;
				}
				i++;
			}
			return index;
		}
		
		//Funciones de regiones
		bool isPointinRegion(std::vector<double> &point, int region_id){
			bool res = false;
			int sum = 0;
			int index = getRegionIndex(region_id);
			if (index != -1){
				ltlplanner::region reg= regions_.at(index);
				int sizep = point.size();
				int sizel = reg.limits.size();
				if(sizep == sizel){
					int i = 0;
					for(std::vector<ltlplanner::limit>::const_iterator it = reg.limits.begin(); it != reg.limits.end(); ++it){
						ltlplanner::limit aux = *it;
						if(point.at(i) >= aux.min && point.at(i) <= aux.max){
							sum++;
						}
						i++;
					}
					if (sum == sizep){
						res = true;
					}
				}
			}
			return res;
		}
		
		int getIdofRegionwithPoint(std::vector<double> &point){
			int res = -1;
			int index = -1;
			int i = 0;
			for(std::vector<ltlplanner::region>::const_iterator it = regions_.begin(); it != regions_.end(); ++it){
				ltlplanner::region aux = *it;
				if(isPointinRegion(point, aux.id)){
					index = i;
				}
				i++;
			}
			if(index != -1){
				res = regions_.at(index).id;
			}
			return res;
		}
		
		bool getNeighboursofRegion(int region_id, std::vector<long int> &neighbours_id){
			bool res = false;
			int index = getRegionIndex(region_id);
			if(index != -1){
				ltlplanner::region aux = regions_.at(index);
				neighbours_id = aux.idneighbours;
				res = true;
			}
			return res;
		}
		
		std::vector<double> getRandomPointfromRegion(int region_id){
			std::vector<double> res;
			int index = getRegionIndex(region_id);
			if(index != -1){
				ltlplanner::region reg = regions_.at(index);
				for(std::vector<ltlplanner::limit>::const_iterator it = reg.limits.begin(); it != reg.limits.end(); ++it){
					ltlplanner::limit aux = *it;
					double f = (double)rand() / RAND_MAX;
   				double raux = aux.min + f * (aux.max - aux.min);	
   				res.push_back(raux);	
				}
			}
			return res;
		}
		
		std::vector<double> getMiddlePointofRegion(int region_id){
			std::vector<double> res;
			int index = getRegionIndex(region_id);
			if(index != -1){
				ltlplanner::region reg = regions_.at(index);
				for(std::vector<ltlplanner::limit>::const_iterator it = reg.limits.begin(); it != reg.limits.end(); ++it){
					ltlplanner::limit aux = *it;
   				double raux = aux.min + ((aux.max - aux.min)/2);	
   				res.push_back(raux);	
				}
			}
			return res;
		}
		
		void generateGraph(ltlplanner::ROI_graph &graph){
			int i = 0;
			int j = 0;
			for(std::vector<ltlplanner::region>::const_iterator it = regions_.begin(); it != regions_.end(); ++it){
				ltlplanner::region aux = *it;
				ltlplanner::ROI_node aux_n;
				aux_n.index = i;
				aux_n.id_region = aux.id;
				aux_n.value = 0.0;
				graph.nodes.push_back(aux_n);
				for(std::vector<long int>::const_iterator iter = aux.idneighbours.begin(); iter != aux.idneighbours.end(); ++iter){
					long int aux_l = *iter;
					ltlplanner::ROI_edge aux_e;
					aux_e.index = j;
					aux_e.id_a = aux.id;
					aux_e.id_b = aux_l;
					aux_e.value = 1.0;
					graph.edges.push_back(aux_e);
					j++;
				}
				i++;
			}
		}
		
		//Funciones de servicios
		bool inPointinRegionSRV(ltlplanner::RM_inRegion::Request  &req, ltlplanner::RM_inRegion::Response &res) {
  		bool resv = false;
  		if(totalRegions == knownRegions){
  			resv = true;
  			res.isInRegion = isPointinRegion(req.values, req.region_id);
  		}
  		return resv;
		}
		
		bool regionIdwithPointSRV(ltlplanner::RM_getRegionwithPoint::Request  &req, ltlplanner::RM_getRegionwithPoint::Response &res){
			bool resv = false;
  		if(totalRegions == knownRegions){
  			resv = true;
  			res.region_id = getIdofRegionwithPoint(req.values);
  		}
  		return resv;
		}
		
		bool neighboursofRegionSRV(ltlplanner::RM_getNeighboursofRegion::Request  &req, ltlplanner::RM_getNeighboursofRegion::Response &res){
			bool resv = false;
			std::vector<long int> neighbours_id;
			resv = getNeighboursofRegion(req.region_id, neighbours_id);
			if(resv && totalRegions == knownRegions){
				res.done = true;
				res.regions_id = neighbours_id;
			}else{
				res.done = false;
			}
			return resv;
		}
		
		bool rdmPointinRegionSRV(ltlplanner::RM_rdmPointinRegion::Request  &req, ltlplanner::RM_rdmPointinRegion::Response &res){
			bool resv = false;
			if(totalRegions == knownRegions && isinRegions(req.region_id)){
  			resv = true;
  			res.values = getRandomPointfromRegion(req.region_id);
  		}
			return resv;
		}
		
		bool middlePointofRegionSRV(ltlplanner::RM_middlePointofRegion::Request  &req, ltlplanner::RM_middlePointofRegion::Response &res){
			bool resv = false;
			if(totalRegions == knownRegions && isinRegions(req.region_id)){
  			resv = true;
  			res.values = getMiddlePointofRegion(req.region_id);
  		}
			return resv;
		}
		
		bool getWorldLimitsSRV(ltlplanner::RM_worldLimits::Request  &req, ltlplanner::RM_worldLimits::Response &res){
			bool resv = true;
			res.x_axis.min = x_axis.min;
			res.x_axis.max = x_axis.max;
			res.y_axis.min = y_axis.min;
			res.y_axis.max = y_axis.max;
			return resv;
		}
		
		bool getAllRegionsSRV(ltlplanner::RM_getAllRegions::Request  &req, ltlplanner::RM_getAllRegions::Response &res){
			bool resv = false;
			if(totalRegions == knownRegions){
  			resv = true;
  			res.regions = regions_;
  		}
			return resv;
		}
		
		bool generateGraphSRV(ltlplanner::RM_ROIgraphGen::Request  &req, ltlplanner::RM_ROIgraphGen::Response &res){
			bool resv = false;
			if(totalRegions == knownRegions){
  			resv = true;
  			generateGraph(graph_);
				res.graph = graph_;
  		}
			return resv;
		}
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "region_manager_srv");
  ros::NodeHandle n;

	RegionManager man(n);
  ROS_INFO("Ready to manage regions");
  ros::spin();

  return 0;
}
