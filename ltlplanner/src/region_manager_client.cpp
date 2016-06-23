#include "ros/ros.h"

#include <ltlplanner/position.h>
#include "ltlplanner/region.h"
#include "ltlplanner/limit.h"

#include <sstream>
#include <vector>
#include <cstdlib>
#include <stdlib.h> 

//Servicios de region_manager
#include "ltlplanner/RM_inRegion.h"
#include "ltlplanner/RM_getRegionwithPoint.h"
#include "ltlplanner/RM_getNeighboursofRegion.h"
#include "ltlplanner/RM_rdmPointinRegion.h"
#include "ltlplanner/RM_middlePointofRegion.h"
#include "ltlplanner/RM_worldLimits.h"
#include "ltlplanner/RM_getAllRegions.h"
#include "ltlplanner/RM_ROIgraphGen.h"

//Publicacion de edo actual
#include <geometry_msgs/PoseStamped.h>

//Para planear con regiones de interes ROI
#include "ltlplanner/ROI_edge.h"
#include "ltlplanner/ROI_node.h"
#include "ltlplanner/ROI_graph.h"
#include "ltlplanner/map_state.h"

/*
* Clase prueba de c/servicio de region_manager
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "region_manager_client");
  ros::NodeHandle nh_;
  
  //1. Prueba inRegion
  std::cout << "Prueba inRegion" << std::endl;
  ros::ServiceClient inRegion_clt = nh_.serviceClient<ltlplanner::RM_inRegion>("inRegion_srv");
	ltlplanner::RM_inRegion msg_1;
	msg_1.request.values.push_back(1.0);
	msg_1.request.values.push_back(4.0);
	msg_1.request.values.push_back(0.0);
	msg_1.request.region_id = 1;
	
	bool res;
	
	if (inRegion_clt.call(msg_1)){
    res = msg_1.response.isInRegion;
    std::cout << "RES: " << res << std::endl;
  }else{
    std::cout << "Servicio no disponible" << std::endl;
  }
  
  ltlplanner::RM_inRegion msg_2;
	msg_2.request.values.push_back(0.8);
	msg_2.request.values.push_back(3.7);
	msg_2.request.values.push_back(-6.0);
	msg_2.request.region_id = 1;
	
	sleep(1.0);
	if(inRegion_clt.call(msg_2)){
   	res = msg_2.response.isInRegion;
    std::cout << "RES: " << res << std::endl;  
  }else{
    std::cout << "Servicio no disponible" << std::endl; 
  }
	
	//2. Prueba getRegionwithPoint
	std::cout << "Prueba getRegionwithPoint" << std::endl;
	ros::ServiceClient getRegionwithPoint_clt = nh_.serviceClient<ltlplanner::RM_getRegionwithPoint>("regionIdwithPoint_srv");
	ltlplanner::RM_getRegionwithPoint msg_3;
	msg_3.request.values.push_back(1.0);
	msg_3.request.values.push_back(4.0);
	msg_3.request.values.push_back(0.0);
	
	int res_id;
	if (getRegionwithPoint_clt.call(msg_3)){
    res_id = msg_3.response.region_id;
    std::cout << "RES: " << res_id << std::endl;
  }else{
    std::cout << "Servicio no disponible" << std::endl;
  }
  
  ltlplanner::RM_getRegionwithPoint msg_4;
	msg_4.request.values.push_back(0.8);
	msg_4.request.values.push_back(4.0);
	msg_4.request.values.push_back(0.0);
	
	sleep(1.0);
	if (getRegionwithPoint_clt.call(msg_4)){
    res_id = msg_4.response.region_id;
    std::cout << "RES: " << res_id << std::endl;
  }else{
    std::cout << "Servicio no disponible" << std::endl;
  }
  
  ltlplanner::RM_getRegionwithPoint msg_5;
	msg_5.request.values.push_back(-4.0);
	msg_5.request.values.push_back(-4.0);
	msg_5.request.values.push_back(0.0);
	
	sleep(1.0);
	if (getRegionwithPoint_clt.call(msg_5)){
    res_id = msg_5.response.region_id;
    std::cout << "RES: " << res_id << std::endl;
  }else{
    std::cout << "Servicio no disponible" << std::endl;
  }
	
	//3. Prueba de getNeighboursofRegion
	std::cout << "Prueba getNeighboursofRegion" << std::endl;
	ros::ServiceClient getNeighboursofRegion_clt = nh_.serviceClient<ltlplanner::RM_getNeighboursofRegion>("neighboursofRegion_srv");
	std::vector<long int> result;
	
	ltlplanner::RM_getNeighboursofRegion msg_6;
	msg_6.request.region_id = -1;
	
	if (getNeighboursofRegion_clt.call(msg_6)){
    result = msg_6.response.regions_id;
    std::cout << "RES: " << result.at(0) << std::endl;
  }else{
    std::cout << "Servicio no disponible" << std::endl;
  }
	
	sleep(0.5);
	ltlplanner::RM_getNeighboursofRegion msg_7;
	msg_7.request.region_id = 1;
	if (getNeighboursofRegion_clt.call(msg_7)){
    result = msg_7.response.regions_id;
    std::cout << "RES: " << result.at(0) << " " << result.at(1) << std::endl;
  }else{
    std::cout << "Servicio no disponible o region no valida" << std::endl;
  }
	
	//4. Prueba de rdmPointinRegion
	std::cout << "Prueba rdmPointinRegion" << std::endl;
	ros::ServiceClient rdmPointinRegion_clt = nh_.serviceClient<ltlplanner::RM_rdmPointinRegion>("rdmPointinRegion_srv");
	std::vector<double> res_d;
	sleep(0.5);
	ltlplanner::RM_rdmPointinRegion msg_8;
	msg_8.request.region_id = -1;
	if (rdmPointinRegion_clt.call(msg_8)){
    res_d = msg_8.response.values;
    std::cout << "RES: " << res_d.at(0) << " , " << res_d.at(1) << " , "<< res_d.at(2)<< std::endl;
  }else{
    std::cout << "Servicio no disponible o region no valida" << std::endl;
  }
  
	sleep(0.5);
	ltlplanner::RM_rdmPointinRegion msg_9;
	msg_9.request.region_id = 1;
	if (rdmPointinRegion_clt.call(msg_9)){
    res_d = msg_9.response.values;
    std::cout << "RES: " << res_d.at(0) << " , " << res_d.at(1) << " , "<< res_d.at(2)<< std::endl;
  }else{
    std::cout << "Servicio no disponible o region no valida" << std::endl;
  }
	
	//5. Prueba middlePointofRegion
	std::cout << "Prueba middlePointofRegion" << std::endl;
	ros::ServiceClient middlePointofRegion_clt = nh_.serviceClient<ltlplanner::RM_middlePointofRegion>("middlePointofRegion_srv");
	
	sleep(0.5);
	ltlplanner::RM_middlePointofRegion msg_10;
	msg_10.request.region_id = -1;
	if (middlePointofRegion_clt.call(msg_10)){
    res_d = msg_10.response.values;
    std::cout << "RES: " << res_d.at(0) << " , " << res_d.at(1) << " , "<< res_d.at(2)<< std::endl;
  }else{
    std::cout << "Servicio no disponible o region no valida" << std::endl;
  }
  
	sleep(0.5);
	ltlplanner::RM_middlePointofRegion msg_11;
	msg_11.request.region_id = 1;
	if (middlePointofRegion_clt.call(msg_11)){
    res_d = msg_11.response.values;
    std::cout << "RES: " << res_d.at(0) << " , " << res_d.at(1) << " , "<< res_d.at(2)<< std::endl;
  }else{
    std::cout << "Servicio no disponible o region no valida" << std::endl;
  }
	
	//6. Prueba worldLimits
	std::cout << "Prueba worldLimits" << std::endl;
	ros::ServiceClient worldLimits_clt = nh_.serviceClient<ltlplanner::RM_worldLimits>("worldLimits_srv");
	ltlplanner::limit x_axis;
	ltlplanner::limit y_axis;
	
	sleep(0.5);
	ltlplanner::RM_worldLimits msg_12;
	msg_12.request.client = "CLIENT";
	if (worldLimits_clt.call(msg_12)){
    x_axis = msg_12.response.x_axis;
    y_axis = msg_12.response.y_axis;
    std::cout << "RES: " << x_axis.min << " , " << x_axis.max << " , "<< y_axis.min << " , " << y_axis.max << std::endl;
  }else{
    std::cout << "Servicio no disponible" << std::endl;
  }
  
	//7. Prueba getAllRegions
	std::cout << "Prueba getAllRegions" << std::endl;
	ros::ServiceClient getAllRegions_clt = nh_.serviceClient<ltlplanner::RM_getAllRegions>("getAllRegions_srv");
	std::vector<ltlplanner::region> regions;
	
	sleep(0.5);
	ltlplanner::RM_getAllRegions msg_13;
	msg_13.request.client = "CLIENT";
	if (getAllRegions_clt.call(msg_13)){
    regions = msg_13.response.regions;
    std::cout << "RES: " << regions.at(0).id << " , " << regions.at(1).id << " , "<< regions.at(2).id << std::endl;
  }else{
    std::cout << "Servicio no disponible" << std::endl;
  }
	//8. Prueba getROIGraph
	std::cout << "Prueba getROIGraph" << std::endl;
	ros::ServiceClient getROIGraph_clt = nh_.serviceClient<ltlplanner::RM_ROIgraphGen>("RM_ROIgraphGen_srv");
  ltlplanner::ROI_graph graph;
  
  sleep(0.5);
	ltlplanner::RM_ROIgraphGen msg_14;
	msg_14.request.client = "CLIENT";
	if (getROIGraph_clt.call(msg_14)){
    graph = msg_14.response.graph;
    std::cout << "RES: " << graph.nodes.at(0).id_region << " , " << graph.nodes.at(1).id_region << " , "<< graph.nodes.at(2).id_region << std::endl;
    std::cout << "RES edge 0: " << graph.edges.at(0).id_a << " , " << graph.edges.at(0).id_b  << std::endl;
    std::cout << "RES edge 1:" << graph.edges.at(1).id_a << " , " << graph.edges.at(1).id_b  << std::endl;
    std::cout << "RES edge 2:" << graph.edges.at(2).id_a << " , " << graph.edges.at(2).id_b  << std::endl;
    std::cout << "RES edge 3:" << graph.edges.at(3).id_a << " , " << graph.edges.at(3).id_b  << std::endl;
  }else{
    std::cout << "Servicio no disponible" << std::endl;
  }
  
  return 0;
}
