#include <ros/ros.h>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <math.h>
//Acciones cliente y servicio
#include <actionlib/client/simple_action_client.h>
#include <ltlplanner/lowcontrol_stepAction.h>
#include <actionlib/server/simple_action_server.h>
#include <ltlplanner/planningRRTAction.h>

#include <ltlplanner/position.h>
#include "ltlplanner/region.h"
#include "ltlplanner/limit.h"

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

//Colisiones
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <map>
#include "geometry_msgs/Point.h"
#include <geometric_shapes/shape_operations.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>

#define PI	3.14159265359
/*
 * planeador RRT de dos capas. 
 */
 
typedef actionlib::SimpleActionClient<ltlplanner::lowcontrol_stepAction> MoveBaseClient;

class Planner{
	protected:

  	ros::NodeHandle nh_;
  	ros::NodeHandle client_;
  	actionlib::SimpleActionServer<ltlplanner::planningRRTAction> as_; 
  	std::string action_name_;
  
  	ltlplanner::planningRRTFeedback feedback_;
  	ltlplanner::planningRRTResult result_;
  
  	ltlplanner::ROI_graph graph_;
  	std::vector<ltlplanner::ROI_node> ROI_DKJpath_;
  	std::vector<ltlplanner::position> ROI_points_path_;
  
  	//Para RRT
  	ltlplanner::ROI_graph RRT_tree_;
  	std::vector<ltlplanner::ROI_node> RTT_DKJpath_;
  	std::vector<ltlplanner::position> RTT_points_path_;
  	
		MoveBaseClient* ac_;
	
		//Clientes de los servicios de region_manager
		ros::ServiceClient inRegion_clt;
		ros::ServiceClient getRegionwithPoint_clt;
		ros::ServiceClient getNeighboursofRegion_clt;
		ros::ServiceClient rdmPointinRegion_clt;
		ros::ServiceClient middlePointofRegion_clt;
		ros::ServiceClient worldLimits_clt;
		ros::ServiceClient getAllRegions_clt;
		ros::ServiceClient getROIGraph_clt;
	
		//Pose actual
		ros::Subscriber sub_pose;
		geometry_msgs::PoseStamped actPose_;
		std::vector<double> current_state;
	
		//Edo actual de automata
		int auto_id;
		int edo_idAct;
	
		//Regions
		std::vector<ltlplanner::region> regions_;
		ltlplanner::limit x_axis;
		ltlplanner::limit y_axis;
		
		//Colisiones
		moveit::planning_interface::MoveGroup* group;
		moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;  
		ros::Publisher collision_object_publisher;
	
	public:

  Planner(std::string name) :
   	as_(nh_, name, boost::bind(&Planner::executeCB, this, _1), false),
   	action_name_(name)
 	{
  	srand(142857);
  	//Inicializacion de pose actual
  	current_state.push_back(0.0);
    current_state.push_back(0.0);
    current_state.push_back(0.0);
    x_axis.min = -6.0;
		x_axis.max = 6.0;
		y_axis.min = -6.0;
		y_axis.max = 6.0;
		
    //Subsripcion a pose de PR2
    sub_pose = nh_.subscribe("/ltlplanner/PR2pose", 1000, &Planner::getPR2pose, this);
  	
  	ac_ = new MoveBaseClient(client_,"lowcontroller_step", true);
  	
  	inRegion_clt = nh_.serviceClient<ltlplanner::RM_inRegion>("inRegion_srv");
  	getRegionwithPoint_clt = nh_.serviceClient<ltlplanner::RM_getRegionwithPoint>("regionIdwithPoint_srv");
		getNeighboursofRegion_clt = nh_.serviceClient<ltlplanner::RM_getNeighboursofRegion>("neighboursofRegion_srv");
		rdmPointinRegion_clt = nh_.serviceClient<ltlplanner::RM_rdmPointinRegion>("rdmPointinRegion_srv");
		middlePointofRegion_clt = nh_.serviceClient<ltlplanner::RM_middlePointofRegion>("middlePointofRegion_srv");
		worldLimits_clt = nh_.serviceClient<ltlplanner::RM_worldLimits>("worldLimits_srv");
  	getAllRegions_clt = nh_.serviceClient<ltlplanner::RM_getAllRegions>("getAllRegions_srv");
  	getROIGraph_clt = nh_.serviceClient<ltlplanner::RM_ROIgraphGen>("RM_ROIgraphGen_srv");
  	
  	// Collision with moveIt
  	
  	group = new moveit::planning_interface::MoveGroup("base");
    group->startStateMonitor();
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    
    collision_object_publisher = nh_.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  	while(collision_object_publisher.getNumSubscribers() < 1)
  	{
    	ros::WallDuration sleep_t(0.5);
    	sleep_t.sleep();
  	}
    
    //Agregar objeto que colisiona
  	moveit_msgs::CollisionObject co;
  	std::string frameplan = group->getPlanningFrame();
  	std::cout << "Frame de planeacion: "<< frameplan <<std::endl;
  	co.header.frame_id = group->getPlanningFrame();
  	co.id= "muros";
  	shapes::Mesh* m = shapes::createMeshFromResource("package://ltlplanner/models/ENV_6.dae");
  	shape_msgs::Mesh co_mesh;
  	shapes::ShapeMsg co_mesh_msg;
  	shapes::constructMsgFromShape(m,co_mesh_msg);
  	co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
  	co.meshes.resize(1);
  	co.meshes[0] = co_mesh;
  	co.mesh_poses.resize(1);
  	co.mesh_poses[0].position.x = -5.0;
  	co.mesh_poses[0].position.y = -5.0;
  	co.mesh_poses[0].position.z = 0.0;
  	co.mesh_poses[0].orientation.w= 0.7071;
  	co.mesh_poses[0].orientation.x= 0.7071 ;
  	co.mesh_poses[0].orientation.y= 0.0;
  	co.mesh_poses[0].orientation.z= 0.0;

  	co.meshes.push_back(co_mesh);
  	co.mesh_poses.push_back(co.mesh_poses[0]);
  	co.operation = co.ADD;
  
  	/* Publish and sleep (to view the visualized results) */
  	collision_object_publisher.publish(co);
  	ros::WallDuration sleep_time(10.0);
  	sleep_time.sleep();

  	std::vector<moveit_msgs::CollisionObject> collision_objects;  
  	collision_objects.push_back(co);
  	planning_scene_interface->addCollisionObjects(collision_objects);
  	sleep(10.0);
  	std::cout << "PLANNER: Duermo " << std::endl;
  	//*****
  	
  	//Publicar accion
    as_.start();
  }

  ~Planner(void){
  	if(ac_ != NULL){
      delete ac_;
    }
    
    if(group != NULL){
      delete group;
    }
    if(planning_scene_interface != NULL){
      delete planning_scene_interface;
    }
  }

	void getPR2pose(const geometry_msgs::PoseStamped::ConstPtr& msg){
  		current_state[0] = msg->pose.position.x;
  		current_state[1] = msg->pose.position.y;
  		current_state[2] = msg->pose.orientation.w;
	}
  
  //Ejecucion de la accion
  void executeCB(const ltlplanner::planningRRTGoalConstPtr &goal)
  {
  	std::cout << "PLANNER: Empiezo executeCB " << std::endl;
    bool success = true;
    bool pathfound = false;
  	feedback_.done = false;
  	
    while(!ac_->waitForServer(ros::Duration(5.0))){
    	ROS_INFO("PLANNER : Waiting for lowplanner action server to come up");
  	}
  	
  	//Validar edo actual
  	int region_actId = -1;
  	ltlplanner::RM_getRegionwithPoint getRegion_msg;
  	getRegion_msg.request.values = current_state;
  	
  	if (getRegionwithPoint_clt.call(getRegion_msg)){
  		std::cout << "PLANNER: Region actual " << getRegion_msg.response.region_id << std::endl;
  		region_actId = getRegion_msg.response.region_id;
  	}else{
    	ROS_ERROR("PLANNER: Failed to call service getIdofRegionwithPoint");
    	success = false;
    }
  	
  	if(validateRegion(region_actId) && validateRegion(goal->region_id.at(0))){
  		getROIGraph();
  		std::cout << "PLANNER: Grafo creado " << std::endl;
  		std::vector<ltlplanner::ROI_node> ROI_DKJpath;
  		getROIplan(ROI_DKJpath, region_actId, goal->region_id.at(0));
  		std::cout << "PLANNER: trayectoria en ROI creada " << std::endl;
  		for(std::vector<ltlplanner::ROI_node>::const_iterator it = ROI_DKJpath.begin(); it != ROI_DKJpath.end(); ++it){
  			ltlplanner::ROI_node aux = *it;
  			std::cout << "PLANNER: trayectoria ROI: " << aux.id_region << std::endl;
  		}
  		
  		//Arbol RRT con ROI
  		std::vector<ltlplanner::position> result_;
  		double sizeStep = 0.1;
  		int pointsResolutionMap = 2000;
  		generateGraphRRTPoints(sizeStep, pointsResolutionMap, result_, ROI_DKJpath, current_state);
  		std::cout << "PLANNER: objetivos generados " << std::endl;
  		
  		for(std::vector<ltlplanner::position>::const_iterator it = result_.begin(); it != result_.end(); ++it){
  			ltlplanner::position aux = *it;
  			if (as_.isPreemptRequested() || !ros::ok()){
     			feedback_.done = false;
       		as_.setPreempted();
       		success = false;
       	}
       	
       	feedback_.done = false;
      	as_.publishFeedback(feedback_);
       	
       	ltlplanner::lowcontrol_stepGoal goalp;
  			
  			goalp.goalPose.header.frame_id = "map";
  			goalp.goalPose.header.stamp = ros::Time::now();

  			goalp.goalPose.pose.position.x = aux.values.at(0);
  			goalp.goalPose.pose.position.y = aux.values.at(1);
  			goalp.goalPose.pose.orientation.w = aux.values.at(2);

  			ac_->sendGoal(goalp);
  			ac_->waitForResult();
  			
  			if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    			ROS_INFO("PLANNER : Bien");
    		}else{
    			ROS_INFO("PLANNER : Mal");
    			success = false;
  			}
  			
  		}
  	}else{
  		success = false;
  	}
		
    if(success)
    {
      result_.done = true;
  		as_.setSucceeded(result_);
    }else{
    	result_.done = false;
    	as_.setAborted (result_);
    } 
  }
  
  bool validateRegion(int region_actId){
  	bool res = false;
  	if(isinRegions(region_actId)){
  		res = true;
  	}
  	return res;
  }
  
  bool isinRegions(int region_id){
  		getRegions();
			bool res = false;
			for(std::vector<ltlplanner::region>::const_iterator it = regions_.begin(); it != regions_.end(); ++it){
				ltlplanner::region aux = *it;
				if(aux.id == region_id){
					res = true;
				}
			}
			return res;
	}
  
  //Punto medio en region
  bool generateGraphMiddlePoints(std::vector<ltlplanner::position> &result_, std::vector<ltlplanner::ROI_node> &ROI_path){
  	bool res = true;
  	ltlplanner::RM_middlePointofRegion msg;
  	for(std::vector<ltlplanner::ROI_node>::const_iterator it = ROI_path.begin(); it != ROI_path.end(); ++it){
			ltlplanner::ROI_node aux = *it;
			msg.request.region_id = aux.id_region;
			if (middlePointofRegion_clt.call(msg)){
				ltlplanner::position vals;
				vals.values = msg.response.values;
  			result_.push_back(vals);
  		}
		}
  	return res;
  }
  
  bool getRegions(){
  	bool res = false;
  	ltlplanner::RM_getAllRegions msg;
  	msg.request.client = "HIGH_PLANNER";
  	
  	if (getAllRegions_clt.call(msg)){
  		res = true;
  		regions_ = msg.response.regions;
  	}
  	return res;
  }
  
  bool getROIGraph(){
  	bool res = false;
  	ltlplanner::RM_ROIgraphGen msg;
  	msg.request.client = "HIGH_PLANNER";
  	
  	if (getROIGraph_clt.call(msg)){
  		res = true;
  		graph_ = msg.response.graph;
  	}
  	return res;
  }
  
  //I. Metodo Raiz para DIKJSTRA
  bool getROIplan(std::vector<ltlplanner::ROI_node> &ROI_DKJpath, long int region_id_a, long int region_id_b){
  	bool res = false;
  	res = lookforPath(graph_, ROI_DKJpath, region_id_a, region_id_b);
  	return res;
  }
  
  /*
   * Funciones parte de DIKJSTRA para planear en Areas de interes
  */
	
	//II. Metodo Dikjstra para generar una trayectoria corta y valida dentro de ROIs.
	bool lookforPath(ltlplanner::ROI_graph &auto_, std::vector<ltlplanner::ROI_node> &path_res, long int region_id_a, long int region_id_b){
		bool res = false;
			
		//Auxiliary initialization
		std::vector<ltlplanner::ROI_node> settledNodes;
		std::vector<ltlplanner::ROI_node> unSettledNodes;
		std::vector<ltlplanner::map_state> predecessors;
		std::vector<ltlplanner::map_state> distance;
			
		//Initial and final node
		ltlplanner::ROI_node initial = getNode(region_id_a, auto_.nodes);
		ltlplanner::ROI_node end = getNode(region_id_b, auto_.nodes);
			
		//Dikjstra
		ltlplanner::map_state dist_aux_ini;
		dist_aux_ini.id_a = initial.id_region;
		dist_aux_ini.dist = 0.0;
		
		distance.push_back(dist_aux_ini);
		unSettledNodes.push_back(initial);
			
		while(unSettledNodes.size() > 0){	
			ltlplanner::ROI_node min;
			bool minfind = getMinimum(unSettledNodes, min, distance);
			settledNodes.push_back(min);
			bool removed = removeNode(unSettledNodes, min.id_region);
			findMinimalDistances(min, unSettledNodes, predecessors, distance, settledNodes, auto_.nodes, auto_.edges);
		}
		
		bool pathmade = getPath(end, path_res, predecessors, auto_.nodes);
		if(pathmade){
			res = true;
		}
		return res;
	}
		
		//Funcion que encuentra la distancia minima de todos los vecinos de un nodo hacia el origen
		void findMinimalDistances(ltlplanner::ROI_node &node, std::vector<ltlplanner::ROI_node> &unSettledNodes, 
			std::vector<ltlplanner::map_state> &predecessors, std::vector<ltlplanner::map_state> &distance, std::vector<ltlplanner::ROI_node> &settledNodes, std::vector<ltlplanner::ROI_node> &states, std::vector<ltlplanner::ROI_edge> &transitions){
			
			std::vector<ltlplanner::ROI_node> adjacentNodes;
			getNeighbors(node, adjacentNodes, transitions, states, settledNodes);
			
			for(std::vector<ltlplanner::ROI_node>::const_iterator it = adjacentNodes.begin(); it != adjacentNodes.end(); ++it){
				ltlplanner::ROI_node aux = *it;
				if(getShortestDistance(aux, distance) > (getShortestDistance(node, distance) + getDistance(node, aux, transitions))){
					ltlplanner::map_state dist_aux;
					dist_aux.id_a = aux.id_region;
					dist_aux.dist = getShortestDistance(node, distance) + getDistance(node, aux, transitions);
					addDistance(dist_aux, distance);
					
					ltlplanner::map_state ante_aux;
					ante_aux.id_a = aux.id_region;
					ante_aux.id_b = node.id_region;
					addPredecessors(ante_aux, predecessors);
					unSettledNodes.push_back(aux);
				}
			}
		}
		
		//Agrega nuevos valores de distancias a vector de distancias
		void addDistance(ltlplanner::map_state &dist, std::vector<ltlplanner::map_state> &distance){
			bool found = false;
			int i = 0;
			int place = 0;
			for(std::vector<ltlplanner::map_state>::const_iterator it = distance.begin(); it != distance.end(); ++it){
				ltlplanner::map_state aux = *it;
				if(aux.id_a == dist.id_a){
					found = true;
					place = i;
				}
				i++;
			}
			if(found){
				distance.at(place).dist = dist.dist;
			}else{
				distance.push_back(dist);
			}
		
		}
		
		void addPredecessors(ltlplanner::map_state &map, std::vector<ltlplanner::map_state> &predecessors){
			bool found = false;
			int i = 0;
			int place = 0;
			for(std::vector<ltlplanner::map_state>::const_iterator it = predecessors.begin(); it != predecessors.end(); ++it){
				ltlplanner::map_state aux = *it;
				if(aux.id_a == map.id_a){
					found = true;
					place = i;
				}
				i++;
			}
			if(found){
				predecessors.at(place).id_b = map.id_b;
			}else{
				predecessors.push_back(map);
			}
		
		}
		
		//Regresa vector con los nodos vecinos de un nodo.
		bool getNeighbors(ltlplanner::ROI_node &node , std::vector<ltlplanner::ROI_node> &adjacentNodes, std::vector<ltlplanner::ROI_edge> &transitions, std::vector<ltlplanner::ROI_node> &states, std::vector<ltlplanner::ROI_node> &settledNodes){
			bool res = false;
			for(std::vector<ltlplanner::ROI_edge>::const_iterator it_t = transitions.begin(); it_t != transitions.end(); ++it_t){
				ltlplanner::ROI_edge aux = *it_t;
				if(aux.id_a == node.id_region && !isSettled(aux.id_b, settledNodes)){
					ltlplanner::ROI_node target;
					bool boolendstate = getNodePtr(aux.id_b, target, states);
					if(boolendstate){
						adjacentNodes.push_back(target);
					}
					res = boolendstate;
				}
			}
			return res;
		}
		
		//Regresa el valor de un nodo.
		ltlplanner::ROI_node getNode(int state_id, std::vector<ltlplanner::ROI_node> &states){
			ltlplanner::ROI_node state_;
			int place = -1;
			int i = 0;
			for(std::vector<ltlplanner::ROI_node>::const_iterator it = states.begin(); it != states.end(); ++it){
				ltlplanner::ROI_node aux = *it;
				if(aux.id_region == state_id){
					place = i;
					state_.id_region = aux.id_region;
					state_.value = aux.value;
				}
				i++;
			}
			return state_;
		}
		//Regresa un nodo.
		bool getNodePtr(int state_id, ltlplanner::ROI_node &state_, std::vector<ltlplanner::ROI_node> &states){
			bool res = false;
			int place = -1;
			int i = 0;
			for(std::vector<ltlplanner::ROI_node>::const_iterator it = states.begin(); it != states.end(); ++it){
				ltlplanner::ROI_node aux = *it;
				if(aux.id_region == state_id){
					state_ = aux;
					res = true;
				}
				i++;
			}
			return res;
		}
		
		//Regresa el nodo dentro del vector vertexes con menor distancia se;alada en el vector distancia
		bool getMinimum(std::vector<ltlplanner::ROI_node> &vertexes, ltlplanner::ROI_node &minimum, std::vector<ltlplanner::map_state> &distance){
			bool res = false;
			ltlplanner::ROI_node min = vertexes.at(0);
			for(std::vector<ltlplanner::ROI_node>::const_iterator it_t = vertexes.begin(); it_t != vertexes.end(); ++it_t){
				ltlplanner::ROI_node aux = *it_t;
				if(getShortestDistance(aux, distance) < getShortestDistance(min, distance)){
					min = aux;
					res = true;
				}
			}
			minimum = min;
			return res;
		}
		
		//Regresa la distancia de un nodo al origen
		int getShortestDistance(ltlplanner::ROI_node &destination, std::vector<ltlplanner::map_state> &distance){
			int d = 99999;
			for(std::vector<ltlplanner::map_state>::const_iterator it = distance.begin(); it != distance.end(); ++it){
				ltlplanner::map_state aux = *it;
				if(aux.id_a == destination.id_region){
					d = aux.dist;
				}
			}
			return d;
		}
		
		//Elimina nodo de vector de nodos
		bool removeNode(std::vector<ltlplanner::ROI_node> &states, int state_id){
			bool res = false;
			int place = -1;
			int i = 0;
			for(std::vector<ltlplanner::ROI_node>::const_iterator it = states.begin(); it != states.end(); ++it){
				ltlplanner::ROI_node aux = *it;
				if(aux.id_region == state_id){
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
		bool isSettled(int node_id, std::vector<ltlplanner::ROI_node> &settledNodes){
			bool res = false;
			for(std::vector<ltlplanner::ROI_node>::const_iterator it = settledNodes.begin(); it != settledNodes.end(); ++it){
				ltlplanner::ROI_node aux = *it;
				if(aux.id_region == node_id){
					res = true;
				}
			}
			return res;
		}
		
		//Devuelve la distancia entre dos nodos.
		float getDistance(ltlplanner::ROI_node &node, ltlplanner::ROI_node &target, std::vector<ltlplanner::ROI_edge> &transitions){
			int dist = -1;
			for(std::vector<ltlplanner::ROI_edge>::const_iterator it_t = transitions.begin(); it_t != transitions.end(); ++it_t){
				ltlplanner::ROI_edge aux = *it_t;
				if(aux.id_a == node.id_region && aux.id_b == target.id_region){
					dist = aux.value;
				}
			}
			return dist;
		}
		
		//Devuelve la trayectoria mas corta entre nodo origen y otro nodo
		bool getPath(ltlplanner::ROI_node &target, std::vector<ltlplanner::ROI_node> &path, std::vector<ltlplanner::map_state> &predecessors, std::vector<ltlplanner::ROI_node> &states){
			std::vector<ltlplanner::ROI_node> pathaux;
			ltlplanner::ROI_node step;
			ltlplanner::ROI_node aux;
			step = target;
			
			if(!predecessorsGet(step.id_region, aux, predecessors, states)){
				return false;
			}
			pathaux.push_back(step);
			while(predecessorsGet(step.id_region, aux, predecessors, states)){
				step = aux;
				pathaux.push_back(step);
			}
			reverse(pathaux, path);
			
			return true;
		}
		
		bool predecessorsGet(int id, ltlplanner::ROI_node &node, std::vector<ltlplanner::map_state> &predecessors, std::vector<ltlplanner::ROI_node> &states){
			bool res = false;
			int place = -1;
			for(std::vector<ltlplanner::map_state>::const_iterator it = predecessors.begin(); it != predecessors.end(); ++it){
				ltlplanner::map_state aux = *it;
				if(aux.id_a == id){
					place = aux.id_b;
					res = true;
				}
			}
			if(res){
				getNodePtr(place, node, states);
			}
			return res;
		}
		
	void reverse(std::vector<ltlplanner::ROI_node> &pathaux, std::vector<ltlplanner::ROI_node> &path){
		for(std::vector<ltlplanner::ROI_node>::const_iterator it = pathaux.end()-1; it >= pathaux.begin(); --it){
			ltlplanner::ROI_node aux = *it;
			path.push_back(aux);
		}
	}
		
	//RRT para regiones de interes
	bool generateGraphRRTPoints(double sizeStep, int pointsResolutionMap, std::vector<ltlplanner::position> &result_, std::vector<ltlplanner::ROI_node> &ROI_DKJpath, std::vector<double> current_state_){
		bool res = false;
		int k = 3;
		int index_node = 0;
		int index_edge = 0;
		//Pedir punto aleatorio dentro de area especifica
		ltlplanner::RM_rdmPointinRegion msg;
		ltlplanner::position valsAct;
		valsAct.values = current_state_;
		std::vector<double> vectorAux;
		std::vector<double> vectorAux2;
		vectorAux2.push_back(0.0);
		vectorAux2.push_back(0.0);
		vectorAux2.push_back(0.0);
		
		//Inicializar arbol
		ltlplanner::ROI_graph tree;
		ltlplanner::ROI_node initial;
		ltlplanner::ROI_node actNode;
		initial.point.values = current_state_;
		initial.id_region = getRegionofPoint(valsAct.values);
		initial.index = index_node;
		index_node ++;
		
		tree.initial_node = initial;
		tree.nodes.push_back(initial);
		
		actNode = initial;
		
		for(std::vector<ltlplanner::ROI_node>::const_iterator it = ROI_DKJpath.begin(); it != ROI_DKJpath.end(); ++it){
			ltlplanner::ROI_node aux = *it;
			msg.request.region_id = aux.id_region;
			
			while(getRegionofPoint(actNode.point.values)!= aux.id_region){
				if (rdmPointinRegion_clt.call(msg)){
					vectorAux = msg.response.values;
					getStep(actNode.point.values, vectorAux, sizeStep, vectorAux2);
					bool collision = checkCollision(vectorAux2);
					if(!collision){
						ltlplanner::ROI_node aux3;
						aux3.id_region = aux.id_region;
						aux3.point.values = vectorAux2;
						aux3.index = index_node;
						index_node ++;
						tree.nodes.push_back(aux3);
						ltlplanner::ROI_edge edge_aux;
						edge_aux.index = index_edge;
						index_edge ++;
						edge_aux.id_a = actNode.index;
						edge_aux.id_b = aux3.index;
						tree.edges.push_back(edge_aux);
						getRandomNode(tree.nodes, actNode);
					}
				}	
			}
		}
		res = getTrajectoryRRT(result_, tree);
		return res;
	}
	
	void getStep(std::vector<double> &initial_point, std::vector<double> &vec_dir, double sizeStep, std::vector<double> &vec_res){
		double distance = sqrt(((initial_point.at(0) - vec_dir.at(0)) * (initial_point.at(0) - vec_dir.at(0))) + ((initial_point.at(1) - vec_dir.at(1)) * (initial_point.at(1) - vec_dir.at(1))));
		if(distance <= sizeStep){
			vec_res = vec_dir;
		}else{
			//Calcular angulos A y B para luego calcular x2 y y2 de punto a distancia step
			double ang_c = PI/2;
			double ang_a = asin((vec_dir.at(0) * sin(ang_c))/distance);
			double ang_b = asin((vec_dir.at(1) * sin(ang_c))/distance);
			
			vec_res.at(2) = vec_dir.at(2);
			vec_res.at(0) = (sizeStep * sin(ang_a)/ sin(ang_c));
			vec_res.at(1) = (sizeStep * sin(ang_b)/ sin(ang_c));
		}
	}
	
	bool checkCollision(std::vector<double> &point){
		bool res = false;
		std::vector<std::string> aux = planning_scene_interface->getKnownObjectNamesInROI(point.at(0) - 0.5, point.at(1) - 0.5, 0.0, point.at(0) + 0.5, point.at(1) + 0.5, 1.0, false);
		if(aux.size() > 0 ){
			res = true;
		} 	
		return res;
	} 
	
	void getRandomNode(std::vector<ltlplanner::ROI_node> &nodes, ltlplanner::ROI_node &actNode){
		int place = rand() % nodes.size();         // v1 in the range 0 to 99
		actNode = nodes.at(place);
	}
	
	bool getTrajectoryRRT(std::vector<ltlplanner::position> &result_, ltlplanner::ROI_graph &tree){
		bool res = false;
		std::vector<ltlplanner::ROI_node> ROI_DKJpath;
		res = lookforPath(tree, ROI_DKJpath, tree.nodes.at(0).index, tree.nodes.at(tree.nodes.size()-1).index);
		if(res){
			for(std::vector<ltlplanner::ROI_node>::const_iterator it = ROI_DKJpath.begin(); it != ROI_DKJpath.end(); ++it){
				ltlplanner::ROI_node aux = *it;
				result_.push_back(aux.point);
			}
		}
		return res;
	}
	
	int getRegionofPoint(std::vector<double> &values){
		int res;
		ltlplanner::RM_getRegionwithPoint getRegion_msg;
  	getRegion_msg.request.values = values;
  	if (getRegionwithPoint_clt.call(getRegion_msg)){
  		res = getRegion_msg.response.region_id;
  	}
		return res;
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twolevelplanning_srv");

  Planner planner(ros::this_node::getName());
  ros::spin();

  return 0;
}
