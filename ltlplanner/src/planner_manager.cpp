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
<<<<<<< HEAD

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
#include "ltlplanner/object.h"

#define PI	3.1415926
=======

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
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
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
<<<<<<< HEAD
		ltlplanner::limit angle_axis;
=======
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
		
		//Colisiones
		moveit::planning_interface::MoveGroup* group;
		moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;  
		ros::Publisher collision_object_publisher;
<<<<<<< HEAD
		std::vector<ltlplanner::object> collisions_;
=======
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
	
	public:

  Planner(std::string name) :
   	as_(nh_, name, boost::bind(&Planner::executeCB, this, _1), false),
   	action_name_(name)
<<<<<<< HEAD
 	{	
 		std::cout << "PLANNER: Inicializando " <<std::endl;
=======
 	{
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
  	srand(142857);
  	//Inicializacion de pose actual
  	current_state.push_back(0.0);
    current_state.push_back(0.0);
    current_state.push_back(0.0);
    x_axis.min = -6.0;
		x_axis.max = 6.0;
		y_axis.min = -6.0;
		y_axis.max = 6.0;
<<<<<<< HEAD
		angle_axis.min = -3.14;
		angle_axis.max = 3.14;
=======
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
		
    //Subsripcion a pose de PR2
    sub_pose = nh_.subscribe("/ltlplanner/PR2pose", 1000, &Planner::getPR2pose, this);
  	
  	ac_ = new MoveBaseClient(client_,"lowcontroller_step", true);
  	
<<<<<<< HEAD
  	//Servicios de regiones
=======
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
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
<<<<<<< HEAD
  	//std::cout << "PLANNER: Frame de planeacion: "<< frameplan <<std::endl;
=======
  	std::cout << "Frame de planeacion: "<< frameplan <<std::endl;
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
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
<<<<<<< HEAD
  	//std::cout << "PLANNER: Duermo 10 s" << std::endl;
  	
  	addCollisionObjects(collisions_);
=======
  	std::cout << "PLANNER: Duermo " << std::endl;
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
  	//*****
  	
  	//Publicar accion
    as_.start();
<<<<<<< HEAD
    std::cout << "PLANNER: Listo"<< frameplan <<std::endl;
=======
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
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
<<<<<<< HEAD
  	std::cout << "PLANNER: Empiezo ejecucion ... " << std::endl;
=======
  	std::cout << "PLANNER: Empiezo executeCB " << std::endl;
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
    bool success = true;
    bool pathfound = false;
  	feedback_.done = false;
  	
    while(!ac_->waitForServer(ros::Duration(5.0))){
<<<<<<< HEAD
    	ROS_INFO("PLANNER : Waiting for low controller action server to come up");
=======
    	ROS_INFO("PLANNER : Waiting for lowplanner action server to come up");
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
  	}
  	
  	//Validar edo actual
  	int region_actId = -1;
  	ltlplanner::RM_getRegionwithPoint getRegion_msg;
  	getRegion_msg.request.values = current_state;
  	
  	if (getRegionwithPoint_clt.call(getRegion_msg)){
<<<<<<< HEAD
  		//std::cout << "PLANNER: Region actual " << getRegion_msg.response.region_id << std::endl;
=======
  		std::cout << "PLANNER: Region actual " << getRegion_msg.response.region_id << std::endl;
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
  		region_actId = getRegion_msg.response.region_id;
  	}else{
    	ROS_ERROR("PLANNER: Failed to call service getIdofRegionwithPoint");
    	success = false;
    }
  	
  	if(validateRegion(region_actId) && validateRegion(goal->region_id.at(0))){
  		getROIGraph();
<<<<<<< HEAD
  		//std::cout << "PLANNER: Grafo creado " << std::endl;
  		std::vector<ltlplanner::ROI_node> ROI_DKJpath;
  		
  		ros::Time secs_start_ROI =ros::Time::now();
  		
  		getROIplan(ROI_DKJpath, region_actId, goal->region_id.at(0));
  		
  		ros::Time secs_end_ROI =ros::Time::now(); 
			double time_total_ROI = secs_end_ROI.sec - secs_start_ROI.sec;
			
			std::cout << "PLANNER: Tiempo (s) para crear ROI path = " << time_total_ROI << std::endl;
  		std::cout << "PLANNER: trayectoria en ROI creada " << std::endl;
  		
  		/*for(std::vector<ltlplanner::ROI_node>::const_iterator it = ROI_DKJpath.begin(); it != ROI_DKJpath.end(); ++it){
  			ltlplanner::ROI_node aux = *it;
  			//std::cout << "PLANNER: trayectoria ROI: " << aux.id_region << std::endl;
  		}*/
  		
  		//Arbol RRT con ROI
  		//std::cout << "PLANNER: Crear arbol RRT con ROI" << std::endl;
  		std::vector<ltlplanner::position> result_;
  		double sizeStep = 0.3;
  		int pointsResolutionMap = 1000;
  		
  		double secs_start_RRT =ros::Time::now().toSec();
  		bool generated = generateGraphRRTPoints(sizeStep, pointsResolutionMap, result_, ROI_DKJpath, current_state);
  		double secs_end_RRT =ros::Time::now().toSec(); 
			double time_total_RRT = secs_end_RRT - secs_start_RRT;
  		
  		if(generated){
  			std::cout << "PLANNER: Tiempo (s) para crear RRT path = " << time_total_RRT << std::endl;
  			std::cout << "PLANNER: objetivos generados para RRT " << std::endl;
  		}else{
  			std::cout << "PLANNER: objetivos NO generados para RRT " << std::endl;
  			success = false;
  		}
  
  		/*int i = 0;
  		for(std::vector<ltlplanner::position>::const_iterator iter = result_.begin(); iter != result_.end(); ++iter){
  			ltlplanner::position posAux = *iter;
  			//std::cout << "PLANNER: punto: " << i << std::endl;
  			for(std::vector<double>::const_iterator k = posAux.values.begin(); k != posAux.values.end(); ++k){
  				//std::cout << "PLANNER: valor: " << *k << std::endl;
  			}
  			i++;
  		}*/
  		
  		double secs_start_TOT =ros::Time::now().toSec();
=======
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
  		
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
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

<<<<<<< HEAD
				double secs_start_STEP =ros::Time::now().toSec();
=======
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
  			ac_->sendGoal(goalp);
  			ac_->waitForResult();
  			
  			if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
<<<<<<< HEAD
    			ROS_INFO("PLANNER : Se ejecuto accion Bien");
    		}else{
    			ROS_INFO("PLANNER : Se ejecuto accion Mal");
    			success = false;
  			}
  			double secs_end_STEP =ros::Time::now().toSec(); 
				double time_total_STEP = secs_end_STEP - secs_start_STEP;
  			std::cout << "PLANNER: Tiempo (s) para un PASO = " << time_total_STEP << std::endl;
  		}
  		double secs_end_TOT =ros::Time::now().toSec(); 
			double time_total_TOT = secs_end_TOT - secs_start_TOT;
  		std::cout << "PLANNER: Tiempo (s) para crear TOTAL path plan = " << time_total_TOT << std::endl;
=======
    			ROS_INFO("PLANNER : Bien");
    		}else{
    			ROS_INFO("PLANNER : Mal");
    			success = false;
  			}
  			
  		}
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
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
<<<<<<< HEAD
  	msg.request.client = "PLANNER";
=======
  	msg.request.client = "HIGH_PLANNER";
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
  	
  	if (getAllRegions_clt.call(msg)){
  		res = true;
  		regions_ = msg.response.regions;
  	}
  	return res;
  }
  
  bool getROIGraph(){
  	bool res = false;
  	ltlplanner::RM_ROIgraphGen msg;
<<<<<<< HEAD
  	msg.request.client = "PLANNER";
=======
  	msg.request.client = "HIGH_PLANNER";
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
  	
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
<<<<<<< HEAD
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
=======
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
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
		}
	}
		
<<<<<<< HEAD
	//RRT para regiones de interes
	bool generateGraphRRTPoints(double sizeStep, int pointsResolutionMap, std::vector<ltlplanner::position> &result_, std::vector<ltlplanner::ROI_node> &ROI_DKJpath, std::vector<double> current_state_){
		//Variable indicadora si se encontro el camino
		bool res = false;
		
		//Indices e iterador maximo
		int iterator = 0;
		int index_node = 0;
		int index_edge = 0;
		
		//Auxiliares
		ltlplanner::position valsAct;
		valsAct.values = current_state_;
		
		//Inicializar arbol
		ltlplanner::ROI_graph tree;
		ltlplanner::ROI_node initial;
		ltlplanner::ROI_node actNode;
		initial.point.values = current_state_;
		initial.id_region = getRegionofPoint(valsAct.values);
		initial.index = index_node;
		initial.parent = -1;
		index_node ++;
		
		tree.initial_node = initial;
		tree.nodes.push_back(initial);
		
		actNode = initial;
		if(ROI_DKJpath.size() <= 1){
			return res;
		}
		
		for(int k = 0; k < ROI_DKJpath.size()-1; k++){
			//Checamos validez en region
			if(actNode.id_region == ROI_DKJpath.at(k).id_region){
				//std::cout << "PLANNER: Region objetivo: " << ROI_DKJpath.at(k+1).id_region << std::endl;
				iterator = 0;
				while(getRegionofPoint(actNode.point.values)!= ROI_DKJpath.at(k+1).id_region && iterator < pointsResolutionMap){
					//Generar punto aleatorio dentro del mapa
					std::vector<ltlplanner::limit> limites;
					std::vector<double> RdmVector;
					RdmVector.push_back(0.0);
					RdmVector.push_back(0.0);
					RdmVector.push_back(0.0);
					limites.push_back(x_axis);
					limites.push_back(y_axis);
					limites.push_back(angle_axis);
					getRandomPoint(limites, RdmVector);
				
					/*for(std::vector<double>::const_iterator it = RdmVector.begin(); it != RdmVector.end(); ++it){
							std::cout << "PLANNER: Punto aleatorio: " << *it << std::endl;
					}*/
					//Punto del arbol mas cercano a ese punto generado
					getNearestNode(tree.nodes, actNode, RdmVector);
				
					//Generar paso hacia ese punto
					std::vector<double> RdmVector2;
					RdmVector2.push_back(0.0);
					RdmVector2.push_back(0.0);
					RdmVector2.push_back(0.0);
					getStep(actNode.point.values, RdmVector, sizeStep, RdmVector2);
		
					/*for(std::vector<double>::const_iterator it2 = RdmVector2.begin(); it2 != RdmVector2.end(); ++it2){
					std::cout << "PLANNER: Paso: " << *it2 << std::endl;
					}*/
				
					//Checar colision
					//std::cout << "PLANNER: Checando colision " << std::endl;
					std::vector<double> vectorCollisionAux;
					vectorCollisionAux.push_back(RdmVector2.at(0));
					vectorCollisionAux.push_back(RdmVector2.at(1));
					vectorCollisionAux.push_back(0.1);
					bool collision = checkCollision(vectorCollisionAux);
				
					if(!collision){
						ltlplanner::ROI_node aux3;
						aux3.id_region = getRegionofPoint(RdmVector2);
						aux3.point.values = RdmVector2;
						aux3.index = index_node;
						aux3.parent = actNode.index;
=======
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
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
						index_node ++;
						tree.nodes.push_back(aux3);
						ltlplanner::ROI_edge edge_aux;
						edge_aux.index = index_edge;
						index_edge ++;
						edge_aux.id_a = actNode.index;
						edge_aux.id_b = aux3.index;
						tree.edges.push_back(edge_aux);
<<<<<<< HEAD
					
						ltlplanner::ROI_edge edge_aux2;
						edge_aux2.index = index_edge;
						index_edge ++;
						edge_aux2.id_b = actNode.index;
						edge_aux2.id_a = aux3.index;
						tree.edges.push_back(edge_aux2);
						//std::cout << "PLANNER: agregando punto a arbol " << std::endl;
						iterator ++;
					
					
					}else{
						//std::cout << "PLANNER: no agregando punto a arbol " << std::endl;
					}
					actNode = tree.nodes.at(tree.nodes.size()-1);
					//std::cout << "PLANNER: Region actual: " << getRegionofPoint(actNode.point.values) << std::endl;
				}	
			}
		}
		
		//Imprimiendo puntos hasta ahora
		/*std::cout << "PLANNER: Arbol nodos " << std::endl;
		for(std::vector<ltlplanner::ROI_node>::const_iterator k = tree.nodes.begin(); k != tree.nodes.end(); ++k){
			ltlplanner::ROI_node nodoAuxi = *k;
			std::cout << "PLANNER: nodos index: " << nodoAuxi.index << std::endl;
			std::cout << "PLANNER: nodos parent: " << nodoAuxi.parent << std::endl;
			for(std::vector<double>::const_iterator iters = nodoAuxi.point.values.begin(); iters != nodoAuxi.point.values.end(); ++iters){
				std::cout << "PLANNER: nodo values: " << *iters << std::endl;
			}
		}
		for(std::vector<ltlplanner::ROI_edge>::const_iterator k = tree.edges.begin(); k != tree.edges.end(); ++k){	
			ltlplanner::ROI_edge edgeAuxi = *k;
			std::cout << "PLANNER: edge values index: " << edgeAuxi.index << std::endl;
			std::cout << "PLANNER: edge values id_a: " << edgeAuxi.id_a << std::endl;
			std::cout << "PLANNER: edge values id_b: " << edgeAuxi.id_b << std::endl;
		}*/
				
		if(actNode.id_region == ROI_DKJpath.at(ROI_DKJpath.size()-1).id_region ){
			res = getTrajectoryRRTBack(result_, tree);
			if(res){
				std::cout << "PLANNER: Generando trayectoria dentro de RRT " << std::endl;
			}else{
				std::cout << "PLANNER: No se genero trayectoria dentro de RRT " << std::endl;
			}
		}else{
			std::cout << "PLANNER: No se genero trayectoria " << std::endl;
		}
=======
						getRandomNode(tree.nodes, actNode);
					}
				}	
			}
		}
		res = getTrajectoryRRT(result_, tree);
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
		return res;
	}
	
	void getStep(std::vector<double> &initial_point, std::vector<double> &vec_dir, double sizeStep, std::vector<double> &vec_res){
<<<<<<< HEAD
		double distance = sqrt(((vec_dir.at(0) - initial_point.at(0)) * (vec_dir.at(0) - initial_point.at(0))) + ((vec_dir.at(1) - initial_point.at(1)) * (vec_dir.at(1) - initial_point.at(1))));
		/*if(distance <= sizeStep){
			vec_res = vec_dir;
		}else{*/
			//Calcular angulos A y B para luego calcular x2 y y2 de punto a distancia step
			double ang_c = PI/2;
			double ang_a = asin(((vec_dir.at(0) - initial_point.at(0)) * sin(ang_c))/distance);
			double ang_b = asin(((vec_dir.at(1) - initial_point.at(1)) * sin(ang_c))/distance);
			
			vec_res.at(2) = vec_dir.at(2);
			vec_res.at(0) = initial_point.at(0) + (sizeStep * sin(ang_a)/ sin(ang_c));
			vec_res.at(1) = initial_point.at(1) + (sizeStep * sin(ang_b)/ sin(ang_c));
		//}
=======
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
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
	}
	
	bool checkCollision(std::vector<double> &point){
		bool res = false;
<<<<<<< HEAD
		/*std::vector<std::string> aux = planning_scene_interface->getKnownObjectNamesInROI(point.at(0) - 0.5, point.at(1) - 0.5, 0.0, point.at(0) + 0.5, point.at(1) + 0.5, 2.0, false);
		if(aux.size() > 0 ){
			res = true;
		}	*/
		//std::cout << "PLANNER: Objetos: " << collisions_.size() << std::endl;
		for(std::vector<ltlplanner::object>::const_iterator it = collisions_.begin(); it != collisions_.end(); ++it){
			ltlplanner::object aux = *it;
			if((point.at(0) > aux.x_min && point.at(0) < aux.x_max) && (point.at(1) > aux.y_min && point.at(1) < aux.y_max) ){
				res = true;
			}
		}
=======
		std::vector<std::string> aux = planning_scene_interface->getKnownObjectNamesInROI(point.at(0) - 0.5, point.at(1) - 0.5, 0.0, point.at(0) + 0.5, point.at(1) + 0.5, 1.0, false);
		if(aux.size() > 0 ){
			res = true;
		} 	
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
		return res;
	} 
	
	void getRandomNode(std::vector<ltlplanner::ROI_node> &nodes, ltlplanner::ROI_node &actNode){
		int place = rand() % nodes.size();         // v1 in the range 0 to 99
		actNode = nodes.at(place);
<<<<<<< HEAD
		//std::cout << "PLANNER: random place: " << place << std::endl;
	}
	
	void getNearestNode(std::vector<ltlplanner::ROI_node> &nodes, ltlplanner::ROI_node &actNode, std::vector<double> &RdmVector){
		float distance = 999.999;
		float actDist;
		int place = -1;
		int i = 0;
		for(std::vector<ltlplanner::ROI_node>::const_iterator it = nodes.begin(); it != nodes.end(); ++it){
			ltlplanner::ROI_node aux = *it;
			actDist = sqrt(((RdmVector.at(0) - aux.point.values.at(0)) * (RdmVector.at(0) - aux.point.values.at(0))) + ((RdmVector.at(1) - aux.point.values.at(1)) * (RdmVector.at(1) - aux.point.values.at(1))));
			if(actDist <= distance){
				place = i;
				distance = actDist; 
			}
			i ++;
		}
		actNode = nodes.at(place);
	}
	
	void getRandomPoint(std::vector<ltlplanner::limit> &limites, std::vector<double> &vector_){
		int i = 0;
		for(std::vector<ltlplanner::limit>::const_iterator it = limites.begin(); it != limites.end(); ++it){
			ltlplanner::limit aux = *it;
			double f = (double)rand() / RAND_MAX;
   		double raux = aux.min + f * (aux.max - aux.min);
   		vector_.at(i) = raux;
   		i++;
		}
=======
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
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
<<<<<<< HEAD
		}
		return res;
	}
	
	bool getTrajectoryRRTBack(std::vector<ltlplanner::position> &result_, ltlplanner::ROI_graph &tree){
		bool res = true;
		ltlplanner::ROI_node aux = tree.nodes.at(tree.nodes.size()-1);
		std::vector<ltlplanner::position> trajAux;
		while(aux.parent != -1){
			trajAux.push_back(aux.point);
			int place = getIndexOfNode(tree.nodes, aux.parent);
			aux = tree.nodes.at(place);
		}
		for(int i = trajAux.size()-1; i >= 0 ; i--){
			result_.push_back(trajAux.at(i));
		}
		return res;
	}
	
	int getIndexOfNode(std::vector<ltlplanner::ROI_node> &nodes_, int index){
		int place = -1;
		int i = 0;
		bool res = false;
		ltlplanner::ROI_node aux = nodes_.at(i);
		while(!res){
			if(aux.index == index){
				place = i;
				res = true;
			}
			i++;
			aux = nodes_.at(i);
		}
		return place;
	}
	
=======
		}
		return res;
	}
	
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
	int getRegionofPoint(std::vector<double> &values){
		int res;
		ltlplanner::RM_getRegionwithPoint getRegion_msg;
  	getRegion_msg.request.values = values;
  	if (getRegionwithPoint_clt.call(getRegion_msg)){
  		res = getRegion_msg.response.region_id;
  	}
		return res;
	}
<<<<<<< HEAD
	
	void addCollisionObjects(std::vector<ltlplanner::object> &collisions){
		ltlplanner::object aux1;
		aux1.id = 0;
		aux1.x_min = -5.0;
		aux1.x_max = 5.0;
		aux1.y_min = -5.0;
		aux1.y_max = -4.0;
		collisions.push_back(aux1);
		
		ltlplanner::object aux2;
		aux2.id = 1;
		aux2.x_min = -5.0;
		aux2.x_max = 5.0;
		aux2.y_min = 4.0;
		aux2.y_max = 5.0;
		collisions.push_back(aux2);
		
		ltlplanner::object aux3;
		aux3.id = 2;
		aux3.x_max = 5.0;
		aux3.x_min = 4.0;
		aux3.y_min = -5.0;
		aux3.y_max = 5.0;
		collisions.push_back(aux3);
		
		ltlplanner::object aux4;
		aux4.id = 3;
		aux4.x_max = -4.0;
		aux4.x_min = -5.0;
		aux4.y_min = -5.0;
		aux4.y_max = 5.0;
		collisions.push_back(aux4);
		
		ltlplanner::object aux5;
		aux5.id = 4;
		aux5.x_max = -1.5;
		aux5.x_min = -2.5;
		aux5.y_min = -5.0;
		aux5.y_max = -1.0;
		collisions.push_back(aux5);
		
		ltlplanner::object aux6;
		aux6.id = 5;
		aux6.x_max = 0.0;
		aux6.x_min = -3.5;
		aux6.y_min = -2.0;
		aux6.y_max = -0.5;
		collisions.push_back(aux6);
		
		ltlplanner::object aux7;
		aux7.id = 6;
		aux7.x_max = 2.6;
		aux7.x_min = 1.3;
		aux7.y_min = -5.0;
		aux7.y_max = -0.4;
		collisions.push_back(aux7);
		
		ltlplanner::object aux8;
		aux8.id = 7;
		aux8.x_max = 3.2;
		aux8.x_min = 1.3;
		aux8.y_min = -2.0;
		aux8.y_max = -0.4;
		collisions.push_back(aux8);
		
		ltlplanner::object aux9;
		aux9.id = 8;
		aux9.x_max = -1.2;
		aux9.x_min = -5.0;
		aux9.y_min = 1.5;
		aux9.y_max = 3.0;
		collisions.push_back(aux9);
		
		ltlplanner::object aux10;
		aux10.id = 9;
		aux10.x_min = -0.8;
		aux10.x_max = 0.8;
		aux10.y_min = 1.5;
		aux10.y_max = 5.0;
		collisions.push_back(aux10);
		
		ltlplanner::object aux11;
		aux11.id = 10;
		aux11.x_max = 5.0;
		aux11.x_min = 1.8;
		aux11.y_min = 1.5;
		aux11.y_max = 3.2;
		collisions.push_back(aux11);
	}
=======
>>>>>>> 5f79198d86948135cef3a03bcd2814573960aa96
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twolevelplanning_srv");

  Planner planner(ros::this_node::getName());
  ros::spin();

  return 0;
}

