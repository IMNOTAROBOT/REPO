#include <ros/ros.h>

#include "pr2_moveit_ltl/dijkstra_msg.h"

#include "pr2_moveit_ltl/automaton.h"
#include "pr2_moveit_ltl/area_proposition.h"
#include "pr2_moveit_ltl/limit.h"
#include "pr2_moveit_ltl/state.h"
#include "pr2_moveit_ltl/transition.h"
#include "pr2_moveit_ltl/node.h"
#include "pr2_moveit_ltl/edge.h"

#include <vector>
#include <cstdlib>

class Dijkstra
{
private:
	pr2_moveit_ltl::automaton auto_;
	int a_;
	int b_;
	int size_nodes;
	int size_edges;
	std::vector<node> nodes;
	std::vector<edge> edges;
  
public:
    
  Dijkstra(pr2_moveit_ltl::automaton &automata, int a, int b) 
  {
  	auto_ = automata;
  	a_ = a;
  	b_ = b; 
  	size_nodes = 0;
  	size_edges = 0;
  	setNodesEdges(); 
  }

  ~Dijkstra()
  {
   
  }

	void setNodesEdges(){
		int i = 0;
		int j = 0;
		for(std::vector<pr2_moveit_ltl::state>::const_iterator it = auto_.states.begin(); it != auto_.states.end(); ++it){
			pr2_moveit_ltl::nodePtr node_(new pr2_moveit_ltl::node);
			node_->id = *it.id;
			node_->value = 99999;
			nodes.push_back(node_);
			for(std::vector<pr2_moveit_ltl::transition>::const_iterator it_a = *it.transitions.begin(); it_a != *it.transitions.end(); ++it_a){
				pr2_moveit_ltl::edgePtr edge_(new pr2_moveit_ltl::edge);
				edge_->id = i;
				edge_->a = *it_a.initial_state;
				edge_->b = *it_a.final_state;
				edge_->value = *it_a.value;
				edges.push_back(edge_);
				i++;
			}
			j++;
		}
		size_nodes = j;
		size_edges = i;
	}
	
	// Using DIJKSTRA
	void setPath(std::vector<int> &res)
  {
    float dist[20];
    int prev[20];
    std::vector<pr2_moveit_ltl::node> Q;
    
    for(int i = 0; i < size_nodes; i++){
    	dist[i] = 99999;
    	prev[i] = -1;
    	Q.push_back(nodes.at(i));
    }
    
    dist[a] = 0;
    int act = -1;
    while (Q.size > 0 && act != b){
    	act = minDist(dist);
    	Q = erase(Q, act);
    	
    	int vecinos[] = getVecinos(act); 
    	int alt = 0;
    	for(int i = 0; i < vecinos.length(); i++){
    		alt = dist[act] + distancia(act, vecino[i])
    		if (alt < dist[i]){
    			dist[i] = alt;
    			prev[i] = act;
    		}
    	}
    }
    
    std::vector<int> result;
    int aux1 = b;
    while (prev[aux1] != -1){
    	result.push_back(aux1);
    	aux1 = prev[aux1];
    } 
    result.push_back(aux1); 
    
    for(int k = 0; k < result.size(); k++){
    	res.push_back(result.back());
    	result.pop_back();
    }
  }
  
  int minDist(int[] dist){
  
  }
  
};

bool generate_path(pr2_moveit_ltl::dijkstra_msg::Request  &req,
         pr2_moveit_ltl::dijkstra_msg::Response &res)
{
  
  return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "DIJKSTRA");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("plan_gen_dijkstra", generate_path);
  ROS_INFO("Ready to generate plan (DIJKSTRA).");
  ros::spin();

  return 0;
}
