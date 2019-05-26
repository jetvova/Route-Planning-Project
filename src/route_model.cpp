#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
	int counter = 0;
  	for (Model::Node n : this -> Nodes()) {
    	RouteModel::Node *add = new RouteModel::Node(counter, this, n);
     	this -> m_nodes.push_back(*add);
    	counter++;
    }
  	CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap() {
  for (const Model::Road &road : Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      for (int index : Ways()[road.way].nodes) {
        if (node_to_road.find(index) == node_to_road.end()) {
          node_to_road[index] = std::vector<const Model::Road*> ();
        }
        node_to_road[index].push_back(&road);
      }
    }
  }
}

RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
  Node *closestNode = nullptr;
  for (int currentIndex : node_indices) {
    Node currentNode = parent_model->SNodes()[currentIndex];  
    if (this->distance(currentNode) != 0 && !currentNode.visited) {
      if ( (closestNode == nullptr) || this->distance(currentNode) < this->distance(*closestNode) ) {
        closestNode = &parent_model->SNodes()[currentIndex];    
      }
    }
  }
  return closestNode;
}

void RouteModel::Node::FindNeighbors() {
  for(auto & road : parent_model->node_to_road[this->index]) {
    RouteModel::Node *result = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
    if (result) {
      this->neighbors.push_back(result);
    }
  }
}

RouteModel::Node& RouteModel::FindClosestNode(float x, float y) {
  RouteModel::Node inputNode;
  inputNode.x = x;
  inputNode.y = y;
  float closestDistance = std::numeric_limits<float>::max();
  float currentDistance;
  int closestIndex;
  
  for (const Model::Road& road : Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      for (int currentIndex : Ways()[road.way].nodes) {
      	currentDistance = inputNode.distance(SNodes()[currentIndex]);  
        if (currentDistance < closestDistance) {
          closestDistance = currentDistance;  
          closestIndex = currentIndex;
        }
      }     
    } 
  }
  return SNodes()[closestIndex];
}


