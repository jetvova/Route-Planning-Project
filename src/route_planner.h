#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    float GetDistance() const {return routeDistance;}
    void AStarSearch();

  private:
    RouteModel &m_Model;
  	RouteModel::Node* startNode;
    RouteModel::Node* endNode;
  	std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *currentNode);
    std::vector<RouteModel::Node*> openList;
    float routeDistance;
    float CalculateHValue (const RouteModel::Node* current);
  	RouteModel::Node* NextNode(); 
    void AddNeighbors (RouteModel::Node* node);
  
};
#endif