#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
  m_Model = model;
  start_x *= 0.01; start_y *= 0.01; end_x *= 0.01; end_y*= 0.01;
  startNode = &m_Model.FindClosestNode(start_x, start_y);
  endNode = &m_Model.FindClosestNode(end_x, end_y);
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *currentNode) {
  std::vector<RouteModel::Node> finalPath;
  routeDistance = 0.0f;
  RouteModel::Node parentNode;
  while(currentNode->parent != nullptr) {
    finalPath.push_back(*currentNode);
    parentNode = *(currentNode->parent);
    routeDistance += currentNode->distance(parentNode);
    currentNode = currentNode->parent;
  } 
  finalPath.push_back(*currentNode);
  routeDistance *= m_Model.MetricScale();
  return finalPath;
}

void RoutePlanner::AStarSearch() {
  startNode->visited = true;
  openList.push_back(startNode);
  RouteModel::Node *currentNode = nullptr;
  while (openList.size() > 0) {
    currentNode = NextNode();  
    if (currentNode->distance(*endNode) == 0) {
      m_Model.path = ConstructFinalPath(currentNode);  
      return;
    }
    AddNeighbors(currentNode);  
  }
}


float RoutePlanner::CalculateHValue (const RouteModel::Node* current) {
  return current->distance(*endNode);
}  
RouteModel::Node* RoutePlanner::NextNode() {
  std::sort(openList.begin(), openList.end(), [](const auto& first, const auto second) {
    return first->h_value + first->g_value < second->h_value + second->g_value;     
  });
  RouteModel::Node* nextNode = openList.front();  
  openList.erase(openList.begin());
  return nextNode;
}  

void RoutePlanner::AddNeighbors(RouteModel::Node* currentNode) {
  currentNode->FindNeighbors();
  for (auto neighbor : currentNode->neighbors) {
    neighbor->parent = currentNode;
    neighbor->g_value = (currentNode->distance(*neighbor) + currentNode->g_value);
    neighbor->h_value = CalculateHValue(neighbor);
    openList.push_back(neighbor);
    neighbor->visited = true;
  }
}

  
  