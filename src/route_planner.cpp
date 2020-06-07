#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
    start_node->parent = nullptr;
    start_node->visited = true;
    std::cout << "starting at: "<< start_node->x << ", " << start_node->y
    << "\n ending at: " << end_node->x << ", " << end_node->y << "\n";
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h;
    h = node->distance(*end_node);
    return h;
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->neighbors.clear();
    current_node->FindNeighbors();
    for (auto n : current_node->neighbors){
        n->parent = current_node;
        n->h_value = this->CalculateHValue(n);
        n->g_value = current_node->g_value + current_node->distance(*n);
        if(!n->visited){
            open_list.push_back(n);
            n->visited = true;
        }
    }
    
}


RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node* res;
    sort(open_list.begin(), open_list.end(), this->compare);
    res = open_list.back();
    open_list.pop_back();
    return res;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    int counter = 0;
    std::vector<RouteModel::Node> path_found;
    path_found.push_back(*current_node);
    //    for (int i = 0; i < m_counter; ++i){
    //        std:: cout << current_node->parent->x << "\n";
    //        distance += current_node->distance(*current_node->parent);
    //        path_found.push_back(*(current_node->parent));
    //        current_node = current_node->parent;
    //    }
    while (current_node->parent){
        //std::cout << "Current Parent Address is \t " << current_node->parent << "\n";
        path_found.push_back(*(current_node->parent));
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
        //std::cout << "Current Parent Address is \t " << current_node->parent << "\n" << "\n";
        //std::cout << current_node->x << "\t" << counter << "\n";
        //if(counter > 10)    break;
        //if(current_node->x == end_node->x && current_node->y == end_node->y) break;
        
    }
    std::cout << path_found.size() << "\n";
    reverse(path_found.begin(), path_found.end());
    // TODO: Implement your solution here.
    
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
    
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    RouteModel::Node *last_node = nullptr;
    RouteModel::Node temp;
    int counter = 0;
    // TODO: Implement your solution here.
    open_list.clear();
    open_list.push_back(start_node);
    std::cout << "Entering while loop \n open list size: " << open_list.size() << "\n";
    while(open_list.size() > 0){
        current_node = NextNode();
        //current_node->parent = last_node;
        //std::cout << "Current Location: " << current_node->x << "," << current_node->y << "\n";
        if(current_node->x == end_node->x && current_node->y == end_node->y){
            m_Model.path = ConstructFinalPath(current_node);
            std:: cout << "Path Found! \n";
            return;
        }
        AddNeighbors(current_node);
        last_node = current_node;
        if(current_node->parent){
            //std::cout << "stage 5 \n" /*<< "parent node position is " << current_node->parent->x
            //<< ", " << current_node->parent->y << "\n"*/;
        }
        counter ++;
        m_counter = counter;
    }
    
    return;
}
bool RoutePlanner::compare(RouteModel::Node* n1, RouteModel::Node* n2)
{
    float f1 = n1->h_value + n1->g_value;
    float f2 = n2->h_value + n2->g_value;
    return f1 >= f2;
}
