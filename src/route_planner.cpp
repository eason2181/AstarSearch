#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    
    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
    start_node->parent = nullptr;
    start_node->visited = true;
    std::cout << "starting at: "<< start_node->x << ", " << start_node->y
    << "\n ending at: " << end_node->x << ", " << end_node->y << "\n";
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h;
    h = node->distance(*end_node);
    return h;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value.
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

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


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node* res;
    sort(open_list.begin(), open_list.end(), this->compare);
    res = open_list.back();
    open_list.pop_back();
    return res;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

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


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

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
