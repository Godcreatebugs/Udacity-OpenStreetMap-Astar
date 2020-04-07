#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Done 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
      start_node = &m_Model.FindClosestNode(start_x,start_y);
      end_node = &m_Model.FindClosestNode(end_x,end_y); 
      std::cout<<"m_model start_x and start_y"<<start_node->x<<start_node->y;
      std::cout<<"m_model end_x and end_y"<<end_node->x<<end_node->y;
}


// Done 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Done 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    //neighbors found and added to radar
    current_node->FindNeighbors();
    // using for loop deciding parent,h and g values 
    //range based for loop
    for(auto *neighbour :current_node->neighbors ){
        //parent
        neighbour->parent = current_node;
        //g value is basically the moves you made which is previous g value and distance till the neighbour.
        //its not a grid so you will just add 1
        neighbour->g_value = current_node->g_value + current_node->distance(*neighbour);
        //h-value is using H-Value function           
        neighbour->h_value = CalculateHValue(neighbour);
        //add neighbours to open list cuz we want them to explore
        open_list.emplace_back(neighbour);
        //mark the neighbour closed
        neighbour->visited = pclose;
    }
}


// Done 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

//Getting f-value function

// float GetFValue(RouteModel::Node *node){
//     return node->g_value + node->h_value;   
// }

RouteModel::Node *RoutePlanner::NextNode() {
      //Ok, Now we can use the sort function in standard library. The use is new For me but the syntax is what is it

      //Sort the data
      std::sort(open_list.begin(), open_list.end(), [](const auto & _1iter, const auto & _2iter){
          return _1iter->g_value +_1iter->h_value < _2iter->g_value + _2iter->h_value; 
          });  
      //get the front out and again this open list is not a vector of nodes but pointer to the nodes
      //Used open_list->front but it only works when open_list is pointer but it is vector of pointers.
      //so we have to access function using the -> operator
      RouteModel::Node *lowest_sum =  open_list.front();   
      //remove that node 
      open_list.erase(open_list.begin());
      //return the lowest_sum
      return lowest_sum;
      
}


// Done 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    //current_node = end_node;
    // TODO: Implement your solution here.
    //using while loop to backtrack
    //we have started with null pointer and we have to go back to null pointer
    while(current_node->parent != start_node){
        //pushing back current node in path_found vector (of nodes)
        path_found.emplace_back(*current_node);
        //add distance variable
        //float total_distance=0;
        distance += current_node->distance (*(current_node->parent));
        std::cout<<distance<<"\n";
        //going back means that current node set to its parent
        current_node = current_node->parent;
    
    }
    // now consider a stack where pointer moves upwards and if we just rotate that stack by 180 degres we have our backtacked path
    // I guess this is the most important line in the function
   // path_found.push_back(*current_node);rou   
    //using reverse function
    std::reverse(path_found.begin(),path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    //now reversing current node
    //this method needs an iterator which we dont have cuz current_node is just a vector and not a list
    //std::reverse()
    std::cout<<"Path has been found here."<<"\n";
    return path_found;

}


// Done 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    //start_node is closed and start happening with that
    start_node->visited =true;
    //open_list start with start_node
    open_list.push_back(start_node);
    //current_node pointing to start_node
    //current_node->start_node;
    //use of while loop untill the current_node makes a chain till end
    while(open_list.size()>0){
          //next node method but a pointer used to point to that node
          current_node = NextNode();
          
          //if current_node is an end then return the path
          if(current_node->distance(*end_node)==0){
                m_Model.path=ConstructFinalPath(current_node);
              
          std::cout<<"Path found and program ends";
          return;
    
    }
    AddNeighbors(current_node);
    }
    return;
    //this is for populating open_list
    
    

}