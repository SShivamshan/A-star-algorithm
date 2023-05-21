#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <exception>
#include "network.hpp"

#define INF 1000000

/**
 * @brief Constructor for the network class(child class) which inherits from the Map_creator class(parent class).
 * In this constructor we define all the nodes as well as it's neighbors 
 * 
 * @param length
 * @param height
 * @param obstacle
 * @param obstacle_type
 */
Network::Network(int length,int height,std::string obstacle,std::string obstacle_type): Map_creator(length,height,obstacle,obstacle_type){
    std::vector<std::vector<std::string>> grid = this->get_grid_map();
    std::cout << "################# Network contruction ##################"  << std::endl;
    
    // for each point(node) in the grid map we create a node
    for(int i=0;i<length;i++){ 
        for(int j=0;j<height;j++){ 

            node[j*length + i].visited = false; // all the nodes are set to be unvisited at first 

            // declare the nodes which contains 1 as the obstacles 
            if(std::atoi(grid[i][j].c_str()) == 0){
                node[j*length + i].obstacle = false;
            }else if(std::atoi(grid[i][j].c_str()) == 1){
                node[j*length + i].obstacle = true;
            }
            // columns
            node[j*length + i].x = i;
            // lines
            node[j*length + i].y = j;
            // parent nodes of each nodes 
            node[j*length + i].parent = nullptr;
            // cost values f,g,h
            node[j*length + i].g = 0; node[j*length + i].h = 0;
            node[j*length + i].f = 0 ;

        }
    }
    
    // neighbors of each nodes
    for(int i=0;i<length;i++){ // x
        // the node vector will contain length*height nodes thus we need to have
        for(int j=0;j<height;j++){ //y
            if(i>0){ // WEST
                node[j*length+i].neighbors.push_back(&node[j*length+(i-1)]);
            }
            if(i<length-1){ // EAST
                node[j*length+i].neighbors.push_back(&node[j*length+(i+1)]);
            }

            if(j>0){ // NORTH
                node[j*length+i].neighbors.push_back(&node[(j-1)*length+(i)]);
            }
            if(j<height-1){ // SOUTH
                node[j*length+i].neighbors.push_back(&node[(j+1)*length+(i)]);
            }

            if(i>0 and j>0){ // NORTH-WEST
                node[j*length+i].neighbors.push_back(&node[(j-1)*length+(i-1)]);
            }
            if(i>0 and i<length-1){ // NORTH-EAST
                node[j*length+i].neighbors.push_back(&node[(j+1)*length+(i-1)]); // i-1 j+1
            }

            if(j>0 and i<length-1){ // SOUTH-WEST
                node[j*length+i].neighbors.push_back(&node[(j-1)*length+(i+1)]); // j-1 i+1
            }
            
            if(i<length-1 and j<height-1){ // SOUTH-EAST
                node[j*length+i].neighbors.push_back(&node[(j+1)*length+(i+1)]);
            }

        }
    }       
    // for (Node *n:node[0*length+1].neighbors){
        // std::cout << n->x << " " << n->y << " " << n->obstacle << std::endl;
    // }
    
}

/**
 * @brief Destructor for the Network class 
 * 
 */
Network::~Network(){
    delete [] node;
    std::cout << "Destruction of the network" << std::endl;
}

/**
 * @brief To set the start position node within the grid map
 * 
 * @param x 
 * @param y 
 * @return ** void 
 */
void Network::set_start(int x,int y){
    _start = &node[y*this->get_length() + x];
    // change the start position from 0 -> 8 for visualization
    operator()(x,y) = "S"; // to modify  
}
/**
 * @brief To set the end position node within the grid map
 * 
 * @param x 
 * @param y 
 * @return ** void 
 */
void Network::set_end(int x,int y){
    _end = &node[y*this->get_length() + x];
    // change the start position from 0 -> 8 for visualization
    operator()(x,y) = "E"; // to modify  
}

void Network::set_cost(){
    int length = this->get_length();
    for(int i=0;i<length;++i){ 
        for(int j=0;j<this->get_height();++j){
            
            if((&node[j*length + i] != _start)){ // set all the f cost to a max value except for the start node 
                
                node[j*length+i].f = INF;
                node[j*length + i].g = INF;
                node[j*length+i].h = heuristic(node[j*length + i].x, node[j*length+i].y); // will be used for Greedy BFS all the nodes heuristic values are set
                
            }else{
                node[j*length + i].f = heuristic(node[j*length + i].x, node[j*length + i].y); // will be used for the A_star directly 
                node[j*length + i].g = g_cost(_start->x,_start->y); // will be used for Dijksrta f(n) = g(n), start cost set to zero
                node[j*length + i].h = heuristic(node[j*length + i].x, node[j*length+i].y); // will be used for Greedy BFS
            }
            // std::cout << node[j*length + i].g << " ";
        }
        // std::cout << std::endl;
    }
}
/**
 * @brief Return the start node 
 * 
 * @return ** Node& 
 */
Node& Network::get_start(){
    return *_start;
}

/**
 * @brief Return the end node 
 * 
 * @return ** Node& 
 */
Node& Network::get_end(){
    return *_end;
}
// Node& Node::operator=(const Node& that){
//     if(this != &that){
//         this->f = that.f; this->g = that.g; this->h = that.h;
//         this->obstacle= that.obstacle; this->visited = that.visited;
//         this->x = that.x; this->y = that.y;
//         this->parent=that.parent;
        
//         this->neighbors.clear();
//         for(unsigned int i=0; i<that.neighbors.size(); i++){
//             // delete this->neighbors[i];
//             this->neighbors.push_back(that.neighbors[i]);
//         }
//     }
//     return *this;
// }

/**
 * @brief To set the path planning algorithm to use
 * 
 * @param path 1: A*, 2: Dijkstra, 3: Greedy best-first search 
 * @return ** void 
 */
void Network::set_pathtype(int path){
    this->path_algo = path;
}

// heuristic values : Diagonal distance / Chebyshev distance
// cost of moving diagonally is not equivalent to moving horizontally or vertically
// or cost of moving diagonally is equivalent to moving horizantally or vertically
// change the values if necessary to view the difference especially for the GBFS
/**
 * @brief Calculate the heurstic distance between the goal node and the node n. If the diagonal cost is lesser 
 * than moving straight cost, the algorithm will tend to move more diagonally and if moving straight cost is lesser than 
 * moving diagonally cost then the algorithm will move horizontally or vertically than diagonally. 
 * 
 * @param a 
 * @param b 
 * @return ** int 
 */
int Network::heuristic(int a,int b){
    int h_diagonal = std::min(std::abs(a - _end->x),std::abs(b - _end->y));
    int h_straight = std::abs(a - _end->x) + std::abs(b - _end->y);
    double diag_cost = 1; // moving diagonally 
    double cost = 1; // moving straight cost 
    return (int)cost * (h_diagonal+h_straight) + std::min(h_diagonal,h_straight) * (diag_cost - 2*cost);
}
/**
 * @brief Calculate the distance from start node to current node
 * 
 * @param a 
 * @param b 
 * @return ** int 
 */
int Network::g_cost(int a,int b){
    // double dx = std::abs(_start->x - a) ;
    // double dy = std::abs(_start->y - b);
// 
    // if(dx > dy){
        // return 2*dy + 1*(dx - dy);
    // }
    // return 2*dx + 1*(dy - dx);
    double dx = std::abs(_start->x - a);
    double dy = std::abs(_start->y - b);

    return (int)std::sqrt(dx + dy);
}

/**
 * @brief  Calculate distance between current node and neighbor node
 * 
 * @param current 
 * @param neighbor 
 * @return ** int 
 */
int Network::d(Node *current,Node *neighbor){
    double dx = std::abs(current->x - neighbor->x);
    double dy = std::abs(current->y - neighbor->y);

    if(dx > dy){
        return 2*dy + 1*(dx - dy);
    }
    return 2*dx + 1*(dy - dx);
}
/**
 * @brief 
 * 
 * @return ** std::vector<std::pair<int,int>> 
 */
std::vector<std::pair<int,int>> Network::reverse_path(){
    std::vector<std::pair<int,int>> path;
    std::vector<Node *>pnodes;
    // to retrieve the path from the start node to end node
    // to do so we loop backward starting from the end node to the start nod
    Node *end = (this->_end);
    // we know that all the child node will be assigned a parent node once we start searching for the end node 
    // except for the start node parent will stay as nullptr since it's were we started(the first parent node)
    while(end->parent != nullptr){
        end = end->parent;
        pnodes.emplace_back(end);
    }

    // we reverse the vector containing the nodes of the path
    std::reverse(pnodes.begin(),pnodes.end());
    // we retrieve the coordiantes of the path
    for(auto nodes:pnodes){
        path.emplace_back(std::make_pair(nodes->x,nodes->y));  
    }
    // std::cout << pnodes.size() << std::endl
    return path;
}

/**
 * @brief To verify is a node equal to another node using their coordinates(x,y)
 * 
 * @param n 
 * @return true 
 * @return false 
 */
bool Node::isequal(Node *n){
    return x == n->x && y == n->y;
}

/************************************ Path Planning ****************************************************/
/**
 * @brief A* : calculate the shortest path using the heuristic cost and g_cost f(n) = g(n) + h(n)
 * set the values of the moving diagonal and straight cost to visualize the effects of path taken.
 * 
 * @param _start 
 * @param _end 
 * @return ** std::vector<std::pair<int,int>> 
 */
std::vector<std::pair<int,int>> Network::A_star(Node *_start,Node *_end){
    std::vector<std::pair<int,int>> path;
    this->set_cost(); // set all f cost values to inf except for the start node 
    Node *current; 
   
    openlist.push_back(_start);
    while(!openlist.empty()){
        current = *openlist.begin(); 
        // int actual_dist = current->f;
        
        for(unsigned int j=0;j<openlist.size();j++){
            if(openlist[j]->f < current->f && openlist[j]->visited == false){
                current = openlist[j];
                // actual_dist = openlist[j]->f;
            }
        }
        
        // if the current node == end node 
        if (current->isequal(this->_end)){
            current->visited = true;
            // std::cout << "End found " << std::endl;
            path = this->reverse_path();
            // break;
            return path;
        }else{
            
            for(auto neighbor : current->neighbors){
            
                if(neighbor->obstacle == true || neighbor->visited == true){
                    continue;
                }
                int temp_gscore = current->g + d(current,neighbor); // calculate the distance from start to neighbor going through the current node 
                if (temp_gscore < neighbor->g){
                    neighbor->g = temp_gscore;
                    neighbor->f = temp_gscore + heuristic(neighbor->x,neighbor->y);
                    neighbor->parent = current;
                    // operator()(neighbor->x,neighbor->y) = "2";
                    // std::cout << neighbor->parent->x << " " << neighbor->parent->y <<std::endl;
                }
                if(std::find(openlist.begin(),openlist.end(),neighbor) != openlist.end()){ // if the neigbor is present in the openlist 
                    continue;
                }
                openlist.push_back(neighbor);
            }
            
        }

        // for the current chosen node, we change the bool visited value to true
        for(std::vector<Node *>::iterator it = openlist.begin(); it != openlist.end();)
        {
            if(*it == current){
                current->visited = true;
                openlist.erase(it);
            }else{
                ++it;
            }
        }
        // std::cout << current->x << " " << current->y <<" "<< current->visited <<std::endl;

    }
    openlist.clear();
    return {};
}

/**
 * @brief Dijkstra: Calculate the shortest path from the start node to end node using the g_cost function f(n) = g(n)
 * @param _start 
 * @param _end 
 * @return ** std::vector<std::pair<int,int>> 
*/
std::vector<std::pair<int,int>> Network::Dijkstra(Node *_start,Node *_end){
    std::vector<std::pair<int,int>> path;
    this->set_cost(); // set all g cost values to inf except for the start node
    Node *current; 
    // int size = this->get_length() * this->get_height();
    // for(int i =0;i< size;i++){
        // openlist.push_back(std::make_shared<Node>(node[i]));
        // openlist.push_back(&node[i]);
    // }
    openlist.push_back(_start);
    while(!openlist.empty()){
        current = *openlist.begin(); 
        // int actual_dist = current->g;
        // int current_index = 0;
        for(unsigned int j=0;j<openlist.size();j++){
            if(openlist[j]->g < current->g && openlist[j]->visited == false){
                current = openlist[j];
                // current_index = (int)j;
                // actual_dist = openlist[j]->g;
            }else{
                continue;
            }
        }

        if(current->obstacle == true){
            continue;
        }
    
        if (current->isequal(this->_end)){
            current->visited = true;
            
            path = this->reverse_path();
            std::cout << "End found " << std::endl;
            // break;
            return path;
        }else{
        
            for(Node *neighbor : current->neighbors){
            
                if(neighbor->obstacle == true || neighbor->visited == true){
                    continue;
                }
                int temp_gscore = current->g + g_cost(neighbor->x,neighbor->y); // calculate the distance from start to neighbor node n
                if (temp_gscore < neighbor->g && current->g != INF ){
                    neighbor->g = temp_gscore;
                    neighbor->f = neighbor->g; // f(n) = g(n)
                    neighbor->parent = current;
                    // operator()(neighbor->x,neighbor->y) = "2";
                    openlist.push_back(neighbor);
                }
                
            }
        }
       
        for(auto it = openlist.begin(); it != openlist.end();)
        {
            if(*it == current){
                current->visited = true;
                openlist.erase(it);
                // current = nullptr;
            }else{
                ++it;
            }
        }
        // openlist.erase(openlist.begin() + current_index);
    }
    openlist.clear();

    return {}; // no path found 

}

/*
Instead of using a priority queue to store the nodes in order of their heuristic values 
We use a simple vector which gets sorted in order of their heuristic values so that we get the smallest heurstics(distance from node n to goal)
*/
/**
 * @brief Calculate the path using the Greedy best-first search algorithm using the heuristic h(n),
 * f(n) = h(n). This algorithm doesn't always gives out the optimal path since it always for the shortest path from 
 * a node n to goal node. 
 * 
 * @param _start 
 * @param _end 
 * @return ** std::vector<std::pair<int,int>> 
 */
std::vector<std::pair<int,int>> Network::GBFS(Node *_start, Node *_end){
    std::vector<std::pair<int,int>> path;
    
    this->set_cost(); 
    Node *current; 

    openlist.push_back(_start);
    this->_start->visited = true;
    while(!openlist.empty()){
        
        std::sort(openlist.begin(), openlist.end(),[](const Node *a,const Node *b){ return (a->h < b->h);}); // will sort by the value of first element of the pair thus this gives us the shortest heuristic value each time
                                                                                                             // use of a lambda function to sort the nodes through the heuristic values 
        current = *openlist.begin();
        if (current->isequal(this->_end)){
            current->visited = true;
            // std::cout << "End found " << std::endl;
            path = this->reverse_path();
            // break;
            return path;
        }else{
            for(auto neighbors:current->neighbors){
                if(neighbors->obstacle == true || neighbors->visited == true){
                    continue;
                }

               if(std::find(openlist.begin(),openlist.end(),neighbors)== openlist.end()){
                    openlist.push_back(neighbors);
                    neighbors->parent = current; // to get the parent node of each child nodes from which we visited it
                }

            }

        }
        // for the current chosen node, we change the bool visited value to true
        for(std::vector<Node *>::iterator it = openlist.begin(); it != openlist.end();)
        {
            if(current->isequal(*it)){
                current->visited = true;
                openlist.erase(it);
                // current = nullptr;
            }else{
                ++it;
            }
        }
        
    }
    openlist.clear();
    return {};
}
/*Possibility of improvement : create a vector of function or tuple of functions */
/**
 * @brief Compute and compare two path planning algorithms 
 * 
 * @param start 
 * @param end 
 * @param a 
 * @param b 
 * @return ** void 
 */
void Network::compute_compare(Node *start,Node *end,int a,int b){
    std::vector<std::pair<int,int>> path;
    std::unordered_map<int,std::string> algos = {{1,"A*"},{2,"Dijkstra"},{3,"Greedy BFS"}}; 
    std::cout << "Comparing two path planning algorithms : " << algos[a] << " and " << algos[b] << std::endl;
    std::vector<int> vec = {a,b};
    for(auto v:vec){
        if(v==1){
            path = this->A_star(start,end);

        }else if(v==2){
            path = this->Dijkstra(start,end);
            
        }else{
            path = this->GBFS(start,end);
        }
        int i = 1;
        path.erase(path.begin());
        std::vector<std::vector<std::string>> grid = this->get_grid_map();
        for(auto p:path){
            // this->operator()(p.first,p.second) = std::to_string(i);
            grid[p.first][p.second] = std::to_string(i);
            i++;
        }
        std::cout << "Path found from S(" << this->_start->x << "," << this->_start->y << ")"<< "to E("<< this->_end->x << "," << this->_end->y << ")  using "<< algos[v]  << " : " << std::endl;
        for(int i=0;i<this->get_length();i++){
            std::cout << " +";
            for(int j=0;j<this->get_height();j++){
                std::cout << " " << grid[i][j];

            }
            std::cout << " +" << std::endl;
        }
        i=0;
        path = {};
    }
    
}

/**
 * @brief Calculate the process time for the process of the path planning as well as to 
 * display the path taken from going to start node to end node (S -> E)
 * @param _start 
 * @param _end 
 * @return ** void 
*/
void Network::compute_display(Node *_start,Node *_end){
    std::vector<std::pair<int,int>>path;
    std::unordered_map<int,std::string> algos = {{1,"A*"},{2,"Dijkstra"},{3,"Greedy BFS"}};
    
    if(this->path_algo == 1){
        auto start = std::chrono::high_resolution_clock::now();
        path =  this->A_star(_start,_end);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Time taken for path planning : " << duration.count() << " microseonds" << std::endl;
    }else if(this->path_algo == 2){
        auto start = std::chrono::high_resolution_clock::now();
        path =  this->Dijkstra(_start,_end);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Time taken for path planning : " << duration.count() << " microseonds" << std::endl;
    }else if(this->path_algo == 3){
        auto start = std::chrono::high_resolution_clock::now();
        try{
            path =  this->GBFS(_start,_end);
        }catch(const std::exception &e){
            std::cerr << "Exception : " << e.what() << std::endl;
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Time taken for path planning : " << duration.count() << " microseonds" << std::endl;
    }
    
    if (path.empty() == true){
        std::cout << "Path from S(" << this->_start->x << "," << this->_start->y << ")"<< "to E("<< this->_end->x << "," << this->_end->y << ")  using "<< algos[this->path_algo] << " : " << std::endl;
        std::cout << "End node couldn't be found!. Perhaps the node start node is blocked from the end node or the end node is in a obstacle " << std::endl;
    }
    if(path.empty() == false){
        int i = 1;
        path.erase(path.begin());
        std::vector<std::vector<std::string>> grid = this->get_grid_map();
        for(auto p:path){
            // this->operator()(p.first,p.second) = std::to_string(i);
            grid[p.first][p.second] = std::to_string(i);
            i++;
        }
        std::cout << "Path found from S(" << this->_start->x << "," << this->_start->y << ")"<< "to E("<< this->_end->x << "," << this->_end->y << ")  using "<< algos[this->path_algo]  << " : " << std::endl;
        for(int i=0;i<this->get_length();i++){
            std::cout << " +";
            for(int j=0;j<this->get_height();j++){
                std::cout << " " << grid[i][j];
                
            }
            std::cout << " +" << std::endl;
        }
    }
    
}


