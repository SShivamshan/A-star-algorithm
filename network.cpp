#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <unordered_map>
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
Network::~Network(){
    delete [] node;
    std::cout << "Destruction of the network" << std::endl;
}

void Network::set_start(int x,int y){
    _start = &node[y*this->get_length() + x];
    // change the start position from 0 -> 8 for visualization
    operator()(x,y) = "S"; // to modify  
}

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
                
            }else{
                node[j*length + i].f = heuristic(node[j*length + i].x, node[j*length + i].y);
                node[j*length + i].g = 0;
            }
            // std::cout << node[j*length + i].f << " ";
        }
        // std::cout << std::endl;
    }
}

Node& Network::get_start(){
    return *_start;
}

Node& Network::get_end(){
    return *_end;
}

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
// cost of moving diagonally is not equivalent to moving horizonatlly or verically
int Network::heuristic(int a,int b){
    int h_diagonal = std::min(std::abs(a - _end->x),std::abs(b - _end->y));
    int h_straight = std::abs(a - _end->x) + std::abs(b - _end->y);
    double diag_cost = 0.5; // moving diagonally 
    double cost = 1; // moving straight cost 
    return (int)diag_cost * h_diagonal + cost * (h_straight - 2*h_diagonal);
}
// calculate the distance from start node to current node(a,b) 
int Network::g_cost(int a,int b){
    double dx = std::abs(_start->x - a) ;
    double dy = std::abs(_start->y - b);

    if(dx > dy){
        return 2*dy + 1*(dx - dy);
    }
    return 2*dx + 1*(dy - dx);
}

// calculate distance between current node and neigbor node
int Network::d(Node *current,Node *neighbor){
    double dx = std::abs(current->x - neighbor->x);
    double dy = std::abs(current->y - neighbor->y);

    if(dx > dy){
        return 2*dy + 1*(dx - dy);
    }
    return 2*dx + 1*(dy - dx);
}

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


std::vector<std::pair<int,int>> Network::A_star(Node *_start,Node *_end){
    std::vector<std::pair<int,int>> path;
    this->set_cost(); // set all f cost values to inf except for the start node 
    Node *current; 
    int size = this->get_length() * this->get_height();
    // all nodes are considered open at first
    for(int i =0;i< size;i++){
        openlist.push_back(&node[i]);
    }

    while(!openlist.empty()){
        current = *openlist.begin(); 
        int actual_dist = current->f;
        
        for(int j=0;j<size;j++){
            if(openlist[j]->f < actual_dist && openlist[j]->visited == false){
                current = openlist[j];
                actual_dist = openlist[j]->f;
            }else{
                continue;
            }
        }
        
        // if the current node == end node 
        if (current ==  this->_end){
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
                    if(std::find(openlist.begin(),openlist.end(),neighbor)== openlist.end()){
                        openlist.push_back(neighbor);
                    }
                }
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
    return {};
}

/**
 * @brief Calculate the shortest path from the start node to end node using the cost of 
 * moving from one node to another node given by the g_cost
 * @param _start 
 * @param _end 
 * @return ** std::vector<std::pair<int,int>> 
*/
std::vector<std::pair<int,int>> Network::Dijkstra(Node *_start,Node *_end){
    std::vector<std::pair<int,int>> path;
    this->set_cost(); // set all f cost values to inf except for the start node 
    Node *current; 
    int size = this->get_length() * this->get_height();
    // all nodes are considered open at first
    for(int i =0;i< size;i++){
        openlist.push_back(&node[i]);
    }

    while(!openlist.empty()){
        current = *openlist.begin(); 
        int actual_dist = current->g;

        for(int j=0;j<size;j++){
            if(openlist[j]->g < actual_dist && openlist[j]->visited == false){
                current = openlist[j];
                actual_dist = openlist[j]->g;
            }else{
                continue;
            }
        }
        // if the current node == end node 
        if (current ==  this->_end){
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
                int temp_gscore = current->g + g_cost(neighbor->x,neighbor->y); // calculate the distance from start to neighbor node n
                if (temp_gscore < neighbor->g && current->g != INF ){
                    neighbor->g = temp_gscore;
                    neighbor->parent = current;
                    // operator()(neighbor->x,neighbor->y) = "2";
                    
                }
        
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
    return {};

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
    std::unordered_map<int,std::string> algos = {{1,"A*"},{2,"Dijkstra"}};

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
    }
    
    if (path.empty() == true){
        std::cout << "Path from S(" << this->_start->x << "," << this->_start->y << ")"<< "to E("<< this->_end->x << "," << this->_end->y << ")  using "<< algos[this->path_algo] << " : " << std::endl;
        std::cout << "End node couldn't be found!. Perhaps the node start node is blocked from the end node or the end node is in a obstacle " << std::endl;
    }
    if(path.empty() == false){
        int i = 1;
        path.erase(path.begin());
        for(auto p:path){
            this->operator()(p.first,p.second) = std::to_string(i);
            i++;
        }
        std::cout << "Path found from S(" << this->_start->x << "," << this->_start->y << ")"<< "to E("<< this->_end->x << "," << this->_end->y << ")  using "<< algos[this->path_algo]  << " : " << std::endl;
        std::vector<std::vector<std::string>> grid = this->get_grid_map();
        for(int i=0;i<this->get_length();i++){
            std::cout << " +";
            for(int j=0;j<this->get_height();j++){
                std::cout << " " << grid[i][j];
            }
            std::cout << " +" << std::endl;
        }
    }
    
}


