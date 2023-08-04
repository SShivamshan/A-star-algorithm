#ifndef NETWORK_H
#define NETWORK_H

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include "Map_creator.hpp"

// A* : Informed method 
// f(n) = g(n) + h(n), g(n) represents the cost from the starting node to the next node n
//                     h(n) represents the distance from node n to the end node 
// The more we are closer to the end node, g(n) will go up h(n) will go down 

// Problem of A* : necessary to find a good heuristic function 
// For the heuristic h(n) in our case for a grid map, it's necessary to have a good estimation of the mimimum cost from a node n to the goal. 

// Dijkstra : Non informed method 
// For the Dijkstra which depends on the g cost thus the cost to reach a node n 

// Greedy best-first : Informed method
// Depends on a heuristic function to get the distance from node n to the goal

struct Node{
    bool visited;                   // Set to false and set to true if the node was visited 
    bool obstacle;                  // The node is an obstacle if it's 1 then it's set to true or else false
    std::vector<Node *> neighbors;
    int f,g,h;                      // the distance score 
    Node *parent;                   // parent node for each node
    int x,y;                        // poisition of the nodes in the grid map
    bool isequal(Node *n);
    // Node& operator=(const Node& that); // copy assignment operator 
};
class Network: public Map_creator{

    public:
        Network(int length,int height,std::string obstacle,std::string obstacle_type);
        virtual ~Network();                                                     // virtual so that we first destroy the network class then the map class 
        void compute_display(Node *_start,Node *_end);                          // to display the path taken and the process time
        void set_pathtype(int path);                                            // to set the path planning to use 
        void set_start(int x,int y);
        void set_end(int x,int y);
        // void set_movecost(int diag,int front);                                  // to set the coefficient of moving diagonally or horizantally which has an impact on the path taken
        Node& get_start();
        Node& get_end();
        void compute_compare(Node *start,Node *end,int a,int b);                // to compare two path planning algorithms  
        
    protected:                                                                  // only accesible for the derived class 
        Node *_start = nullptr;
        Node *_end = nullptr;
    private:
        void set_cost();
        int heuristic(int a,int b);                                             // calculate the heuristic distance using the diagonal distance 
        int g_cost(int a,int b);                                                // calculate the distance from the start node to node x 
        int distance(Node *current,Node *neighbor);                             // calculate the distance from the current node to it's neighbor node 
        std::vector<std::pair<int,int>> reverse_path();                         // to reverse the path starting from the end node to start node 

        Node *node = new Node[this->get_length()*this->get_height()];           // create the number of nodes throught the length and height variables given by the Map_creator class 
        int path_algo;                                                          // set the algorithm to be used 1. A* 2. Dijkstra 
        std::vector<Node *> openlist;
        // std::vector<std::reference_wrapper<Node>> openlist;                    
        // std::vector<std::shared_ptr<Node>> openlist;
        
        // Path planning algorithms implemented 
        std::vector<std::pair<int,int>> Dijkstra(Node *_start,Node *_end);       // Dijkstra f(n) = g(n)
        std::vector<std::pair<int,int>> A_star(Node *_start,Node *_end);         // A* f(n) = g(n) + h(n)
        std::vector<std::pair<int,int>> GBFS(Node *_start,Node *_end);           // Greedy best-first search f(n) = h(n)
};


#endif