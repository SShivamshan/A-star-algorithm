#include <iostream>
#include <vector>
#include <cassert>
#include <string>
#include <ctype.h>
// #include "Map_creator.hpp"
#include "network.hpp"
int main(int argc, char *argv[]){
    std::string length,height;
    std::string obstacle,obstacle_type;
    int path_algo;
    
    if (argc == 1 || argv[1] == NULL){  // default values 
        length = "25";
        height = "25";
        obstacle = "yes";
        obstacle_type = "maze";
    }else{ 
        assert(argv[1] != 0 && isdigit(argv[1][0]) && "Invalid argument for grid length");
        assert(argv[2] != 0 && isdigit(argv[2][0]) && "Invalid argument for grid height");
        assert(argv[3] != 0 && !isdigit(argv[3][0]) && "Invalid argument for the obstacle argument");
        length = argv[1]; 
        height = argv[2];
        obstacle =  argv[3];
        
        if(argv[4] == NULL){
            obstacle_type = "maze";
        }else{
            obstacle_type = argv[4];
        }
        
    }
    
    // Map_creator map(std::atoi(length.c_str()),std::atoi(height.c_str()),obstacle);
    // std::cout << map << std::endl;

    Network map(std::atoi(length.c_str()),std::atoi(height.c_str()),obstacle,obstacle_type);
    
    std::cout << map << std::endl;
    int x,y;
    std::cout << "Insert the start point(line,column) : " << std::endl;
    std::cin >> x >> y;

    assert(x<std::atoi(length.c_str()) && x >=0 && "x value for start position incorrect");
    assert(y<std::atoi(height.c_str()) && y >=0 && "y value for start position incorrect");
    // S :start position 
    // E : end position
    
    map.set_start(x,y); // set the start position 

    std::cout << "Insert the end point(line,column) : " << std::endl;
    std::cin >> x >> y;

    assert(x<std::atoi(length.c_str()) && x >=0 && "x value for end position incorrect");
    assert(y<std::atoi(height.c_str()) && y >=0 && "y value for end position incorrect");

    map.set_end(x,y);
    std::cout << "Start position(S) and end position(E) :" << std::endl;
    std::cout << map << std::endl;

    // Start node and end node 
    Node start = map.get_start();
    Node end = map.get_end();

    // set the path planning to be used
    std::cout << "Choose the path planning algorithm : 1. A*  2. Dijkstra " << std::endl;
    std::cout << "Enter the number : ";
    std::cin >>path_algo;
    std::cout << std::endl;
    map.set_pathtype(path_algo);

    // compute and display the path taken 
    map.compute_display(&start,&end);
    // std::cout << map << std::endl;
    return 0;

}