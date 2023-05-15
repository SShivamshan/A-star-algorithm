#include <iostream>
#include "Map_creator.hpp"
#include <vector>
#include <unordered_map>
#include <cstdlib>
#include <time.h>
#include <string>

/* Possibly use the depth-first search method to create a map */
/**
 * @brief Constructor for the Map_creator which takes the length, height, if the user wants to have an obstacle
 * and the type of obstacle 
 * @param height
 * @param length
 * @param obstacle
 * @param obstacle_type
 *
 */

Map_creator::Map_creator(int length,int height,std::string obstacle,std::string obstacle_type){
    this->_length = length;
    this->_height = height;
    this->_obstacle = obstacle;
    
    std::cout << "################# Map contruction ##################" << std::endl;
    //int num = 0;
    for(int i=0; i<this->_length;i++){
        std::vector<std::string> v1;
        for (int j=0; j<this->_height;j++){
            v1.push_back("0");
            //num += 10;
        }
        this->_grid_map.push_back(v1);
        v1.clear();
    }

    // set the obstacles 
    this->_grid_map = this->set_obstacles(this->_grid_map,this->_obstacle,obstacle_type);
}

/**
 * @brief Overrride of the operator<< in order to print out the map in a certain format 
 * 
 * @param os 
 * @param grid 
 * @return ** std::ostream& 
 */
std::ostream& operator<<(std::ostream &os,Map_creator &grid){
    
    for(int i = 0;i < grid._height; i++){
        os << "+";
        for (int j = 0;j < grid._length;j++){
            os << " " << grid._grid_map[i][j];
        }
        os << " +" << std::endl;
    }
    
    return os;
}
/**
 * @brief Destructor for the Map_creator class 
 * 
 */
Map_creator::~Map_creator(){
    std::cout << "Destruction of the Map" << std::endl;
}

/**
 * @brief Set some obstacles or create a maze like architecture depending on argument given through obstacle type 
 * the obstacle argument defines if the user wants to have obstacles or not. 
 * 
 * @param grid retrieve the grid containing only zeros
 * @param obstacle defines if user wants to have obstacles or not
 * @param obstacle_type the type of obstacles : maze or randomly placed obstacles
 * @return ** std::vector<std::vector<std::string>> 
 */
std::vector<std::vector<std::string> > Map_creator::set_obstacles(std::vector<std::vector<std::string> >& grid,std::string obstacle,std::string obstacle_type){

    if (obstacle == "no"){
        return grid;
    }
    
    if (obstacle == "yes"){
        srand(time(0));
        if(obstacle_type == "maze"){
            if (this->_height <= 10){ 
                for(int i = 0;i < 5; i++){
                    int val = rand()% 10 ;

                    if ( val % 2 == 0){
                        for(int j = 0;j < val;j++){
                            grid[j][val] = "1";
                        }
                    }else{
                        for(int j = 0;j < val;j++){
                            grid[val][j] = "1";
                        } 
                    }

                }
            }else{
                for(int i = 0;i < (int)this->_height/2+6;i++){
                    int val = rand() % this->_height;
                    if ( val % 2 == 0){
                        for(int j = 0;j < val;j++){
                            grid[j][val] = "1";

                        }
                    }else{
                        for(int j = 0;j < val;j++){
                            grid[val][j] = "1";
                            grid[i][val] = "1";
                        } 
                    }
                }
            }
        }else if(obstacle_type == "random"){
            for(int i=0 ; i<this->_length*(int)this->_length/2;i++){
                int len = rand()%(int)this->_length;
                int hei = rand()%(int)this->_height;
                grid[len][hei] = "1";
            }
        }    
        
    }
    
    return grid;
}
/**
 * @brief Override of the operator() to set an element on to the grid map
 * 
 * @param x 
 * @param y 
 * @return ** std::string& 
 */
std::string& Map_creator::operator()(int x, int y){
    return this->_grid_map[x][y]; 
}
/**
 * @brief Override of the operator() to get the element of the grid map for a given x,y coordinate
 * 
 * @param x 
 * @param y 
 * @return ** std::string 
 */
std::string Map_creator::operator()(int x, int y) const{
    return this->_grid_map[x][y];
}

/**
 * @brief This function allows to get the grid map which can be used for by the child class for their purposes
 * 
 * @return ** const std::vector<std::vector<std::string>>& 
 */
const std::vector<std::vector<std::string>>& Map_creator::get_grid_map() const{
    return this->_grid_map;
}

/**
 * @brief Getter function to retrieve the length value of the grid map
 * 
 * @return ** int 
 */
int Map_creator::get_length()const{return this->_length;}
/**
 * @brief Getter function to retrieve the height value of the grid map
 * 
 * @return ** int 
 */
int Map_creator::get_height()const{return this->_height;}