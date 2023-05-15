#ifndef MAP_CREATOR_H
#define MAP_CREATOR_H

#include <vector>
#include <iostream>
#include <string>

class Map_creator{
    private:
    int _length;
    int _height;
    std::string _obstacle;
    std::vector<std::vector<std::string> > _grid_map;

    public:
    Map_creator(int length,int height,std::string obstacle,std::string obstacle_type);
    ~Map_creator();

    std::vector<std::vector<std::string>> set_obstacles(std::vector<std::vector<std::string> >& grid,std::string obstacle,std::string obstacle_type); // to set the obstacles depending if the user wants it or not
    const std::vector<std::vector<std::string>>& get_grid_map() const;                                                                                // return the grid map to be used
    int get_length() const;                             // to retrieve the length of the grid map
    int get_height() const;                             // to retrieve the height of the grid map 
    
    std::string& operator()(int x,int y);               // to insert an element into the grid map
    std::string operator()(int x,int y)const;           // to retrieve an element from the grid map

    friend std::ostream & operator<<(std::ostream& os,Map_creator &grid);
};


#endif 