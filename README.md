# A-star-algorithm 

Implementation of A* algorithm on a 2D grid map. The map in question can create two forms of maps: 
* Maze like map   
  ![](https://github.com/SShivamshan/A-star-algorithm/blob/main/test/Screenshot%20from%202023-05-21%2021-15-14.png "Maze grid")  
* A map containing random obstacles   
  ![](https://github.com/SShivamshan/A-star-algorithm/blob/main/test/Screenshot%20from%202023-05-21%2021-28-26.png "Random obstacle grid")  
The grid maps consists of 0 and 1 with ones defining an obstacle and zeros the lane. The start and end position are shown by the S and E.  

## Path planning algorithms
The size of the map can de defined by user through the command line as well. 
There are two more path planning algorithms : A*, Dijsktra and Greedy Best-First Search. All 3 path planning works quite same way algorithmically, the difference is due to the fact that A* uses both a heuristic cost which is the mouvement cost from node n to goal node(how far are we from the end node from the current node) and the exact cost of moving from the start node to node n compared to Dijkstra which only uses the cost of moving from the start node to node n and finally the Greedy BFS which uses only the heuristic funtion thus the how far are we from the end node which doesn't always lead to the optimal path. 

The most important factor about A* is the heuristic cost : it's 0 if f(n) = g(n) we would only depend on the exact cost and if h(n) is greater than g(n) we will only depend on h(n) not both. This is the most important function to impelement in our case of a grid map,we will use the diagonal distance. 

Diagonal distance :         
The diagonal distance is given by : cost * (dx+dy) + min(dx,dy) * (diag_cost - 2*cost), cost corresponds to moving straight and diag_cost for moving diagonally. These coefficients can be modified. If these coeffcient are equal then they are called the : Chebyshev distance.
There are two other functions such as the g_cost which calculates the distance from the start node to a node n and the distance function which calculates the distance from the current node to the neigbor node
## Compiling and debugging 
To compile the program :`g++ -ansi -Wall -pedantic -g -std=c++11 main.cpp Map_creator.hpp Map_creator.cpp network.cpp network.hpp -o main.exe`  
To excute the program, there are two possible ways :   
* without any arguments : this will create by default a 25x25 of a maze type grid
 `./main.exe` 
 * with arguments sent through the command line : the third argument is to define if the user wants to have any obstacles and the fourth argument is to define the map type: **maze** for a maze type grid and **random** for randomly generated obstacles inside a grid map.  
 `./main.exe 15 15 yes maze` or `./main.exe 15 15 yes random`
 The follwing images were compiled and debugged through valgrind.  
 The following is the result that i got for the maze type grid using the **A***:  
 
![](https://github.com/SShivamshan/A-star-algorithm/blob/main/test/Screenshot%20from%202023-05-21%2021-14-52.png "Result A*")  
 The following is the result that i got for the randomly generated obstacles type grid using the **Greedy Best-First Search**:  
 ![](https://github.com/SShivamshan/A-star-algorithm/blob/main/test/Screenshot%20from%202023-05-21%2021-34-58.png "Result GBFS")  
 The following is the result that i got for the randomly generated obstacles type grid using the **Dijkstra**:  
 ![](https://github.com/SShivamshan/A-star-algorithm/blob/main/test/Screenshot%20from%202023-05-21%2021-36-17.png "Result Dijkstra") 

**Used valgrind to find any memory leaks. As we can see on the third image that even though that i don't have any memory leaks at the end of the program and path from the goal to end also appears, I do get some errors occasionally (invalid read) that i don't seem to understand why is it occuring at some times, which needs further invesigation(any remarks or correction is welcome).**

## Further improvements
* Use of smart pointers(shared_ptr) instead of raw pointers(with the raw pointers, memory are always freed at the end of the program)
* Improvement of the grid map especially for the maze. 
* To use different type of distance(euclidean distance) to view how the path planning works
* Implement a duration calculation of cpu time use for each path planning algorithm(currently gives 0 or 1 microseconds when used with ./main) better than the one the implemented







