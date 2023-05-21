# A-star-algorithm 

Implementation of A* algorithm on a 2D grid map. The map in question can create two forms of maps: 
* Maze like map 
* A map containing random obstacles 

The size of the map can de defined by user through the command line as well. 
There are two more path planning algorithms : A*, Dijsktra and Greedy Best-First Search. All 3 path planning works quite same way algorithmically, the difference is due to the fact that A* uses both a heuristic cost which is the mouvement cost from node n to goal node(how far are we from the end node) and the exact cost of moving from the start node to node n compared to Dijkstra which only uses the cost of moving from the start node to node n. 
The most important factor about A* is the heuristic cost : it's 0 : f(n) = g(n) we would only depend on the exact cost and if h(n) is greater than g(n) we will only depend on h(n) not both. This is the most important function to impelement in our case of a grid map,we will use the diagonal distance. 

Diagonal distance :         
The diagonal distance is given by : cost * (dx+dy) + min(dx,dy) * (diag_cost - 2*cost), cost corresponds to moving straight and diag_cost for moving diagonally. These coefficients can be modified. If these coeffcient are equal then they are called the : Chebyshev distance.

Debugged the code using valgrind to see any memory leaks.

To compile the program : 'g++ -ansi -Wall -pedantic -g -std=c++11 main.cpp Map_creator.hpp Map_creator.cpp network.cpp network.hpp -o main.exe'




