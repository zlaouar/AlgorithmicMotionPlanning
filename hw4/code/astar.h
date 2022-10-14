#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <set>
#include "Graph.h"



// CLASS ASTAR
class Astar{
    
    public:
        struct NodeComp{
            bool operator() (const adjNode &lhs, const adjNode &rhs){
                
                double path_length_lhs = lhs.weight;
                double path_length_rhs = rhs.weight;

                double heuristic_lhs = lhs.heuristic;
                double heuristic_rhs = rhs.heuristic;

                double cost_lhs = path_length_lhs + heuristic_lhs;
                double cost_rhs = path_length_rhs + heuristic_rhs;
                
                if(cost_rhs == cost_lhs){
                    return true;
                }
                return cost_rhs>cost_lhs;
            }
        };
        adjNode **vertPointers;
        std::vector<graphNode> graphNodes;
        int start, goal, num_iterations;
        double path_length;
        int valid_path;
        std::set<adjNode, NodeComp> open, closed;

        std::vector<adjNode> open_vec, closed_vec;
        // Constructor
        Astar(adjNode **head, std::vector<graphNode> nodes,int startNode, int goalNode);
        std::vector<graphNode> astar_search();
        

        void clear(){
            graphNodes.clear();
            open.clear();
            closed.clear();
        }

};
#endif