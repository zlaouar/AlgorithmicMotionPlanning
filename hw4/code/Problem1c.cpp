#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include "graph.h"
#include "astar.h"

using namespace std;

#define PI 3.14159265

// graph implementation
int main()
{
    //------------------------- Part (c) ----------------------------
    // graph edges array.
    vector<graphEdge> edges = {
        // (x, y, w) -> edge from x to y with weight w
        //{0,1,2},{0,2,4},{1,4,3},{2,3,2},{3,1,4},{4,3,3}
        {0,1,1,0},{0,2,1,0},{0,3,1,0},{1,4,2,0},{1,5,2,0},{1,6,4,0},
        {2,7,5,0},{2,8,2,0},{2,9,3,0},{3,10,2,0},{3,11,2,0},{3,12,2,0},
        {5,13,5,0},{7,13,8,0},{9,13,6,0},{11,13,4,0}
    };
    int N = 14; // Number of vertices in the graph
    // calculate number of edges
    int n = edges.size();
    // construct graph
    
    Graph diagraph;
    diagraph.createGraph(edges, n, N);
    
    Astar astar(diagraph.head, diagraph.nodes, 0, 13);
    
    vector<graphNode> path = astar.astar_search();
    
    cout << "Path: ";
    for(int i=0; i<path.size(); i++){
        cout << path[i].vertex << ", ";
    }
    
    cout << endl << "Path Length: " <<astar.path_length <<endl;
    cout << "Num Iterations: " << astar.num_iterations << endl;
    return 0;
}