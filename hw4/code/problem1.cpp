#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include "graph.h"
#include "astar.h"

using namespace std;

#define PI 3.14159265

// print all adjacent vertices of given vertex
void display_AdjList(adjNode* ptr, int i)
{
    while (ptr != nullptr) {
        cout << "(" << i << ", " << ptr->vertex
            << ", " << ptr->weight << ") ";
        ptr = ptr->next;
        //cout << i << endl;
    }
    cout << endl;
}
// graph implementation
int main()
{

    //------------------------- Part (a) ----------------------------
    // graph edges array.
    vector<graphEdge> edges = {
        // (x, y, w) -> edge from x to y with weight w
        //{0,1,2},{0,2,4},{1,4,3},{2,3,2},{3,1,4},{4,3,3}
        {0,1,1,3},{0,2,1,2},{0,3,1,3},{1,4,2,3},{1,5,2,1},{1,6,4,3},
        {2,7,5,2},{2,8,2,1},{2,9,3,2},{3,10,2,3},{3,11,2,2},{3,12,2,3},
        {5,13,5,0},{7,13,8,0},{9,13,6,0},{11,13,4,0}
    };
    int N = 14; // Number of vertices in the graph
    // calculate number of edges
    int n = edges.size();
    // construct graph
    
    Graph diagraph;
    diagraph.createGraph(edges, n, N);
    //static const adjNode ** ptr = diagraph.head;
    //for (int i = 1; i < N; i++)
    //{
        // display adjacent vertices of vertex i
        //display_AdjList(diagraph.head[i], i);
        //cout << diagraph.nodes[i].vertex << "->" << diagraph.nodes[i].parent->vertex << ", ";
    //}
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