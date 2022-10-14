#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include "graph.h"
#include "astar.h"
#include "prm.h"
#include "cobstacle.h"
#include "potentials.h"

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
    //------------------------- Part (b) ----------------------------
    int n = 200;
    double r = 3;
    vector<int> x_bound{-6,36};
    vector<int> y_bound{-6,6};
    Point qstart{0,0};
    Point qgoal{35,0};
    Prm prm_planner(n,r,x_bound,y_bound,qstart,qgoal);
    int num_obstacles = 9;
    // retrieve obstacles
    vector<Point>obstacle1{{-6,-6},{25,-6},{25,-5},{-6,-5}};
    vector<Point>obstacle2{{-6,5},{30,5},{30,6},{-6,6}};
    vector<Point>obstacle3{{-6,-5},{-5,-5},{-5,5},{-6,5}};
    vector<Point>obstacle4{{4,-5},{5,-5},{5,1},{4,1}};
    vector<Point>obstacle5{{9,0},{10,0},{10,5},{9,5}};
    vector<Point> obstacle6{{14,-5},{15,-5},{15,1},{14,1}};
    vector<Point> obstacle7{{19,0},{20,0},{20,5},{19,5}};
    vector<Point> obstacle8{{24,-5},{25,-5},{25,1},{24,1}};
    vector<Point> obstacle9{{29,0},{30,0},{30,5},{29,5}};
    prm_planner.retrieveObstacles(num_obstacles, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7, obstacle8, obstacle9);
    prm_planner.createPrm();
    
    // Create Graph
    Graph graph;
    graph.createGraph(prm_planner.edges, prm_planner.edges.size(), prm_planner.prm_verts.size());
  
    // Write the prm verts to a csv file
    ofstream verts_file;
    verts_file.open ("data/prm2b1_verts.csv");
    for(int i=0; i<prm_planner.prm_verts.size(); i++){
        verts_file << prm_planner.prm_verts[i].x << "," << prm_planner.prm_verts[i].y <<"\n";
    }
    verts_file.close();

    // Write the prm edges to a csv file
    ofstream edges_file;
    edges_file.open ("data/prm2b1_edges.csv");
    for(int i=0; i<prm_planner.edges.size(); i++){
        edges_file << prm_planner.edges[i].start_vert << "," << prm_planner.edges[i].end_vert << ","<< prm_planner.edges[i].weight <<"\n";
    }
    edges_file.close();


    Astar astar(graph.head, graph.nodes, 0, prm_planner.prm_verts.size() - 1);
    vector<graphNode> path = astar.astar_search();
    
    
    cout << endl << "Path Length: " <<astar.path_length <<endl;
    cout << "Num Iterations: " << astar.num_iterations << endl;

    // Write the prm path to a csv file
    ofstream path_file;
    path_file.open ("data/prm2b1_path.csv");
    path_file << astar.path_length <<"\n";   
    for(int i=0; i<path.size(); i++){
        path_file << path[path.size()-i-1].vertex <<"\n";
    }
    path_file.close();

    return 0;
}