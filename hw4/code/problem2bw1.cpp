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
    double r = 2;
    vector<int> x_bound{-1,13};
    vector<int> y_bound{-1,13};
    Point qstart{0,0};
    Point qgoal{10,10};
    Prm prm_planner(n,r,x_bound,y_bound,qstart,qgoal);
    int num_obstacles = 5;
    // retrieve obstacles
    vector<Point> obstacle1{{1,1},{2,1},{2,5},{1,5}};
    vector<Point> obstacle2{{3,4},{4,4},{4,12},{3,12}};
    vector<Point> obstacle3{{3,12},{12,12},{12,13},{3,13}};
    vector<Point> obstacle4{{12,5},{13,5},{13,13},{12,13}};
    vector<Point> obstacle5{{6,5},{12,5},{12,6},{6,6}};
    prm_planner.retrieveObstacles(num_obstacles, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5);
    prm_planner.createPrm();
    
    // Create Graph
    Graph graph;
    graph.createGraph(prm_planner.edges, prm_planner.edges.size(), prm_planner.prm_verts.size());
  
    // Write the prm verts to a csv file
    ofstream verts_file;
    verts_file.open ("data/prm2b_verts.csv");
    for(int i=0; i<prm_planner.prm_verts.size(); i++){
        verts_file << prm_planner.prm_verts[i].x << "," << prm_planner.prm_verts[i].y <<"\n";
    }
    verts_file.close();

    // Write the prm edges to a csv file
    ofstream edges_file;
    edges_file.open ("data/prm2b_edges.csv");
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
    path_file.open ("data/prm2b_path.csv");
    path_file << astar.path_length <<"\n";   
    for(int i=0; i<path.size(); i++){
        path_file << path[path.size()-i-1].vertex <<"\n";
    }
    path_file.close();

    return 0;
}