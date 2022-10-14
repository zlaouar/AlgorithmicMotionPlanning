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
    //------------------------- Part (a) ----------------------------
    int n = 200;
    double r = 1;
    vector<int> x_bound{-1,11};
    vector<int> y_bound{-3,3};
    Point qstart{0,0};
    Point qgoal{10,0};
    Prm prm_planner(n,r,x_bound,y_bound,qstart,qgoal);
    int num_obstacles = 2;
    // retrieve obstacles
    vector<Point> obstacle1{{3,2},{5,2},{5,0},{3,0}};
    vector<Point> obstacle2{{6,-2},{8,-2},{8,0},{6,0}};
    prm_planner.retrieveObstacles(num_obstacles, obstacle1, obstacle2);
    prm_planner.createPrm();
    
    // Create Graph
    Graph graph;
    graph.createGraph(prm_planner.edges, prm_planner.edges.size(), prm_planner.prm_verts.size());
    //cout << "done" << endl;
    /*for (int i = 1; i < prm_planner.prm_verts.size(); i++)
    {
        //display adjacent vertices of vertex i
        display_AdjList(graph.head[i], i);
        //cout << graph.nodes[i].vertex << "->" << graph.nodes[i].parent->vertex << ", ";
    }*/
    
    // Write the prm verts to a csv file
    ofstream verts_file;
    verts_file.open ("data/prm2a_verts.csv");
    for(int i=0; i<prm_planner.prm_verts.size(); i++){
        verts_file << prm_planner.prm_verts[i].x << "," << prm_planner.prm_verts[i].y <<"\n";
    }
    verts_file.close();

    // Write the prm edges to a csv file
    ofstream edges_file;
    edges_file.open ("data/prm2a_edges.csv");
    for(int i=0; i<prm_planner.edges.size(); i++){
        edges_file << prm_planner.edges[i].start_vert << "," << prm_planner.edges[i].end_vert << ","<< prm_planner.edges[i].weight <<"\n";
    }
    edges_file.close();

    
    

    Astar astar(graph.head, graph.nodes, 0, prm_planner.prm_verts.size() - 1);
    vector<graphNode> path = astar.astar_search();
    vector<graphNode> smooth_path = prm_planner.path_smooth(path, 50);
    
    //double path_length = 0;
    cout << "Path: ";
    for(int i=0; i<path.size(); i++){
        cout << path[i].vertex << ", ";//path[path.size()-i-1].vertex << ", ";
        //path_length += path[path.size()-i-1].weight;
    }
    cout <<endl<< "Path Total: ";
    for(int i=0; i<path.size(); i++){
        //path_length += path[path.size()-i-1].weight;
        cout << path[i].weight << ", ";
        //path_length += path[path.size()-i-1].weight;
    }
    /*
    cout << endl << "Lengths: ";
    for(int i=0; i<path.size(); i++){
        cout << path[path.size()-i-1].weight << ", ";
        //path_length += path[path.size()-i-1].weight;
    }
    */
    cout << endl << "Path Length: " <<astar.path_length <<endl;
    cout << endl << "Smooth Path Length: " <<prm_planner.smooth_path_length <<endl;
    cout << "Num Iterations: " << astar.num_iterations << endl;

    // Write the prm path to a csv file
    ofstream path_file;
    path_file.open ("data/prm2a_path.csv");
    path_file << astar.path_length <<"\n";   
    for(int i=0; i<path.size(); i++){
        path_file << path[i].vertex <<"\n";
    }
    path_file.close();

    // Write the prm smooth path to a csv file
    ofstream smooth_path_file;
    smooth_path_file.open ("data/prm2a_smooth_path.csv");
    smooth_path_file << prm_planner.smooth_path_length <<"\n";   
    for(int i=0; i<path.size(); i++){
        smooth_path_file << smooth_path[i].vertex <<"\n";
    }
    smooth_path_file.close();
    return 0;
}