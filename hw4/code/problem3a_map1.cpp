#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include "graph.h"
#include "astar.h"
#include "rrt.h"
#include "cobstacle.h"
#include "potentials.h"

using namespace std;

// Goal Bias RRT implementation
int main()
{
    //------------------------- Part (a) ----------------------------
    int n = 5000;
    double r = 0.5;
    double e = 0.25;
    double pgoal = 0.05;
    vector<int> x_bound{-1,11};
    vector<int> y_bound{-3,3};
    Point qstart{0,0};
    Point qgoal{10,0};
    RRT rrt_planner(n,r,e,pgoal,x_bound,y_bound,qstart,qgoal);
    int num_obstacles = 2;
    // retrieve obstacles
    vector<Point> obstacle1{{3,2},{5,2},{5,0},{3,0}};
    vector<Point> obstacle2{{6,-2},{8,-2},{8,0},{6,0}};
    rrt_planner.retrieveObstacles(num_obstacles, obstacle1, obstacle2);
    rrt_planner.createRRT();
    
    
    // Write the prm verts to a csv file
    ofstream verts_file;
    verts_file.open ("data/rrt3a1_verts.csv");
    for(int i=0; i<rrt_planner.rrt_verts.size(); i++){
        verts_file << rrt_planner.rrt_verts[i].x << "," << rrt_planner.rrt_verts[i].y <<"\n";
    }
    verts_file.close();

    // Write the prm edges to a csv file
    ofstream edges_file;
    edges_file.open ("data/rrt3a1_edges.csv");
    for(int i=0; i<rrt_planner.edges.size(); i++){
        edges_file << rrt_planner.edges[i].start_vert << "," << rrt_planner.edges[i].end_vert << ","<< rrt_planner.edges[i].weight <<"\n";
    }
    edges_file.close();
    double path_length = (rrt_planner.path.size()-1)*r;
    cout << endl << "Path Length: " << path_length <<endl;
    
    // Write the prm path to a csv file
    ofstream path_file;
    path_file.open ("data/rrt3a1_path.csv");
    path_file << path_length <<"\n";
    for(int i=0; i<rrt_planner.path.size(); i++){
        path_file << rrt_planner.path[i] <<"\n";
    }
    path_file.close();

    return 0;
}