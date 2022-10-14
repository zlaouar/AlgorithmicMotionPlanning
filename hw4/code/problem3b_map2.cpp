#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <chrono>
#include "graph.h"
#include "astar.h"
#include "rrt.h"
#include "cobstacle.h"
#include "potentials.h"

using namespace std;

// graph implementation
int main()
{
    //------------------------- Part (b) ----------------------------
    int n = 5000;
    double r = 0.5;
    double e = 0.25;
    double pgoal = 0.05;
    vector<int> x_bound{-1,13};
    vector<int> y_bound{-1,13};
    Point qstart{0,0};
    Point qgoal{10,10};
    RRT rrt_planner(n,r,e,pgoal,x_bound,y_bound,qstart,qgoal);
    int num_obstacles = 5;
    // retrieve obstacles
    vector<Point> obstacle1{{1,1},{2,1},{2,5},{1,5}};
    vector<Point> obstacle2{{3,4},{4,4},{4,12},{3,12}};
    vector<Point> obstacle3{{3,12},{12,12},{12,13},{3,13}};
    vector<Point> obstacle4{{12,5},{13,5},{13,13},{12,13}};
    vector<Point> obstacle5{{6,5},{12,5},{12,6},{6,6}};
    rrt_planner.retrieveObstacles(num_obstacles, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5);
    
    
    int num_runs = 100;
    
    vector<double> path_len_vec;
    vector<bool> valid_path_vec;
    vector<double> comp_time_vec;
    
    
    for(int j=0; j<num_runs; j++){
        // Get starting timepoint 
        auto start = std::chrono::high_resolution_clock::now(); 

        rrt_planner.createRRT();
        
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 
        
        path_len_vec.push_back((rrt_planner.path.size()-1)*r);
        valid_path_vec.push_back(rrt_planner.valid_path);
        comp_time_vec.push_back(duration.count());
        cout << (rrt_planner.path.size()-1)*r << endl;
        rrt_planner.rrt_verts.clear();
        rrt_planner.edges.clear();
        rrt_planner.path.clear();
        rrt_planner.nodes.clear();
        
    }
    
    
    // Write the path lengths to a csv file
    ofstream path_file;
    path_file.open ("data/rrt3b2_path.csv");
    for(int i=0; i<path_len_vec.size(); i++){
        path_file << path_len_vec[i] <<"\n";
    }
    path_file.close();
    
    // Write the prm edges to a csv file
    ofstream valid_file;
    valid_file.open ("data/rrt3b2_valid.csv");
    for(int i=0; i<valid_path_vec.size(); i++){
        valid_file << valid_path_vec[i] <<"\n";
    }
    valid_file.close();
    
    // Write the prm path to a csv file
    ofstream comp_file;
    comp_file.open ("data/rrt3b2_comp.csv");
    for(int i=0; i<comp_time_vec.size(); i++){
        comp_file << comp_time_vec[i] <<"\n";
    }
    comp_file.close();
    

    return 0;
}