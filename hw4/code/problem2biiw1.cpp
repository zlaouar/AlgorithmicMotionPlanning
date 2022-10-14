#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <chrono>
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
    }
    cout << endl;
}

// graph implementation
int main()
{
    //------------------------- Part (a) ----------------------------
    vector<int> nvec{200,200,500,500,1000,1000};
    vector<double> rvec{1,2,1,2,1,2};
    vector<int> x_bound{-1,13};
    vector<int> y_bound{-1,13};
    Point qstart{0,0};
    Point qgoal{10,10};
    Prm prm_planner(nvec[0],rvec[0],x_bound,y_bound,qstart,qgoal);
    int num_obstacles = 5;
    // retrieve obstacles
    vector<Point> obstacle1{{1,1},{2,1},{2,5},{1,5}};
    vector<Point> obstacle2{{3,4},{4,4},{4,12},{3,12}};
    vector<Point> obstacle3{{3,12},{12,12},{12,13},{3,13}};
    vector<Point> obstacle4{{12,5},{13,5},{13,13},{12,13}};
    vector<Point> obstacle5{{6,5},{12,5},{12,6},{6,6}};
    prm_planner.retrieveObstacles(num_obstacles, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5);
    vector<graphNode> nodes;
    vector<graphEdge> edges;
    adjNode **head;
    Astar astar(head, nodes, 0, 0);
   
    vector<graphNode> path; 
    int num_runs = 100;
    
    vector<vector<double>> path_len_vec(nvec.size(), vector<double> (num_runs, 0));
    vector<vector<int>> valid_path_vec(nvec.size(), vector<int> (num_runs, 0));
    vector<vector<double>> comp_time_vec(nvec.size(), vector<double> (num_runs, 0));
    Graph graph;
    graph.createGraph(edges, 0, 0);
    
    for(int i=0; i<nvec.size(); i++){
        prm_planner.n = nvec[i];
        prm_planner.r = rvec[i];
        
        for(int j=0; j<num_runs; j++){
            // Get starting timepoint 
            auto start = std::chrono::high_resolution_clock::now(); 
  
            prm_planner.createPrm();  
            
            graph.deleteGraph();
            graph.createGraph(prm_planner.edges, prm_planner.edges.size(), prm_planner.prm_verts.size());
            
            astar.vertPointers = graph.head;
            
            astar.graphNodes = graph.nodes;
          
            astar.goal = prm_planner.prm_verts.size() - 1;
            path = astar.astar_search();

            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 

            path_len_vec[i][j] = astar.path_length;
            valid_path_vec[i][j] = astar.valid_path;
            comp_time_vec[i][j] = duration.count();
            
            astar.clear();
            prm_planner.prm_verts.clear();
            prm_planner.edges.clear();
        }
        cout << "Step finished: " << i << endl;
    }
  
    // Write the path lengths to a csv file
    ofstream path_file;
    path_file.open ("data/prm2biiw1_path.csv");
    for(int i=0; i<path_len_vec.size(); i++){
        for(int j=0; j<path_len_vec[0].size(); j++){
            path_file << path_len_vec[i][j] << ",";
        }
        path_file <<"\n";
    }
    path_file.close();
    
    // Write the prm edges to a csv file
    ofstream valid_file;
    valid_file.open ("data/prm2biiw1_valid.csv");
    for(int i=0; i<valid_path_vec.size(); i++){
        for(int j=0; j<valid_path_vec[0].size(); j++){
            valid_file << valid_path_vec[i][j] << ",";
        }
        valid_file <<"\n";
    }
    valid_file.close();
    
    // Write the prm path to a csv file
    ofstream comp_file;
    comp_file.open ("data/prm2biiw1_comp.csv");
    for(int i=0; i<comp_time_vec.size(); i++){
        for(int j=0; j<comp_time_vec[0].size(); j++){
            comp_file << comp_time_vec[i][j] << ",";
        }
        comp_file <<"\n";
    }
    comp_file.close();
    

    return 0;
}