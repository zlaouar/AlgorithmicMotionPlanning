#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include "potentials.h"
#include "cobstacle.h"

using namespace std;

#define PI 3.14159265

int main(){
    int grid_disc;
    int link_length;
    int num_obstacles;
   

    // 3 a) Map 1_______________________________________________________________________________
    // Retrieve obstacles

    grid_disc = 100;
    link_length = 1;
    num_obstacles = 2;
    Point qstart{0,0};
    Point qgoal{PI,0};
    
    qstart.x = floor(grid_disc*(qstart.x/(2*PI)));
    qstart.y = floor(grid_disc*(qstart.y/(2*PI)));
    qgoal.x = floor(grid_disc*(qgoal.x/(2*PI)));
    qgoal.y = floor(grid_disc*(qgoal.y/(2*PI)));
    Cobstacle cspace;
    Potentials planner(qstart, qgoal);

    cout << "start: " << qstart.x << "," << qstart.y << "    goal: " << qgoal.x << "," << qgoal.y << endl;
    //return 0;
    
    vector<vector<double>> obstacle1{{-0.25,1.1},{-0.25,2},{0.25,2},{0.25,1.1}};
    vector<vector<double>> obstacle2{{-2,-0.5},{-2,-0.3},{2,-0.3},{2,-0.5}};
    vector<vector<int>> map = cspace.genCpaceMap(num_obstacles+2,grid_disc,link_length,obstacle1,obstacle2);
    planner.map = map;
    // Generate path for gradient descent algorithm
    //planner.scale_factor = 4; // Set pixel width to 0.25
    //planner.discretizeMap();// Create discretized map with obstacles as 1s

    // Write the path of the robot to a csv file
    planner.gridH = grid_disc;
    planner.gridH = grid_disc;
    vector<Point> path = planner.waveFront();
    ofstream path_file;
    path_file.open ("path_4.csv");
    path_file << qstart.x << "," << qstart.y << "," << qgoal.x << "," << qgoal.y << endl;
    //path_file << planner.top << "," << planner.bottom << "," << planner.left << "," << planner.buffer << "," << planner.scale_factor << endl;
    for(int i=0; i<path.size(); i++){
        path_file << path[i].x << "," << path[i].y <<"\n";
    }
    path_file.close();

    // Write the vertices of the cspace obstacle to a csv file
    ofstream ccobstacle_file;
    ccobstacle_file.open ("cspace_map.csv");
    for(int i=0; i<map.size(); i++){
        for(int j=0; j<map.size(); j++){
            ccobstacle_file << map[i][j] << ",";
        } 
        ccobstacle_file <<"\n";
    }
    ccobstacle_file.close();
    
    return 0;
}