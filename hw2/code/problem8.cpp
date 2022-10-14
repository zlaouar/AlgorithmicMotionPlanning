#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include "cobstacle.h"
#include <cstdarg>

using namespace std;

#define PI 3.14159265

int main(){
    // General Initializations
    int grid_disc;
    int link_length;
    int num_obstacles;
    Cobstacle cspace;

    
    // Case (a) --------------------------------------------------------------------------------
    grid_disc = 100;
    link_length = 1;
    
    num_obstacles = 1;
    vector<vector<double>> obstacle1{{0.25,0.25},{0,0.75},{-0.25,0.25}};
    
    
    vector<vector<int>> c_grid = cspace.genCpaceMap(num_obstacles+2,grid_disc,link_length,obstacle1);
    

    // Write the vertices of the cspace obstacle to a csv file
    ofstream cobstacle_file;
    cobstacle_file.open ("cspace_map1.csv");
    for(int i=0; i<c_grid.size(); i++){
        for(int j=0; j<c_grid.size(); j++){
            cobstacle_file << c_grid[i][j] << ",";
        } 
        cobstacle_file <<"\n";
    }
    cobstacle_file.close();
    
    
    // Case (b) --------------------------------------------------------------------------------
    grid_disc = 100;
    link_length = 1;
    
    num_obstacles = 2;
    obstacle1 = {{-0.25,1.1},{-0.25,2},{0.25,2},{0.25,1.1}};
    vector<vector<double>> obstacle2{{-2,-2},{-2,-1.8},{2,-1.8},{2,-2}};
    
    vector<vector<int>> c_grid_2 = cspace.genCpaceMap(num_obstacles+2,grid_disc,link_length,obstacle1,obstacle2);
    
    // Write the vertices of the cspace obstacle to a csv file
    ofstream ccobstacle_file_2;
    ccobstacle_file_2.open ("cspace_map2.csv");
    for(int i=0; i<c_grid_2.size(); i++){
        for(int j=0; j<c_grid_2.size(); j++){
            ccobstacle_file_2 << c_grid_2[i][j] << ",";
        } 
        ccobstacle_file_2 <<"\n";
    }
    ccobstacle_file_2.close();

    // Case (c) --------------------------------------------------------------------------------
    grid_disc = 100;
    link_length = 1;
    
    num_obstacles = 2;
    obstacle1 = {{-0.25,1.1},{-0.25,2},{0.25,2},{0.25,1.1}};
    obstacle2 = {{-2,-0.5},{-2,-0.3},{2,-0.3},{2,-0.5}};
    
    vector<vector<int>> c_grid_3 = cspace.genCpaceMap(num_obstacles+2,grid_disc,link_length,obstacle1,obstacle2);
    
    // Write the vertices of the cspace obstacle to a csv file
    ofstream ccobstacle_file_3;
    ccobstacle_file_3.open ("cspace_map3.csv");
    for(int i=0; i<c_grid_3.size(); i++){
        for(int j=0; j<c_grid_3.size(); j++){
            ccobstacle_file_3 << c_grid_3[i][j] << ",";
        } 
        ccobstacle_file_3 <<"\n";
    }
    ccobstacle_file_3.close();
    
    return 0;
}